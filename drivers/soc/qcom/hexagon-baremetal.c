#include "linux/pm.h"
#include "linux/types.h"
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset-controller.h>
#include <linux/firmware.h>
#include <linux/miscdevice.h>
#include <linux/notifier.h>
#include <linux/scatterlist.h>
#include <linux/dma-buf.h>
#include <linux/interrupt.h>
#include <linux/panic_notifier.h>
#include <linux/poll.h>
#include <linux/interconnect.h>
#include <dt-bindings/mailbox/qcom-ipcc.h>
#include <linux/iommu.h>
#include <linux/mailbox_client.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>
#include <linux/pm_runtime.h>

#define QDSP6SS_VERSION 0x00

#define QDSP6SS_RST_EVB 0x10

#define QDSP6SS_CORE_CBCR 0x48040
#define QDSP6SS_CORE_CMD_RCGR 0x48020
#define QDSP6SS_CORE_CFG_RCGR 0x48024
#define QDSP6SS_XO_CBCR 0x48120 // ACD_XO
#define QDSP6SS_SLEEP_CBCR 0x48060 // XXX

#define QDSP6SS_BOOT_CORE_START 0x400
#define QDSP6SS_BOOT_CMD 0x404
#define QDSP6SS_BOOT_STATUS 0x408

#define QDSP6SS_INTF_HALTREQ 0x2088
#define QDSP6SS_INTF_HALTACK 0x208c

#define QDSP6SS_RESET_CFG 0x2440
#define QDSP6SS_RESET_CMD 0x2444
#define QDSP6SS_RESET_STATUS 0x2448

#define QDSP6SS_QDSP6V67SS_PLL 0x40000

#define QDSP6V67SS_BOOT_PARAMS0 0x60
#define DSP_APP_DEV_STATE (QDSP6V67SS_BOOT_PARAMS0 + 4)

#define DSP_APP_DEV_RESUMING  BIT(0)
#define DSP_APP_DEV_SUSPENDED BIT(1)
#define DSP_ADSP_DEV_SUSPENDED BIT(2)

#define DSP_ADSP_DEV_SUSPENDED_RETRY 10
#define DSP_ADSP_DEV_SUSPENDED_DELAY_MS 40

struct dsp {
	struct miscdevice miscdev;
	wait_queue_head_t waitqueue;
	atomic_t pending;
	spinlock_t lock;
	struct list_head maps;
	atomic_t open, booted;
	struct iommu_domain *iommu;
	struct mbox_client mbox_client;
	struct mbox_chan *mbox_chan[4];
	void __iomem *base, *evb_sel;
	struct clk_bulk_data *clks;
	unsigned num_clks;
	bool is_xtensa;
	bool is_adsp;
	bool is_cdsp;
	u32 mem_phys;
	u32 mem_size;
	struct device *pds[4];
	int perf[4];
	struct notifier_block dsp_notifier;
	struct icc_path *dma_path;
};

struct dma_map {
	struct list_head node;
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *table;
	u32 iova;
	u32 size;
};

struct map_direct {
	u32 va;
	u32 pa;
	u32 size;
};


#define DSP_IOCTL_IRQ_STATUS 	_IO('C', 0) /* returns mask of pending ipcc signals and clears */
/* note: dsp fd will have POLLIN event if there are any pending ipcc signals */
#define DSP_IOCTL_IRQ_SEND  	_IO('C', 1) /* send an ipcc signal, argument is integer in 0-3 range */
#define DSP_IOCTL_BOOT 		_IO('C', 2) /* boot the DSP. argument is address to boot at */
/* note: for xtensa boot address is ignored (always 0) */
#define DSP_IOCTL_SHUTDOWN 	_IO('C', 3) /* shutdown DSP. no argument */
#define DSP_IOCTL_MAP_DMABUF 	_IO('C', 4) /* 64-bit argument: low 32-bits dmabuf fd, high 32-bits VA */
#define DSP_IOCTL_MAP_DIRECT 	_IOW('C', 5, struct map_direct)
#define DSP_IOCTL_UNMAP 	_IO('C', 6) /* 32 bit va argument */
#define DSP_IOCTL_HANDOVER 	_IO('C', 8) /* Signal to the driver the dsp has started, can unvote resources */

// Dedicated ioctl for cvp boot to set registers inside kernel instead of userspace + using DSP_IOCTL_BOOT
#define CVP_IOCTL_BOOT 	_IO('C', 7) /* argument ignored */

#define write32(x, y) writel(y, x)

#define PLL_MODE           0x00
#if 0
#define PLL_ACTIVE_FLAG    BIT(30)
#define PLL_ACK_LATCH      BIT(29)
#define PLL_UPDATE         BIT(22)
#define PLL_VOTE_FSM_RESET BIT(21)
#define PLL_VOTE_FSM_ENA   BIT(20)
#define PLL_RESET_N        BIT(2)
#endif

#define PLL_OPMODE 0x04
#define PLL_STATE 0x08
#define PLL_STATUS 0x0c
#define PLL_L_VAL 0x10
#define PLL_ALPHA_VAL 0x14
#define PLL_USER_CTL 0x18
#define PLL_USER_CTL_U 0x1c
#define PLL_CONFIG_CTL 0x20
#define PLL_CONFIG_CTL_U 0x24
#define PLL_CONFIG_CTL_U1 0x28
#define PLL_TEST_CTL 0x2c
#define PLL_TEST_CTL_U 0x30
#define PLL_TEST_CTL_U1 0x34
#define PLL_TEST_CTL_U2 0x38
#define PLL_SSC 0x3c

static inline void
clk_pll_configure_lucid_ole(void __iomem *pll_reg, u32 freq)
{
   u16 l = freq / 19200000;
   u16 alpha = (freq % 19200000) * 32 / 9375;

   write32(pll_reg + PLL_MODE, 0x00);

   // Add RINGOSC_CAL_L extra shift?
   write32(pll_reg + PLL_L_VAL, 0x44 << 24 | 0x44 << 16 | l);
   write32(pll_reg + PLL_ALPHA_VAL, alpha);
   write32(pll_reg + PLL_USER_CTL, 0x00008400 | 1);
   write32(pll_reg + PLL_USER_CTL_U, 0x00000005);
   write32(pll_reg + PLL_CONFIG_CTL, 0x20485699);
   write32(pll_reg + PLL_CONFIG_CTL_U, 0x00182261);
   write32(pll_reg + PLL_CONFIG_CTL_U1, 0x82AA299C);
   write32(pll_reg + PLL_TEST_CTL, 0);
   write32(pll_reg + PLL_TEST_CTL_U, 0x00000003);
   write32(pll_reg + PLL_TEST_CTL_U1, 0x00009000);
   write32(pll_reg + PLL_TEST_CTL_U2, 0x00000034);

#define PLL_UPDATE_BYPASS	BIT(23)
	write32(pll_reg + PLL_MODE, PLL_UPDATE_BYPASS);

#define PLL_STANDBY		0x0
#define PLL_RUN			0x1

#define PLL_LOCK_DET BIT(31)
#define OUT_MAIN_HW_CTL BIT(3)
#define PLL_RESET_N  BIT(2)
#define PLL_OUTCTRL  BIT(0)

	write32(pll_reg + PLL_OPMODE, PLL_STANDBY);
	write32(pll_reg + PLL_MODE, PLL_RESET_N | PLL_UPDATE_BYPASS);

	write32(pll_reg + PLL_OPMODE, PLL_RUN);

	//usleep_range(1000, 2000);
	//printk("PLL_MODE= 0x%.8x\n", readl(pll_reg + PLL_MODE));
	while ((readl(pll_reg + PLL_MODE) & PLL_LOCK_DET) == 0);

	write32(pll_reg + PLL_MODE, PLL_RESET_N | PLL_OUTCTRL);// | OUT_MAIN_HW_CTL);
}

static int
xtensa_boot(struct dsp *dsp)
{
	// Can this be called outside of booting the CVP?
	bool is_cvp = true;

#define CVP_SS_BASE_OFFS			0x0
#define CVP_SS_NUM_THREADS			4

#define CVP_SS_IRQ_STATUS			(CVP_SS_BASE_OFFS + 0x0)
#define CVP_SS_WD_IRQ_CLR			(CVP_SS_BASE_OFFS + 0x3c)
#define CVP_SS_THREAD_DONE_CLR(x)	(CVP_SS_BASE_OFFS + 0x514 + 0x40 * (x))

#define CVP_CPU_BASE_OFFS			0x000A0000

#define CPU_CS_A2HSOFTINTEN		(CVP_CPU_BASE_OFFS + 0x10)
#define CPU_CS_A2HSOFTINTENCLR	(CVP_CPU_BASE_OFFS + 0x14)
#define CPU_CS_A2HSOFTINT		(CVP_CPU_BASE_OFFS + 0x18)
#define CPU_CS_A2HSOFTINTCLR	(CVP_CPU_BASE_OFFS + 0x1C)
#define CPU_CS_VMIMSG		(CVP_CPU_BASE_OFFS + 0x34)
#define CPU_CS_VMIMSGAG0		(CVP_CPU_BASE_OFFS + 0x38)
#define CPU_CS_VMIMSGAG1		(CVP_CPU_BASE_OFFS + 0x3C)
#define CPU_CS_VMIMSGAG2		(CVP_CPU_BASE_OFFS + 0x40)
#define CPU_CS_VMIMSGAG3		(CVP_CPU_BASE_OFFS + 0x44)
#define CPU_CS_SCIACMD			(CVP_CPU_BASE_OFFS + 0x48)
#define CPU_CS_H2XSOFTINTEN		(CVP_CPU_BASE_OFFS + 0x148)

/* CVP_CTRL_STATUS */
#define CPU_CS_SCIACMDARG0		(CVP_CPU_BASE_OFFS + 0x4C)
#define CPU_CS_SCIACMDARG0_BMSK	0xff
#define CPU_CS_SCIACMDARG0_SHFT	0x0
#define CPU_CS_SCIACMDARG0_HFI_CTRL_ERROR_STATUS_BMSK	0xfe
#define CPU_CS_SCIACMDARG0_HFI_CTRL_ERROR_STATUS_SHFT	0x1
#define CPU_CS_SCIACMDARG0_HFI_CTRL_INIT_STATUS_BMSK	0x1
#define CPU_CS_SCIACMDARG0_HFI_CTRL_INIT_STATUS_SHFT	0x0
#define CPU_CS_SCIACMDARG0_HFI_CTRL_PC_READY           0x100

/* HFI_QTBL_INFO */
#define CPU_CS_SCIACMDARG1		(CVP_CPU_BASE_OFFS + 0x50)

/* HFI_QTBL_ADDR */
#define CPU_CS_SCIACMDARG2		(CVP_CPU_BASE_OFFS + 0x54)

/* HFI_VERSION_INFO */
#define CPU_CS_SCIACMDARG3		(CVP_CPU_BASE_OFFS + 0x58)

/* CVP_SFR_ADDR */
#define CPU_CS_SCIBCMD		(CVP_CPU_BASE_OFFS + 0x5C)

/* CVP_MMAP_ADDR */
#define CPU_CS_SCIBCMDARG0		(CVP_CPU_BASE_OFFS + 0x60)

/* CVP_UC_REGION_ADDR */
#define CPU_CS_SCIBARG1		(CVP_CPU_BASE_OFFS + 0x64)

/* CVP_UC_REGION_ADDR */
#define CPU_CS_SCIBARG2		(CVP_CPU_BASE_OFFS + 0x68)

#define CPU_CS_SCIBARG3		(CVP_CPU_BASE_OFFS + 0x6C)

#define CPU_CS_H2ASOFTINTEN		(CVP_CPU_BASE_OFFS + 0x148)
#define CPU_CS_H2ASOFTINTENCLR	(CVP_CPU_BASE_OFFS + 0x14c)
#define CPU_CS_H2ASOFTINT		(CVP_CPU_BASE_OFFS + 0x150)
#define CPU_CS_H2ASOFTINTCLR	(CVP_CPU_BASE_OFFS + 0x154)

/* FAL10 Feature Control */
#define CPU_CS_X2RPMh		(CVP_CPU_BASE_OFFS + 0x168)
#define CPU_CS_X2RPMh_MASK0_BMSK	0x1
#define CPU_CS_X2RPMh_MASK0_SHFT	0x0
#define CPU_CS_X2RPMh_MASK1_BMSK	0x2
#define CPU_CS_X2RPMh_MASK1_SHFT	0x1
#define CPU_CS_X2RPMh_SWOVERRIDE_BMSK	0x4
#define CPU_CS_X2RPMh_SWOVERRIDE_SHFT	0x3

/* WRAPPER */

#define CVP_WRAPPER_BASE_OFFS		0x000B0000

#define CVP_WRAPPER_HW_VERSION		(CVP_WRAPPER_BASE_OFFS + 0x00)
#define CVP_WRAPPER_HW_VERSION_MAJOR_VERSION_MASK  0x78000000
#define CVP_WRAPPER_HW_VERSION_MAJOR_VERSION_SHIFT 28
#define CVP_WRAPPER_HW_VERSION_MINOR_VERSION_MASK  0xFFF0000
#define CVP_WRAPPER_HW_VERSION_MINOR_VERSION_SHIFT 16
#define CVP_WRAPPER_HW_VERSION_STEP_VERSION_MASK   0xFFFF

#define CVP_WRAPPER_INTR_STATUS	(CVP_WRAPPER_BASE_OFFS + 0x0C)
#define CVP_WRAPPER_INTR_STATUS_A2HWD_BMSK	0x8
#define CVP_WRAPPER_INTR_STATUS_A2H_BMSK	0x4

#define CVP_WRAPPER_INTR_MASK		(CVP_WRAPPER_BASE_OFFS + 0x10)
#define CVP_FATAL_INTR_BMSK	(CVP_WRAPPER_INTR_MASK_CPU_NOC_BMSK | \
				CVP_WRAPPER_INTR_MASK_CORE_NOC_BMSK | \
				CVP_WRAPPER_INTR_MASK_A2HWD_BMSK)
#define CVP_WRAPPER_INTR_MASK_CPU_NOC_BMSK	0x40
#define CVP_WRAPPER_INTR_MASK_CORE_NOC_BMSK	0x20
#define CVP_WRAPPER_INTR_MASK_A2HWD_BMSK	0x8
#define CVP_WRAPPER_INTR_MASK_A2HCPU_BMSK	0x4
#define CVP_WRAPPER_INTR_MASK_A2HCPU_SHFT	0x2

#define CVP_WRAPPER_INTR_CLEAR		(CVP_WRAPPER_BASE_OFFS + 0x14)
#define CVP_WRAPPER_TZ_BASE_OFFS		0x000C0000
#define CVP_WRAPPER_INTR_CLEAR_A2HWD_BMSK	0x10
#define CVP_WRAPPER_INTR_CLEAR_A2HWD_SHFT	0x4
#define CVP_WRAPPER_INTR_CLEAR_A2H_BMSK	0x4
#define CVP_WRAPPER_INTR_CLEAR_A2H_SHFT	0x2
#define CVP_WRAPPER_CPU_STATUS		(CVP_WRAPPER_TZ_BASE_OFFS + 0x10)
#define CVP_WRAPPER_CPU_CGC_DIS	(CVP_WRAPPER_BASE_OFFS + 0x2010)

#define CVP_WRAPPER_CPU_CLOCK_CONFIG	(CVP_WRAPPER_BASE_OFFS + 0x50)
#define CVP_WRAPPER_DEBUG_BRIDGE_LPI_CONTROL	(CVP_WRAPPER_BASE_OFFS + 0x54)
#define CVP_WRAPPER_DEBUG_BRIDGE_LPI_STATUS	(CVP_WRAPPER_BASE_OFFS + 0x58)
#define CVP_WRAPPER_CORE_CLOCK_CONFIG		(CVP_WRAPPER_BASE_OFFS + 0x88)

	// Error recovery
	u32 intr_status = readl_relaxed(dsp->base + CVP_WRAPPER_INTR_STATUS);
	if(intr_status) {
		// We have pending IRQs from the last session, try to recover from potential errors here
		// Disable the cores power to make sure all per-core watchdog statuses are cleared
		usleep_range(1000, 2000);
		writel_relaxed(0x1, dsp->base + 0xb0084); // VPU_WRAPPER_CORE_POWER_CONTROL
		usleep_range(1000, 2000);
		// No need to re-enable power, done later-on during boot

		// Clear double exception status
		if(readl_relaxed(dsp->base + CVP_WRAPPER_INTR_STATUS) & 0x2) {
			writel_relaxed(0x1, dsp->base + 0xb00c8); // VPU_WRAPPER_CPU_DOUBLEEXCEPTIONERROR_CLR
		}
		// Clear error logger errors (that's needed to clear CPU NOC error status after smmu context fault)
		if(readl_relaxed(dsp->base + 0x1a010)) {
			writel_relaxed(0x1, dsp->base + 0x1a018); // MAIN_ERRORLOGGER_ERRCLR_LOW
		}
		if(readl_relaxed(dsp->base + 0xd0010)) {
			writel_relaxed(0x1, dsp->base + 0xd0018); // IRIS_CPU_NOC_ERRORLOGGER_ERRCLR_LOW
			writel_relaxed(0x1, dsp->base + 0xd0018); // needs to be set twice... otherwise doesn't take effect
		}

		// Clear interrupt/watchdog on the CVP_SS block if the SS_INTR bit is set (happens after invalid iova access issued by hardwares)
		if(readl_relaxed(dsp->base + CVP_WRAPPER_INTR_STATUS) & 0x10) {
			// Assert clear signal on all THREAD_DONE and watchdog IRQ registers, then deassert after a few ms
			writel_relaxed(0xf, dsp->base + CVP_SS_WD_IRQ_CLR);
			for(int i=0; i<CVP_SS_NUM_THREADS; i++)
				writel_relaxed(0x1, dsp->base + CVP_SS_THREAD_DONE_CLR(i));

			usleep_range(1000, 2000);

			writel_relaxed(0x0, dsp->base + CVP_SS_WD_IRQ_CLR);
			for(int i=0; i<CVP_SS_NUM_THREADS; i++)
				writel_relaxed(0x0, dsp->base + CVP_SS_THREAD_DONE_CLR(i));
		}

		u32 post_recovery_status = readl_relaxed(dsp->base + CVP_WRAPPER_INTR_STATUS);
		dev_warn(dsp->miscdev.parent, "CVP error recovery | Initial status: 0x%x | Final status: 0x%x\n", intr_status, post_recovery_status);
		if(post_recovery_status)
		{
			dev_err(dsp->miscdev.parent, "CVP error recovery failed. Status: 0x%x\n", post_recovery_status);
		}
	}

	u32 fw_size = 8 * SZ_1M;
	// qtlb buffer
	u32 addr = 0x80000000;

	// Corresponds to SHARED_QSIZE which is 3Mb
	u32 size = SZ_1M * 3;
	u32 mask_val, count, status;

	if (is_cvp)
		fw_size = 7 * SZ_1M;

	// Make sure host->xtensa and xtensa->host interrupt status are cleared before booting
	// since these prevent the CVP firmware from booting successfully with status 0x6400
	writel_relaxed(1, dsp->base + CPU_CS_A2HSOFTINTCLR);
	writel_relaxed(1, dsp->base + CPU_CS_H2ASOFTINTCLR);

	dev_pm_genpd_set_performance_state(dsp->pds[0], dsp->perf[0]);
	dev_pm_genpd_set_performance_state(dsp->pds[1], dsp->perf[1]);

#define WRAPPER_CPA_START_ADDR 		0xc1020
#define WRAPPER_CPA_END_ADDR 		0xc1024
#define WRAPPER_FW_START_ADDR 		0xc1028
#define WRAPPER_FW_END_ADDR 		0xc102c
#define WRAPPER_NONPIX_START_ADDR 	0xc1030
#define WRAPPER_NONPIX_END_ADDR 	0xc1034

	writel_relaxed(0, dsp->base + WRAPPER_FW_START_ADDR);
	writel_relaxed(fw_size, dsp->base + WRAPPER_FW_END_ADDR);
	writel(0, dsp->base + WRAPPER_CPA_START_ADDR);
	if (is_cvp) {
		writel(0x4b000000, dsp->base + WRAPPER_CPA_END_ADDR);
		writel(0x01000000, dsp->base + WRAPPER_NONPIX_START_ADDR);
		writel(0x26800000, dsp->base + WRAPPER_NONPIX_END_ADDR);
	} else {
		writel(fw_size, dsp->base + WRAPPER_CPA_END_ADDR);
		writel(fw_size, dsp->base + WRAPPER_NONPIX_START_ADDR);
		writel(fw_size, dsp->base + WRAPPER_NONPIX_END_ADDR);
	}

	writel_relaxed(0, dsp->base + 0xB0088);

	writel_relaxed(0, dsp->base + CPU_CS_SCIACMDARG0);

	writel(addr, dsp->base + CPU_CS_SCIBARG1); // UC_REGION_ADDR
	writel(size, dsp->base + CPU_CS_SCIBARG2); // UC_REGION_SIZE
	writel(addr, dsp->base + CPU_CS_SCIACMDARG2); // QTBL_ADDR
	writel(0x01, dsp->base + CPU_CS_SCIACMDARG1); // QTBL_INFO
	writel(addr + size, dsp->base + CPU_CS_SCIBCMD);

	if (is_cvp) {
		// XXX firmware unhappy without DSP queues..
		addr += size + SZ_1M;
		writel(addr, dsp->base + CPU_CS_VMIMSG); // DSP_QTBL_ADDR
		writel(addr, dsp->base + CPU_CS_VMIMSGAG0); // DSP_UC_REGION_ADDR
		writel(size, dsp->base + CPU_CS_VMIMSGAG1); // DSP_UC_REGION_SIZE
	}

	writel(BIT(0) | BIT(3), dsp->base + CPU_CS_SCIACMD);

#define WRAPPER_INTR_MASK  0xb0010
#define CPUSS_WD_INTR   BIT(3)
#define CPUSS_INTR      BIT(2)


	// mask_val = readl_relaxed(dsp->base + CVP_WRAPPER_INTR_MASK);
	// mask_val &= ~(8|4);//~(CVP_FATAL_INTR_BMSK | CVP_WRAPPER_INTR_MASK_A2HCPU_BMSK);
	//writel_relaxed(mask_val, dsp->base + CVP_WRAPPER_INTR_MASK);

	mask_val = readl_relaxed(dsp->base + CVP_WRAPPER_INTR_MASK);
	mask_val &= ~(8|4);//~(CVP_FATAL_INTR_BMSK | CVP_WRAPPER_INTR_MASK_A2HCPU_BMSK);
	writel_relaxed(mask_val, dsp->base + CVP_WRAPPER_INTR_MASK);

	//writel_relaxed(~(CPUSS_WD_INTR|CPUSS_INTR), dsp->base + CVP_WRAPPER_INTR_MASK);


	// busybox devmem 0xab00004 32 0x000D000F
   	// trigger FW init (fw waits for this)
	writel_relaxed(1, dsp->base + CPU_CS_SCIACMDARG3);
	count = 1000;
	do {
		status = readl_relaxed(dsp->base + CPU_CS_SCIACMDARG0);
		if (status & BIT(0))
			break;
		usleep_range(500, 1000);
	} while (--count);

	if(!count)
	{
		dev_err(dsp->miscdev.parent, "CVP boot failed with boot status 0x%x (interrupt status: 0x%x) \n", status, readl_relaxed(dsp->base + CVP_WRAPPER_INTR_STATUS));
	}

	writel_relaxed(1, dsp->base + CPU_CS_H2XSOFTINTEN);
	writel_relaxed(0, dsp->base + CPU_CS_X2RPMh);

	writel_relaxed(0, dsp->base + CVP_WRAPPER_BASE_OFFS + 0x84); // WRAPPER_CORE_POWER_CONTROL.CORE_PWR_DISABLE=0
	return status;
}

static int dsp_boot(struct dsp *dsp, u32 boot_addr)
{
	if (dsp->evb_sel)
		writel(0, dsp->evb_sel);

	// Only try to set src to QDSP6SSPLL for cDSP on 8650
	if (dsp->is_cdsp) {
		icc_set_bw(dsp->dma_path, 0, GBps_to_icc(1));
		dev_pm_genpd_set_performance_state(dsp->pds[0], dsp->perf[0]);
		pm_runtime_get_sync(dsp->pds[0]);
		dev_pm_genpd_set_performance_state(dsp->pds[1], dsp->perf[1]);
		pm_runtime_get_sync(dsp->pds[1]);
		dev_pm_genpd_set_performance_state(dsp->pds[2], dsp->perf[2]);
		pm_runtime_get_sync(dsp->pds[2]);
		dev_pm_genpd_set_performance_state(dsp->pds[3], dsp->perf[3]);
		pm_runtime_get_sync(dsp->pds[3]);

		writel(0x200, dsp->base + QDSP6SS_CORE_CFG_RCGR);
		writel(1, dsp->base + QDSP6SS_CORE_CMD_RCGR);

		writel(1, dsp->base + QDSP6SS_XO_CBCR);
		writel(1, dsp->base + QDSP6SS_SLEEP_CBCR);
		writel(1, dsp->base + QDSP6SS_CORE_CBCR);

		usleep_range(1000, 2000); // XXX
	}

	if (dsp->is_adsp) {
		icc_set_bw(dsp->dma_path, 0, UINT_MAX);
		dev_pm_genpd_set_performance_state(dsp->pds[0], dsp->perf[0]);
		pm_runtime_get_sync(dsp->pds[0]);
		dev_pm_genpd_set_performance_state(dsp->pds[1], dsp->perf[1]);
		pm_runtime_get_sync(dsp->pds[1]);
	}

	writel(boot_addr >> 4, dsp->base + QDSP6SS_RST_EVB);
	writel(1, dsp->base + QDSP6SS_BOOT_CORE_START);
	usleep_range(1000, 2000); // XXX
	writel(1, dsp->base + QDSP6SS_BOOT_CMD);

	usleep_range(1000, 2000);

	return readl(dsp->base + QDSP6SS_BOOT_STATUS);
}

static void dsp_shutdown_low_level_atomic(struct dsp *dsp)
{
	writel(7, dsp->base + QDSP6SS_INTF_HALTREQ);
	printk("haltack=%d\n", readl(dsp->base + QDSP6SS_INTF_HALTACK));
	writel(0, dsp->base + QDSP6SS_INTF_HALTREQ);

	writel(~0u, dsp->base +  QDSP6SS_RESET_CFG);
	writel(1, dsp->base +  QDSP6SS_RESET_CMD);
	writel(0, dsp->base +  QDSP6SS_RESET_CFG);
}

static int dsp_shutdown(struct dsp *dsp)
{
	// XXX possible race condition if dsp_shutdown called twice simultaneously..
	if (atomic_read(&dsp->booted) == 0)
		return -EBUSY;

	if (dsp->is_xtensa) {
		// Make sure the following is set so we know it's safe to reset cores and remove dma mappings
		//	- IDLE status: BIT(30) in CPU_CS_SCIACMDARG0
		//	- CPU Wait For Interrupt: BIT(0) in CVP_WRAPPER_CPU_STATUS
		// (this state may not be reached if the hardware enters an error state, for example after an invalid iova access)
		u32 retry_cnt = 0;
		const u32 max_retry_cnt = 20;
		while(retry_cnt < max_retry_cnt)
		{
			u32 cpu_status = readl_relaxed(dsp->base + CVP_WRAPPER_CPU_STATUS);
			u32 arg0 = readl_relaxed(dsp->base + CPU_CS_SCIACMDARG0);
			if((cpu_status & 0x1) && (arg0 & BIT(30)))
			{
				break;
			}
			retry_cnt++;
			usleep_range(1000, 2000);
		}
		if(retry_cnt == max_retry_cnt)
		{
			dev_warn(dsp->miscdev.parent, "CVP failed to enter idle state during shutdown. (CPU_STATUS=0x%x SCIACMDARG0=0x%x)",
				readl_relaxed(dsp->base + CVP_WRAPPER_CPU_STATUS), readl_relaxed(dsp->base + CPU_CS_SCIACMDARG0));
		}

		writel(1, dsp->base + 0xc1000); // VCODEC_VPU_WRAPPER_TZ_XTSS_SW_RESET

		// disable CPU watchdog while off
		writel(0, dsp->base + 0x90008); // CPU_WD_WDOGCONTROL

		dev_pm_genpd_set_performance_state(dsp->pds[0], 0);
		dev_pm_genpd_set_performance_state(dsp->pds[1], 0);
	} else {
		writel(7, dsp->base + QDSP6SS_INTF_HALTREQ);
		while (!readl(dsp->base + QDSP6SS_INTF_HALTACK)) usleep_range(1000,2000);
		printk("haltack=%d\n", readl(dsp->base + QDSP6SS_INTF_HALTACK));
		writel(0, dsp->base + QDSP6SS_INTF_HALTREQ);

		writel(~0u, dsp->base +  QDSP6SS_RESET_CFG);
		writel(1, dsp->base +  QDSP6SS_RESET_CMD);
		usleep_range(10000, 20000); // XXX
		writel(0, dsp->base +  QDSP6SS_RESET_CFG);

		if (dsp->is_cdsp)  {
			dev_pm_genpd_set_performance_state(dsp->pds[0], 0);
			pm_runtime_put(dsp->pds[0]);
			dev_pm_genpd_set_performance_state(dsp->pds[1], 0);
			pm_runtime_put(dsp->pds[1]);
			dev_pm_genpd_set_performance_state(dsp->pds[2], 0);
			pm_runtime_put(dsp->pds[2]);
			dev_pm_genpd_set_performance_state(dsp->pds[3], 0);
			pm_runtime_put(dsp->pds[3]);
			icc_set_bw(dsp->dma_path, 0, kBps_to_icc(1));
		}

		if (dsp->is_adsp) {
			dev_pm_genpd_set_performance_state(dsp->pds[0], 0);
			pm_runtime_put(dsp->pds[0]);
			dev_pm_genpd_set_performance_state(dsp->pds[1], 0);
			pm_runtime_put(dsp->pds[1]);
			icc_set_bw(dsp->dma_path, 0, kBps_to_icc(1));

		}
	}

	atomic_set(&dsp->booted, 0);
	return 0;
}

static int dsp_panic_nb(struct dsp *dsp)
{
	if (atomic_read(&dsp->booted)) {
		dsp_shutdown_low_level_atomic(dsp);
		atomic_set(&dsp->booted, 0);
	}

	return 0;
}

static int cdsp_panic_nb(struct notifier_block *this, unsigned long event,
		         void *ptr)
{
	struct dsp *cdsp = container_of(this, struct dsp, dsp_notifier);

	return dsp_panic_nb(cdsp);
}

static int adsp_panic_nb(struct notifier_block *this, unsigned long event,
		         void *ptr)
{
	struct dsp *adsp = container_of(this, struct dsp, dsp_notifier);

	return dsp_panic_nb(adsp);
}

static int dsp_handover(struct dsp *dsp)
{
	if (atomic_read(&dsp->booted) == 0)
		return -EINVAL;

	if (dsp->is_cdsp)  {
		// TODO
	}

	if (dsp->is_adsp) {
		dev_pm_genpd_set_performance_state(dsp->pds[0], 0);
		dev_pm_genpd_set_performance_state(dsp->pds[1], 0);
		icc_set_bw(dsp->dma_path, 0, 0);
	}

	return 0;
}


static int dsp_open(struct inode *inode, struct file *file)
{
	struct dsp *dsp = file->private_data;

	if (atomic_xchg(&dsp->open, 1)) /* only open one at a time */
		return -EPERM;
	return 0;
}

static void dsp_unmap(struct dsp *dsp, struct dma_map *map);

static int dsp_release(struct inode *inode, struct file *file)
{
	struct dsp *dsp = file->private_data;
	struct dma_map *map, *n;

	dsp_shutdown(dsp);

	/* no lock - ioctl() should not be ongoing when release() is called */
	list_for_each_entry_safe(map, n, &dsp->maps, node)
		dsp_unmap(dsp, map);
	INIT_LIST_HEAD(&dsp->maps);

	atomic_set(&dsp->open, 0);
	return 0;
}

static __poll_t dsp_poll(struct file *file, struct poll_table_struct *wait)
{
	struct dsp *dsp = file->private_data;

	poll_wait(file, &dsp->waitqueue, wait);

	if (atomic_read(&dsp->pending))
		return POLLIN | POLLRDNORM;
	return 0;
}

static long dsp_map_dmabuf(struct dsp *dsp, int fd, u32 iova)
{
	struct dma_map *map;
	int ret;

	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	map->buf = dma_buf_get(fd);
	if (IS_ERR(map->buf)) {
		ret = PTR_ERR(map->buf);
		goto get_err;
	}

	map->attach = dma_buf_attach(map->buf, dsp->miscdev.parent);
	if (IS_ERR(map->attach)) {
		ret = PTR_ERR(map->attach);
		goto attach_err;
	}

	map->table = dma_buf_map_attachment_unlocked(map->attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(map->table)) {
		ret = PTR_ERR(map->table);
		goto map_err;
	}

	// (CVP firmware needs IOMMU_PRIV, use 0 address to detect?)
	ret = iommu_map_sgtable(dsp->iommu, iova, map->table, IOMMU_READ | IOMMU_WRITE | (iova ? 0 : IOMMU_PRIV));
	WARN_ON(!ret);
	if (ret < 0)
		goto iommu_err;

	map->iova = iova;
	map->size = ret;

	spin_lock(&dsp->lock);
	list_add_tail(&map->node, &dsp->maps);
	spin_unlock(&dsp->lock);

	return ret;

iommu_err:
	dma_buf_unmap_attachment_unlocked(map->attach, map->table, DMA_BIDIRECTIONAL);
map_err:
	dma_buf_detach(map->buf, map->attach);
attach_err:
	dma_buf_put(map->buf);
get_err:
	kfree(map);

	return ret;
}

static int dsp_map_direct(struct dsp *dsp, u32 iova, u32 phys, u32 size)
{
	struct dma_map *map;
	int ret;

	map = kzalloc(sizeof(*map), GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	ret = iommu_map(dsp->iommu, iova, phys, size, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	if (ret) {
		kfree(map);
		return ret;
	}

	map->buf = NULL;
	map->attach = NULL;
	map->table = NULL;
	map->iova = iova;
	map->size = size;

	spin_lock(&dsp->lock);
	list_add_tail(&map->node, &dsp->maps);
	spin_unlock(&dsp->lock);
	return 0;
}

static void dsp_unmap(struct dsp *dsp, struct dma_map *map)
{
	iommu_unmap(dsp->iommu, map->iova, map->size);
	if (map->buf) {
		dma_buf_unmap_attachment_unlocked(map->attach, map->table, DMA_BIDIRECTIONAL);
		dma_buf_detach(map->buf, map->attach);
		dma_buf_put(map->buf);
	}
	kfree(map);
}

static long dsp_unmap_by_va(struct dsp *dsp, u32 va)
{
	struct dma_map *map, *n;

	spin_lock(&dsp->lock);
	list_for_each_entry_safe(map, n, &dsp->maps, node) {
		if (map->iova != va)
			continue;

		list_del(&map->node);
		spin_unlock(&dsp->lock);
		dsp_unmap(dsp, map);
		return 0;
	}
	spin_unlock(&dsp->lock);
	return -EINVAL;
}

#define CPU_CS_X2HSOFTINTCLR	0xa001c
#define CPU_CS_H2XSOFTINT	0xa0150
#define WRAPPER_INTR_STATUS	0xb000c

static long dsp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct dsp *dsp = file->private_data;
	char __user *argp = (char __user *)arg;

	switch (cmd) {
	case DSP_IOCTL_IRQ_STATUS:
		return atomic_xchg(&dsp->pending, 0);
	case DSP_IOCTL_IRQ_SEND:
		if (dsp->is_xtensa) {
			writel_relaxed(1, dsp->base + CPU_CS_H2XSOFTINT);
			return 0;
		}

		if (arg >= 4)
			return -EINVAL;
		mbox_send_message(dsp->mbox_chan[arg], NULL);
		mbox_client_txdone(dsp->mbox_chan[arg], 0);
		return 0;
	case DSP_IOCTL_BOOT:
		if (atomic_xchg(&dsp->booted, 1))
			return -EBUSY;

		if (dsp->is_xtensa) {
			/* Bring out of reset */
			writel(0, dsp->base + 0xc1000);
			return 0;
		}
		return dsp_boot(dsp, arg);
	case CVP_IOCTL_BOOT:
		if (atomic_xchg(&dsp->booted, 1))
			return -EBUSY;

		// Should only call this for CVP file
		if (!dsp->is_xtensa)
			return -EINVAL;

		writel(0, dsp->base + 0xc1000);
		return xtensa_boot(dsp);
	case DSP_IOCTL_SHUTDOWN:
		return dsp_shutdown(dsp);
	case DSP_IOCTL_MAP_DMABUF:
		return dsp_map_dmabuf(dsp, arg & 0xffffffffu, arg >> 32);
	case DSP_IOCTL_MAP_DIRECT: {
		struct map_direct map;
		if (copy_from_user(&map, argp, sizeof(map)))
			return -EFAULT;
		return dsp_map_direct(dsp, map.va, map.pa, map.size);
	}
	case DSP_IOCTL_UNMAP:
		return dsp_unmap_by_va(dsp, arg);
	case DSP_IOCTL_HANDOVER:
		return dsp_handover(dsp);
	default:
		return -ENOTTY;
	}
}

static int dsp_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct dsp *dsp = file->private_data;
	unsigned long user_count = vma_pages(vma);
	unsigned long count = dsp->mem_size >> PAGE_SHIFT;
	unsigned long pfn = PHYS_PFN(dsp->mem_phys);
	unsigned long off = vma->vm_pgoff;

	/* note: only used with CVP region which is a device region */
	vma->vm_page_prot = pgprot_device(vma->vm_page_prot);

	if (off >= count || user_count > count - off)
		return -ENXIO;

	return remap_pfn_range(vma, vma->vm_start, pfn + off,
			user_count << PAGE_SHIFT, vma->vm_page_prot);
}

static const struct file_operations dsp_fops = {
	.open = dsp_open,
	.release = dsp_release,
	.poll = dsp_poll,
	.unlocked_ioctl = dsp_ioctl,
	.compat_ioctl = dsp_ioctl,
	.mmap = dsp_mmap,
};

static irqreturn_t ipcc_irq(int irq, void *data)
{
	struct dsp *dsp = (void*) (((size_t) data) & ~3ul);
	atomic_or(BIT( (size_t) data & 3), &dsp->pending);
	wake_up_interruptible(&dsp->waitqueue);
	return IRQ_HANDLED;
}

static irqreturn_t xtensa_irq(int irq, void *data)
{
	struct dsp *dsp = data;
	u32 status;

	status = readl_relaxed(dsp->base + WRAPPER_INTR_STATUS);
	writel_relaxed(1, dsp->base + CPU_CS_X2HSOFTINTCLR);

	atomic_or(status, &dsp->pending);
	wake_up_interruptible(&dsp->waitqueue);
	return IRQ_HANDLED;
}

static int dsp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dsp *dsp;
	int i, irq, ret;
	//u32 max_rate;
	static int x;

	// missing: driving voltage rails. defer probe a bit for always-on to activate
	if (x++ < 3)
		return -EPROBE_DEFER;

	dsp = devm_kzalloc(dev, sizeof(*dsp), GFP_KERNEL);
	if (!dsp)
		return -ENOMEM;

	dsp->is_xtensa = of_device_is_compatible(dev->of_node, "qcom,sm8650-xtensa");
	dsp->is_adsp = of_device_is_compatible(dev->of_node, "qcom,sm8650-hexagon") &&
			!strcmp(dev->of_node->name, "adsp");
	dsp->is_cdsp = of_device_is_compatible(dev->of_node, "qcom,sm8650-hexagon") &&
			!strcmp(dev->of_node->name, "cdsp");

	/* 36-bit dma address mask */
	if (!dsp->is_xtensa) {
		ret = dma_set_mask_and_coherent(dev, 0xffffffffful);
		if (ret)
			return ret;
	}

	// XXX: get this from device tree instead
	if (!strcmp(dev->of_node->name, "cvp")) {
		dsp->mem_phys = 0x0ab00000;
		dsp->mem_size = 0xf0000;
	}

	dsp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dsp->base))
		return PTR_ERR(dsp->base);

	if (platform_get_resource_byname(pdev, IORESOURCE_MEM, "evb_sel")) {
		dsp->evb_sel = devm_platform_ioremap_resource_byname(pdev, "evb_sel");
		if (IS_ERR(dsp->evb_sel))
			return PTR_ERR(dsp->evb_sel);
	}

	INIT_LIST_HEAD(&dsp->maps);
	init_waitqueue_head(&dsp->waitqueue);

	spin_lock_init(&dsp->lock);

	ret = devm_clk_bulk_get_all(dev, &dsp->clks);
	if (ret < 0)
		return ret;
	dsp->num_clks = ret;

	if (dsp->is_xtensa) {
		irq = platform_get_irq(pdev, 0);
		if (irq < 0)
			return -ENODEV;

		ret = devm_request_irq(dev, irq, xtensa_irq, IRQF_TRIGGER_HIGH, "cvp", dsp);
		if (ret)
			return ret;
	} else {
		for (i = 0; i < 4; i++) {
			irq = platform_get_irq(pdev, i);
			if (irq < 0)
				return -ENODEV;

			ret = devm_request_irq(dev, irq, ipcc_irq, IRQF_TRIGGER_HIGH, "ipcc", (void*) ((size_t) dsp | i));
			if (ret)
				return ret;
		}

		dsp->mbox_client.dev = dev;
		dsp->mbox_client.knows_txdone = true;
		for (i = 0; i < 4; i++) {
			dsp->mbox_chan[i] = mbox_request_channel(&dsp->mbox_client, i);
			WARN_ON(IS_ERR(dsp->mbox_chan[i]));
		}
	}

	platform_set_drvdata(pdev, dsp);
	dev_set_drvdata(dev, dsp);

	dsp->iommu = iommu_paging_domain_alloc(dev);
	ret = iommu_attach_device(dsp->iommu, dev);
	WARN_ON(ret);

	dsp->miscdev.minor = MISC_DYNAMIC_MINOR;
	dsp->miscdev.name = dev->of_node->name;
	dsp->miscdev.fops = &dsp_fops;
	dsp->miscdev.parent = dev;
	ret = misc_register(&dsp->miscdev);
	if (ret)
		return ret;

	/* Get power domains and corresponding power levels */
	if (!strcmp(dev->of_node->name, "cvp")) {
		dsp->pds[0] = dev_pm_domain_attach_by_id(dev, 0);
		if (IS_ERR_OR_NULL(dsp->pds[0]))
			return PTR_ERR(dsp->pds[0]) ? : -ENODATA;
		dsp->pds[1] = dev_pm_domain_attach_by_id(dev, 1);
		if (IS_ERR_OR_NULL(dsp->pds[1]))
			return PTR_ERR(dsp->pds[1]) ? : -ENODATA;

		dsp->perf[0] = of_get_required_opp_performance_state(dev->of_node, 0);
		if (dsp->perf[0] < 0)
			return dsp->perf[0];
		dsp->perf[1] = of_get_required_opp_performance_state(dev->of_node, 1);
		if (dsp->perf[1] < 0)
			return dsp->perf[1];
	}
	if (dsp->is_cdsp) {
		dsp->pds[0] = dev_pm_domain_attach_by_name(dev, "cx");
		if (IS_ERR_OR_NULL(dsp->pds[0]))
			return PTR_ERR(dsp->pds[0]) ? : -ENODATA;
		dsp->pds[1] = dev_pm_domain_attach_by_name(dev, "mxc");
		if (IS_ERR_OR_NULL(dsp->pds[1]))
			return PTR_ERR(dsp->pds[1]) ? : -ENODATA;
		dsp->pds[2] = dev_pm_domain_attach_by_name(dev, "nsp");
		if (IS_ERR_OR_NULL(dsp->pds[2]))
			return PTR_ERR(dsp->pds[2]) ? : -ENODATA;
		dsp->pds[3] = dev_pm_domain_attach_by_name(dev, "nsp2");
		if (IS_ERR_OR_NULL(dsp->pds[3]))
			return PTR_ERR(dsp->pds[3]) ? : -ENODATA;

		dsp->perf[0] = of_get_required_opp_performance_state(dev->of_node, 0);
		if (dsp->perf[0] < 0)
			return dsp->perf[0];
		dsp->perf[1] = of_get_required_opp_performance_state(dev->of_node, 1);
		if (dsp->perf[1] < 0)
			return dsp->perf[1];
		dsp->perf[2] = of_get_required_opp_performance_state(dev->of_node, 2);
		if (dsp->perf[2] < 0)
			return dsp->perf[2];
		dsp->perf[3] = of_get_required_opp_performance_state(dev->of_node, 3);
		if (dsp->perf[3] < 0)
			return dsp->perf[3];
	}
	if (dsp->is_adsp) {
		dsp->pds[0] = dev_pm_domain_attach_by_name(dev, "lcx");
		if (IS_ERR_OR_NULL(dsp->pds[0]))
			return PTR_ERR(dsp->pds[0]) ? : -ENODATA;
		dsp->pds[1] = dev_pm_domain_attach_by_name(dev, "lmx");
		if (IS_ERR_OR_NULL(dsp->pds[1]))
			return PTR_ERR(dsp->pds[1]) ? : -ENODATA;

		dsp->perf[0] = of_get_required_opp_performance_state(dev->of_node, 0);
		if (dsp->perf[0] < 0)
			return dsp->perf[0];
		dsp->perf[1] = of_get_required_opp_performance_state(dev->of_node, 1);
		if (dsp->perf[1] < 0)
			return dsp->perf[1];
	}

	if (dsp->is_xtensa)
    {
        // Uses VIDEO_CC_MVS1C_CLK which looks like it's video_cc_mvs1c_div2_div_clk_src
        // which scales the input rate by 2 to match what is in the freq_tbl??
        // So try to map those allowed rates from the ftbl_video_cc_mvs1_clk_src_sm8650 ->
        // 420000000 -> 840000000
        // 555000000 -> 1110000000
        // 675000000 -> 1350000000
        // 750000000 -> 1500000000
        // 825000000 -> 1650000000
        u32 rate_to_set = 825000000;
        clk_set_rate(dsp->clks[1].clk, rate_to_set);
        u32 applied_rate = clk_get_rate(dsp->clks[1].clk);
        printk("cvp clock rate: %d (called with set_rate: %d)", applied_rate, rate_to_set);
    }

	// XXX
	ret = clk_bulk_prepare_enable(dsp->num_clks, dsp->clks);
	WARN_ON(ret);

	// XXX
	if (dsp->is_cdsp)
	{
		void __iomem *turingcc = devm_ioremap(dev, 0x32008000, 0x20000);
		WARN_ON(IS_ERR_OR_NULL(turingcc));

		// 0x32008404 ?
		writel_relaxed(1, turingcc + 0x404); // TURING_Q6SS_Q6_AXIM_CBCR
		//writel_relaxed(1, turingcc + 0x220); // TURING_CENG_CBCR
		writel_relaxed(1, turingcc + 0x22c); // TURING_NSPNOC_CBCR
		writel_relaxed(1, turingcc + 0x414); // TURING_Q6SS_AHBS_AON_CBCR


		dsp->dma_path = devm_of_icc_get(dev, "mem");
		WARN_ON(IS_ERR(dsp->dma_path));
		icc_set_bw(dsp->dma_path, 0, kBps_to_icc(1));

		// TURING_QDSP6SS_PLL_PLL_CM_PLL_LUCID_OLE
		// Set it to use TURBO_L3 frequency - 1843.2 MHz
		clk_pll_configure_lucid_ole(dsp->base + 0x40000, 1843200000);
		dsp->dsp_notifier.notifier_call = cdsp_panic_nb;
		dsp->dsp_notifier.priority = INT_MAX / 2;
	}

	if (dsp->is_adsp)
	{
		writel(0, dsp->base + DSP_APP_DEV_STATE);
		dsp->dma_path = devm_of_icc_get(dev, "mem");
		WARN_ON(IS_ERR(dsp->dma_path));
		icc_set_bw(dsp->dma_path, 0, UINT_MAX);
		dsp->dsp_notifier.notifier_call = adsp_panic_nb;
		dsp->dsp_notifier.priority = INT_MAX / 2;
	}

	if (dsp->is_adsp || dsp->is_cdsp)
		atomic_notifier_chain_register(&panic_notifier_list, &dsp->dsp_notifier);

	return 0;
}

static void dsp_remove(struct platform_device *pdev)
{
	struct dsp *dsp = platform_get_drvdata(pdev);

	if (dsp->is_adsp || dsp->is_cdsp)
		atomic_notifier_chain_unregister(&panic_notifier_list,
						 &dsp->dsp_notifier);
	misc_deregister(&dsp->miscdev);
	return;
}

static int dsp_noirq_suspend(struct device *dev)
{
	struct dsp *dsp = dev_get_drvdata(dev);

	dev_info(dev, "%s: going to suspend\n", __func__);
	if (dsp->is_adsp)
		writel(DSP_APP_DEV_SUSPENDED,
			dsp->base + DSP_APP_DEV_STATE);

	return 0;
}

static int dsp_noirq_resume(struct device *dev)
{
	struct dsp *dsp = dev_get_drvdata(dev);
	uint32_t adsp_dev_state;
	int retry = DSP_ADSP_DEV_SUSPENDED_RETRY;

	dev_info(dev, "%s: going to resume\n", __func__);
	if (dsp->is_adsp) {
		writel(DSP_APP_DEV_RESUMING,
		 	dsp->base + DSP_APP_DEV_STATE);
		/*
		 * now wait for the adsp to notify us that it had teardown all
		 * on going operations, we still get out if we wait too long.
		 */
		while(retry) {
			adsp_dev_state = readl(dsp->base + DSP_APP_DEV_STATE);
			if (adsp_dev_state & DSP_ADSP_DEV_SUSPENDED) {
				dev_info(dev, "%s: adsp suspended retry: %d\n",
					 __func__,
					 DSP_ADSP_DEV_SUSPENDED_RETRY - retry);
				adsp_dev_state &= ~DSP_ADSP_DEV_SUSPENDED;
				writel(adsp_dev_state,
					dsp->base + DSP_APP_DEV_STATE);
				break;
			}
			mdelay(DSP_ADSP_DEV_SUSPENDED_DELAY_MS);
			retry--;
		}
	}

	return 0;
}

static const struct dev_pm_ops dsp_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(dsp_noirq_suspend, dsp_noirq_resume)
};

static const struct of_device_id dsp_match_table[] = {
	{ .compatible = "qcom,sm8650-hexagon" },
	{ }
};
MODULE_DEVICE_TABLE(of, dsp_match_table);

static struct platform_driver dsp_driver = {
	.probe = dsp_probe,
	.remove = dsp_remove,
	.driver = {
		.name = "dsp-sm8650",
		.pm = &dsp_pm_ops,
		.of_match_table = dsp_match_table,
	},
};
module_platform_driver(dsp_driver);
