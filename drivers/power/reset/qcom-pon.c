// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2017-18 Linaro Limited

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kmsg_dump.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reboot-mode.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>

#define PON_SOFT_RB_SPARE		0x8f

#define GEN1_REASON_SHIFT		2
#define GEN2_REASON_SHIFT		1

#define NO_REASON_SHIFT			0

#define PON_PBS_PS_HOLD_SW_CTL		0x52
#define  PON_PS_HOLD_ENABLE		BIT(7)
#define PON_PBS_PS_HOLD_RESET_CTL2	0x53
#define  PON_PS_HOLD_TYPE_MASK		0x0f
#define  PON_PS_HOLD_TYPE_WARM_RESET	1
#define  PON_PS_HOLD_TYPE_SHUTDOWN	4
#define  PON_PS_HOLD_TYPE_HARD_RESET	7

#define PON_PBS_FAULT_REASON1		0xc7

static void qcom_pon_panic_force(struct work_struct *work);

#define QCOM_PON_COMBO_TIMEOUT_MS 3500
static DEFINE_SPINLOCK(qcom_pon_combo_lock);
static DECLARE_DELAYED_WORK(qcom_pon_panic_force_work, qcom_pon_panic_force);

#define PON_AOP_PON_REASON1		0x7269
#define PON_AOP_WARM_RESET_REASON1	0x726a
#define PON_AOP_ON_REASON		0x726b
#define PON_AOP_POFF_REASON1		0x726c
#define PON_AOP_OFF_REASON1		0x726d
#define PON_AOP_OFF_FAULT_REASON1	0x726e
#define PON_AOP_OFF_FAULT_REASON2	0x726f

struct qcom_pon_data {
	long reason_shift;
	bool needs_pbs_ps_hold_sw_ctl;
	bool pmk8550_pon_reason;
};

struct pm_pon_reason_type
{
  u32 hard_reset : 1;     /* Hard reset event trigger */
  u32 smpl : 1;           /* SMPL trigger */
  u32 rtc : 1;            /* RTC trigger */
  u32 dc_chg : 1;         /* DC Charger trigger */
  u32 usb_chg : 1;        /* USB Charger trigger */
  u32 pon1 : 1;           /* PON1 trigger */
  u32 cblpwr : 1;         /* CBL_PWR1_N trigger */
  u32 kpdpwr : 1;         /* KPDPWR_N trigger */
  u32 known_rsn : 1;      /* known reason. If unknown read sid, pid and irq field */
  u32 sid : 4;            /* slave id */
  u32 pid: 8;             /* peripheral id */
  u32 irq: 3;             /* interrupt */
  u32 reserved:8;         /* reserved  */
};

const char *pm_pon_reason_type_str[] = {
	"HARD_RESET",
	"SMPL",
	"RTC",
	"DC_CHG",
	"USB_CHG",
	"PON1",
	"CBL_PWR1_N",
	"KPDPWR_N",
};

/*! \struct pm_pon_warm_reset_reason_type
   \brief Warm Reset reason type.
 */
struct pm_pon_warm_reset_reason_type
{
  u32 soft : 1;               /* Software trigger */
  u32 ps_hold : 1;            /* PS_HOLD trigger */
  u32 pmic_wd : 1;            /* PMIC Watchdog trigger */
  u32 gp1 : 1;                /* Keypad_Reset1 trigger */
  u32 gp2 : 1;                /* Keypad_Reset2 trigger */
  u32 kpdpwr_and_resin : 1;   /* KPDPWR_N and RESIN_N trigger */
  u32 resin : 1;              /* RESIN_N trigger */
  u32 kpdpwr : 1;             /* KPDPWR_N trigger */
  u32 known_rsn : 1;          /* known reason. If false then read sid, pid and irq field */
  u32 sid : 4;                /* slave id */
  u32 pid: 8;                 /* peripheral id */
  u32 irq: 3;                 /* interrupt */
  u32 reserved:8;             /* reserved  */
};

const char *pm_pon_warm_reset_reason_type_str[] = {
	"KPDPWR_N",
	"RESIN_N",
	"KPDPWR_AND_RESIN",
	"GP2",
	"GP1",
	"PMIC_WD",
	"PS_HOLD",
	"SOFT",
};

/*! \struct pm_pon_on_state_reason_type
   \brief Power on sequences type.
 */
struct pm_pon_on_state_reason_type
{
  unsigned reserved1 : 6;     /* Reserved bits */
  unsigned warm_seq  : 1;     /* Warm sequence */
  unsigned pon_seq   : 1;     /* PON sequence  */
};


/*! \struct pm_pon_poff_reason_type
   \brief Power Off reason type.
 */
struct pm_pon_poff_reason_type
{
  u32 soft : 1;               /* Software trigger */
  u32 ps_hold : 1;            /* PS_HOLD trigger */
  u32 pmic_wd : 1;            /* PMIC Watchdog trigger */
  u32 gp1 : 1;                /* Keypad_Reset1 trigger */
  u32 gp2 : 1;                /* Keypad_Reset2 trigger */
  u32 kpdpwr_and_resin : 1;   /* KPDPWR_N and RESIN_N trigger */
  u32 resin : 1;              /* RESIN_N trigger */
  u32 kpdpwr : 1;             /* KPDPWR_N trigger */
  u32 known_rsn : 1;          /* known reason. If false then read sid, pid and irq field */
  u32 sid : 4;                /* slave id */
  u32 pid: 8;                 /* peripheral id */
  u32 irq: 3;                  /* interrupt */
  u32 reserved:8;              /* reserved  */
};

const char *pm_pon_poff_reason_type_str[] = {
	"SOFT",
	"PS_HOLD",
	"PMIC_WD",
	"GP1",
	"GP2",
	"KPDPWR_AND_RESIN",
	"RESIN",
	"KPDPWR",
};

/*! \struct pm_pon_off_state_reason_type
   \brief Power Off reason type.
 */
struct pm_pon_off_state_reason_type
{
  unsigned reserved1 : 2;               /* Reserved bits */
  unsigned raw_xvdd_rb_occured : 1;     /* raw_xvdd_rb event occured */
  unsigned raw_dvdd_rb_occured : 1;     /* raw_dvdd_rb event occured  */
  unsigned immediate_xvdd_shutdown : 1; /* immediate_xvdd_shutdown s2 Reset */
  unsigned s3_reset : 1;                /* Stage 3 Reset */
  unsigned fault_seq : 1;               /* fault-handling sequence */
  unsigned poff_seq : 1;                /* normal powering-off sequence */
};

const char *pm_pon_off_reason_type_str[] = {
	"RESERVED",
	"RESERVED",
	"RAW_XVDD_RB_OCCURED",
	"RAW_DVDD_RB_OCCURED",
	"IMMEDIATE_XVDD_SHUTDOWN",
	"S3_RESET",
	"FAULT_SEQ",
	"POFF_SEQ",
};

/*! \struct pm_pon_fault_state_reason_type
   \brief Pon fault reason type.
 */
struct pm_pon_fault1_state_reason_type
{
  unsigned gp_fault0 : 1;         /* Triggered by general-purpose fault #0 */
  unsigned gp_fault1 : 1;         /* Triggered by general-purpose fault #1 */
  unsigned gp_fault2 : 1;         /* general-purpose fault #2 */
  unsigned gp_fault3 : 1;         /* general-purpose fault #3 */
  unsigned mbg_fault : 1;         /* MBG fault event */
  unsigned ovlo : 1;              /* OVLO event    */
  unsigned uvlo : 1;              /* UVLO event    */
  unsigned avdd_rb : 1;           /* AVDD_RB event */
};

const char *pm_pon_fault1_reason_type_str[] = {
	"GP_FAULT0",
	"GP_FAULT1",
	"GP_FAULT2",
	"GP_FAULT3",
	"MBG_FAULT",
	"OVLO",
	"UVLO",
	"AVDD_RB",
};

struct pm_pon_fault2_state_reason_type {
  unsigned reserved1 : 3;         /* Reserved bits */
  unsigned fault_n : 1;           /* FAULT_N bus */
  unsigned pbs_watchdog_to : 1;   /* PBS WATCHDOG */
  unsigned pbs_nack : 1;          /* PBS NACK event */
  unsigned restart_pon : 1;       /* RESTART PON */
  unsigned otst3 : 1;             /* OTST3 */
};

const char *pm_pon_fault2_reason_type_str[] = {
	"RESERVED",
	"RESERVED",
	"RESERVED",
	"FAULT_N",
	"PBS_WATCHDOG_TO",
	"PBS_NACK",
	"RESTART_PON",
	"OTST3",
};

struct qcom_pon {
	struct device *dev;
	struct regmap *regmap;
	u32 baseaddr;
	u32 pbs_baseaddr;
	struct reboot_mode_driver reboot_mode;
	long reason_shift;
	bool key_power_pressed;
	bool key_voldown_pressed;
	bool combo_timer_started;
	struct input_handler input_handler;
	bool needs_pbs_ps_hold_sw_ctl;
	union {
		u32 regval;
		struct pm_pon_reason_type fields;
	} pmic_pon_trigger;
	union {
		u32 regval;
		struct pm_pon_on_state_reason_type fields;
	} pmic_pon_type;
	union {
		u32 regval;
		struct pm_pon_warm_reset_reason_type fields;
	} pmic_pon_warm;
	union {
		u32 regval;
		struct pm_pon_poff_reason_type fields;
	} pmic_pon_poff;
	union {
		u32 regval;
		struct pm_pon_off_state_reason_type fields;
	} pmic_off;
	union {
		u32 regval;
		struct pm_pon_fault1_state_reason_type fields;
	} pmic_fault1;
	union {
		u32 regval;
		struct pm_pon_fault2_state_reason_type fields;
	} pmic_fault2;
};

static int qcom_pon_reboot(struct notifier_block *nb,
			   unsigned long action, void *data)
{
	char *cmd = data;

	if (cmd && !strncmp(cmd, "warm", sizeof("warm"))) {
		printk("%s: action: %lu cmd: %s reboot_mode: %d\n",
			__func__, action, cmd, reboot_mode);
		reboot_mode = REBOOT_WARM;
	}

	return NOTIFY_DONE;
}

static struct notifier_block qcom_pon_reboot_notifier = {
	.notifier_call = qcom_pon_reboot,
};

static void qcom_reboot_ps_hold_type_setup(struct qcom_pon *pon, unsigned int type)
{
	regmap_write(pon->regmap, pon->pbs_baseaddr + PON_PBS_PS_HOLD_RESET_CTL2, 0);
	usleep_range(100, 1000);
	regmap_write(pon->regmap, pon->pbs_baseaddr + PON_PBS_PS_HOLD_SW_CTL,
		     type);
	regmap_write(pon->regmap, pon->pbs_baseaddr + PON_PBS_PS_HOLD_RESET_CTL2,
		     PON_PS_HOLD_ENABLE);
}

static int qcom_pon_restart_prepare(struct sys_off_data *data)
{
	struct qcom_pon *pon = data->cb_data;
	unsigned int reset_type;

	dev_err(pon->dev, "Setting PON-PBS for Reset: %s reboot_mode: %d\n",
		data->cmd, reboot_mode);

	if (reboot_mode == REBOOT_WARM) {
		reset_type = PON_PS_HOLD_TYPE_WARM_RESET;
		kmsg_dump(KMSG_DUMP_PANIC);
	} else {
		reset_type = PON_PS_HOLD_TYPE_HARD_RESET;
	}

	qcom_reboot_ps_hold_type_setup(pon, reset_type);

	return NOTIFY_OK;
}

static int qcom_pon_poweroff_prepare(struct sys_off_data *data)
{
	struct qcom_pon *pon = data->cb_data;

	dev_info(pon->dev, "Setting PON-PBS for Shutdown\n");

	qcom_reboot_ps_hold_type_setup(pon, PON_PS_HOLD_TYPE_SHUTDOWN);

	return NOTIFY_OK;
}

static int qcom_pon_reboot_mode_write(struct reboot_mode_driver *reboot,
				    unsigned int magic)
{
	struct qcom_pon *pon = container_of
			(reboot, struct qcom_pon, reboot_mode);
	int ret;

	ret = regmap_update_bits(pon->regmap,
				 pon->baseaddr + PON_SOFT_RB_SPARE,
				 GENMASK(7, pon->reason_shift),
				 magic << pon->reason_shift);
	if (ret < 0)
		dev_err(pon->dev, "update reboot mode bits failed\n");

	return ret;
}

static ssize_t pon_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct qcom_pon *pon = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "0x%02X\n", pon->pmic_pon_type.regval);
}
static DEVICE_ATTR_RO(pon_type);

#define PON_TYPE_FIELD(field) \
static ssize_t pon_type_##field##_show(struct device *dev, struct device_attribute *attr, \
			        char *buf)					\
{										\
	struct qcom_pon *pon = dev_get_drvdata(dev);				\
										\
	return sysfs_emit(buf, "%d\n", pon->pmic_pon_type.fields.field);	\
}										\
static DEVICE_ATTR_RO(pon_type_##field);

PON_TYPE_FIELD(warm_seq);
PON_TYPE_FIELD(pon_seq);

static ssize_t pon_reason_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct qcom_pon *pon = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "0x%02X\n", pon->pmic_pon_trigger.regval);
}
static DEVICE_ATTR_RO(pon_reason);

#define PON_REASON_FIELD(field) \
static ssize_t pon_reason_##field##_show(struct device *dev, struct device_attribute *attr, \
			        char *buf)					\
{										\
	struct qcom_pon *pon = dev_get_drvdata(dev);				\
										\
	return sysfs_emit(buf, "%d\n", pon->pmic_pon_trigger.fields.field);	\
}										\
static DEVICE_ATTR_RO(pon_reason_##field);

PON_REASON_FIELD(smpl);
PON_REASON_FIELD(hard_reset);
PON_REASON_FIELD(usb_chg);

static struct attribute *attrs[] = {
	&dev_attr_pon_reason.attr,
	&dev_attr_pon_reason_smpl.attr,
	&dev_attr_pon_reason_hard_reset.attr,
	&dev_attr_pon_reason_usb_chg.attr,
	&dev_attr_pon_type.attr,
	&dev_attr_pon_type_warm_seq.attr,
	&dev_attr_pon_type_pon_seq.attr,
	NULL,
};

static struct attribute_group sysfs_group = {
	.attrs = attrs,
};

static void qcom_pon_panic_force(struct work_struct *work)
{
	panic("%s: calling panic dumping dmesg to pstore\n", __func__);
}

static struct qcom_pon *qcom_pon_input_handler_to_qcom_pon(struct input_handler *handler)
{
	return container_of(handler, struct qcom_pon, input_handler);
}

static void qcom_pon_combo_event(struct input_handle *handle, unsigned int type,
				 unsigned int code, int value)
{
	struct qcom_pon *pon = qcom_pon_input_handler_to_qcom_pon(handle->handler);
	unsigned long flags;
	bool combo_pressed;

	if (type != EV_KEY)
		return;

	if (code != KEY_POWER && code != KEY_VOLUMEDOWN)
		return;

	spin_lock_irqsave(&qcom_pon_combo_lock, flags);

	if (code == KEY_POWER)
		pon->key_power_pressed = !!value;
	else if (code == KEY_VOLUMEDOWN)
		pon->key_voldown_pressed = !!value;

	combo_pressed = (pon->key_voldown_pressed && pon->key_power_pressed);

	if (combo_pressed && !pon->combo_timer_started) {
		schedule_delayed_work(&qcom_pon_panic_force_work,
					msecs_to_jiffies(QCOM_PON_COMBO_TIMEOUT_MS));
		pon->combo_timer_started = true;
		dev_info(pon->dev, "timer started for key combo\n");
	} else if (!combo_pressed && pon->combo_timer_started) {
		cancel_delayed_work(&qcom_pon_panic_force_work);
		pon->combo_timer_started = false;
		dev_info(pon->dev, "timer stopped for key combo\n");
	}

	spin_unlock_irqrestore(&qcom_pon_combo_lock, flags);
}

static int qcom_pon_combo_connect(struct input_handler *handler,
				  struct input_dev *dev,
				  const struct input_device_id *id)
{
	struct input_handle *handle;
	int ret;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "qcom_pon_combo_timer_listener";

	ret = input_register_handle(handle);
	if (ret)
		goto err_free;

	ret = input_open_device(handle);
	if (ret)
		goto err_unregister;

	return 0;

err_unregister:
	input_unregister_handle(handle);

err_free:
	kfree(handle);

	return ret;
}

static void qcom_pon_combo_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}

static const struct input_device_id qcom_pon_combo_ids[] = {
	{
		.flags = INPUT_DEVICE_ID_MATCH_EVBIT,
		.evbit = { BIT_MASK(EV_KEY) },
	},
	{ },
};
MODULE_DEVICE_TABLE(input, qcom_pon_combo_ids);

static struct input_handler qcom_pon_combo_handler = {
	.event      = qcom_pon_combo_event,
	.connect    = qcom_pon_combo_connect,
	.disconnect = qcom_pon_combo_disconnect,
	.name       = "qcom_pon_combo_timer_handler",
	.id_table   = qcom_pon_combo_ids,
};

static void qcom_pon_register_str(u8 regval, const char *qcom_pon_reg_str[])
{
	int i;

	if (!regval)
		pr_cont(" none");
	else {
		for (i = 0; i < BITS_PER_BYTE; i++) {
			if (regval & BIT(i))
				pr_cont(" %s", qcom_pon_reg_str[i]);
		}
	}
	pr_cont("\n");
}

static void qcom_pon_reason_dump(struct qcom_pon *pon)
{
	u32 pmic_pon_regval = 0;
	int error;

	/*
	 * 0x09 -> `reboot` w/ USB connected
	 * 0x01 -> `reboot` w/o USB
	 * 0x81 -> power button for 16 seconds HARD_RESET
	 * 0x08 -> remove battery and power on via USB insertion
	 * 0x08 -> poweroff with USB removed then USB insertion
	 * 0x80 -> poweroff then power on button with no USB connected
	 *
	 * NOTE buttonledcontrol only looks at 0x80 to start steamvr
	 */

	/* Load the PON Reason from the Always On Processor in PMK8550 SDAM3 */
	/* NOTE The PMK8550 seems to detect USB Insertion as Wireless,
	 * but otherwise the codes are unchanged from XR2 */
	/* first dump the pon reason type */
	error = regmap_read(pon->regmap, PON_AOP_ON_REASON, &pmic_pon_regval);
	pr_info("PON: type [0x%02X]: ", pmic_pon_regval);
	pon->pmic_pon_type.regval = pmic_pon_regval;

	if (pon->pmic_pon_type.fields.warm_seq)
		pr_cont("Warm reset\n");
	else if (pon->pmic_pon_type.fields.pon_seq)
		pr_cont("PON reset seq\n");

	/* after dump the pon reason trigger */
	error = regmap_read(pon->regmap, PON_AOP_PON_REASON1, &pmic_pon_regval);
	pr_info("PON: trigger [0x%02X]:", pmic_pon_regval);
	pon->pmic_pon_trigger.regval = pmic_pon_regval;
	qcom_pon_register_str(pmic_pon_regval, pm_pon_reason_type_str);

	/* warm reset reason */
	error = regmap_read(pon->regmap, PON_AOP_WARM_RESET_REASON1,
			    &pmic_pon_regval);
	pr_info("PON: warm_reset_reason [0x%02X]:", pmic_pon_regval);
	pon->pmic_pon_warm.regval = pmic_pon_regval;
	qcom_pon_register_str(pmic_pon_regval, pm_pon_warm_reset_reason_type_str);
}

static void qcom_poff_reason_dump(struct qcom_pon *pon)
{
	u32 pmic_pon_regval = 0;
	int error;

	error = regmap_read(pon->regmap, PON_AOP_POFF_REASON1, &pmic_pon_regval);
	pr_info("PON: poff_reason [0x%02X]:", pmic_pon_regval);
	qcom_pon_register_str(pmic_pon_regval, pm_pon_poff_reason_type_str);
	pon->pmic_pon_poff.regval = pmic_pon_regval;

	error = regmap_read(pon->regmap, PON_AOP_OFF_REASON1, &pmic_pon_regval);
	pr_info("PON: off_reason [0x%02X]:", pmic_pon_regval);
	qcom_pon_register_str(pmic_pon_regval, pm_pon_off_reason_type_str);
	pon->pmic_off.regval = pmic_pon_regval;

	error = regmap_read(pon->regmap, pon->pbs_baseaddr + PON_PBS_FAULT_REASON1,
			    &pmic_pon_regval);
	pon->pmic_fault1.regval = pmic_pon_regval & 0xff;
	pon->pmic_fault2.regval = (pmic_pon_regval >> 8) & 0xff;
	pr_info("PON: fault_reason1 [0x%02X]:", pon->pmic_fault1.regval);
	qcom_pon_register_str(pon->pmic_fault1.regval, pm_pon_fault1_reason_type_str);
	pr_info("PON: fault_reason2 [0x%02X]:", pon->pmic_fault2.regval);
	qcom_pon_register_str(pon->pmic_fault2.regval, pm_pon_fault2_reason_type_str);
}

static int qcom_pon_probe(struct platform_device *pdev)
{
	const struct qcom_pon_data *data;
	struct qcom_pon *pon;
	int error;

	data = device_get_match_data(&pdev->dev);
	if (!data)
		return -EINVAL;

	pon = devm_kzalloc(&pdev->dev, sizeof(*pon), GFP_KERNEL);
	if (!pon)
		return -ENOMEM;

	pon->dev = &pdev->dev;

	pon->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!pon->regmap) {
		dev_err(&pdev->dev, "failed to locate regmap\n");
		return -ENODEV;
	}

	error = of_property_read_u32(pdev->dev.of_node, "reg", &pon->baseaddr);
	if (error)
		return error;

	if (data->needs_pbs_ps_hold_sw_ctl) {
		error = of_property_read_u32_index(pdev->dev.of_node, "reg", 1,
						   &pon->pbs_baseaddr);
		if (error)
			return error;

		devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_RESTART_PREPARE,
				      SYS_OFF_PRIO_FIRMWARE, qcom_pon_restart_prepare,
				      pon);

		devm_register_sys_off_handler(&pdev->dev, SYS_OFF_MODE_POWER_OFF_PREPARE,
				      SYS_OFF_PRIO_FIRMWARE, qcom_pon_poweroff_prepare,
				      pon);
	}


	error = devm_register_reboot_notifier(&pdev->dev, &qcom_pon_reboot_notifier);
	if (error)
		dev_err(&pdev->dev, "failed to regitster restart handler\n");

	if (data->reason_shift != NO_REASON_SHIFT) {
		pon->reboot_mode.dev = &pdev->dev;
		pon->reason_shift = data->reason_shift;
		pon->reboot_mode.write = qcom_pon_reboot_mode_write;
		error = devm_reboot_mode_register(&pdev->dev, &pon->reboot_mode);
		if (error) {
			dev_err(&pdev->dev, "can't register reboot mode\n");
			return error;
		}
	}

	if (data->pmk8550_pon_reason) {
		error = sysfs_create_group(&pdev->dev.kobj, &sysfs_group);
		if (error) {
			dev_err(&pdev->dev, "can't create sysfs group\n");
			return error;
		}

		/* now dump a more human information */
		qcom_pon_reason_dump(pon);

		qcom_poff_reason_dump(pon);
	}

	/* register input handler to get combo keys pressed to trigger warm reset */
	memcpy(&pon->input_handler, &qcom_pon_combo_handler,
		sizeof(qcom_pon_combo_handler));
	error = input_register_handler(&pon->input_handler);
	if (error)
		dev_err(&pdev->dev, "could not registered input combo handler\n");

	platform_set_drvdata(pdev, pon);

	return devm_of_platform_populate(&pdev->dev);
}

static const struct qcom_pon_data pm8916_pon_data = {
	.reason_shift = GEN1_REASON_SHIFT,
};

static const struct qcom_pon_data pm8941_pon_data = { };

static const struct qcom_pon_data pm8998_pon_data = {
	.reason_shift = GEN2_REASON_SHIFT,
};

static const struct qcom_pon_data pmk8350_el2_pon_data = {
	.reason_shift = GEN2_REASON_SHIFT,
	.needs_pbs_ps_hold_sw_ctl = true,
	.pmk8550_pon_reason = true,
};

static const struct of_device_id qcom_pon_id_table[] = {
	{ .compatible = "qcom,pm8916-pon", .data = &pm8916_pon_data },
	{ .compatible = "qcom,pm8941-pon", .data = &pm8941_pon_data },
	{ .compatible = "qcom,pms405-pon", .data = &pm8916_pon_data },
	{ .compatible = "qcom,pm8998-pon", .data = &pm8998_pon_data },
	{ .compatible = "qcom,pmk8350-pon", .data = &pm8998_pon_data },
	{ .compatible = "qcom,pmk8350-el2-pon", .data = &pmk8350_el2_pon_data },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_pon_id_table);

static struct platform_driver qcom_pon_driver = {
	.probe = qcom_pon_probe,
	.driver = {
		.name = "qcom-pon",
		.of_match_table = qcom_pon_id_table,
	},
};
module_platform_driver(qcom_pon_driver);

MODULE_DESCRIPTION("Qualcomm Power On driver");
MODULE_LICENSE("GPL v2");
