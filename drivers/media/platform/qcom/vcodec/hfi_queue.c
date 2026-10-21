// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022-2023 Qualcomm Innovation Center, Inc. All rights reserved.
 */

#include "hfi_queue.h"

static void venus_dump_packet(const void *packet)
{
	size_t pkt_size = *(u32 *)packet;

	print_hex_dump(KERN_ERR, "", DUMP_PREFIX_OFFSET, 16, 1, packet, pkt_size, true);
}

int write_queue(struct iface_q_info *qinfo, void *packet,
		u32 packet_size, u32 *rx_req)
{
	u32 empty_space, read_idx, write_idx, new_write_idx;
	struct hfi_queue_header *queue;
	u32 *write_ptr, residue;

	queue = qinfo->qhdr;
	if (!queue)
		return -EINVAL;

	//printk("write_queue:\n");
	//venus_dump_packet(packet);

	if (!packet_size || packet_size > qinfo->q_array.size)
		return -EINVAL;

	read_idx = queue->qhdr_read_idx * sizeof(u32);
	write_idx = queue->qhdr_write_idx * sizeof(u32);

	empty_space = (write_idx >=  read_idx) ?
		(qinfo->q_array.size - (write_idx -  read_idx)) :
		(read_idx - write_idx);
	if (empty_space <= packet_size) {
		queue->qhdr_tx_req = 1;
		return -ENOSPC;
	}

	queue->qhdr_tx_req =  0;

	new_write_idx = write_idx + packet_size;
	write_ptr = (u32 *)((u8 *)qinfo->q_array.kernel_vaddr + write_idx);

	if (write_ptr < (u32 *)qinfo->q_array.kernel_vaddr ||
	    write_ptr > (u32 *)(qinfo->q_array.kernel_vaddr +
	    qinfo->q_array.size))
		return -EINVAL;

	if (new_write_idx < qinfo->q_array.size) {
		memcpy(write_ptr, packet, packet_size);
	} else {
		residue = new_write_idx - qinfo->q_array.size;
		memcpy(write_ptr, packet, (packet_size - residue));
		memcpy(qinfo->q_array.kernel_vaddr,
		       packet + (packet_size - residue), residue);
		new_write_idx = residue;
	}

	/* Make sure packet is written before updating the write index */
	mb();
	queue->qhdr_write_idx = new_write_idx / sizeof(u32);
	*rx_req = queue->qhdr_rx_req ? 1 : 0;

	/* Make sure write index is updated before an interrupt is raised */
	mb();

	return 0;
}

int read_queue(struct iface_q_info *qinfo, void *packet, u32 *tx_req)
{
	u32 read_idx, write_idx, new_read_idx;
	struct hfi_queue_header *queue;
	u32 receive_request = 0;
	u32 packet_size, residue;
	u32 *read_ptr;
	int ret = 0;

	/* Make sure data is valid before reading it */
	mb();
	queue = qinfo->qhdr;
	if (!queue)
		return -EINVAL;

	if (queue->qhdr_type & IFACEQ_MSGQ_ID)
		receive_request = 1;

	read_idx = queue->qhdr_read_idx * sizeof(u32);
	write_idx = queue->qhdr_write_idx * sizeof(u32);

	if (read_idx == write_idx) {
		queue->qhdr_rx_req = receive_request;
		/* Ensure qhdr is updated in main memory */
		mb();
		return -ENODATA;
	}

	read_ptr = (u32 *)(qinfo->q_array.kernel_vaddr + read_idx);
	if (read_ptr < (u32 *)qinfo->q_array.kernel_vaddr ||
	    read_ptr > (u32 *)(qinfo->q_array.kernel_vaddr +
	    qinfo->q_array.size - sizeof(*read_ptr)))
		return -ENODATA;

	packet_size = *read_ptr;
	if (!packet_size)
		return -EINVAL;

	new_read_idx = read_idx + packet_size;
	if (packet_size <= IFACEQ_CORE_PKT_SIZE &&
	    read_idx <= qinfo->q_array.size) {
		if (new_read_idx < qinfo->q_array.size) {
			memcpy(packet, read_ptr, packet_size);
		} else {
			residue = new_read_idx - qinfo->q_array.size;
			memcpy(packet, read_ptr, (packet_size - residue));
			memcpy((packet + (packet_size - residue)),
			       qinfo->q_array.kernel_vaddr, residue);
			new_read_idx = residue;
		}
	} else {
		new_read_idx = write_idx;
		ret = -EBADMSG;
	}

	queue->qhdr_rx_req = receive_request;

	queue->qhdr_read_idx = new_read_idx / sizeof(u32);

	*tx_req = queue->qhdr_tx_req ? 1 : 0;
	/* Ensure qhdr is updated in main memory */
	mb();

	//printk("read_queue:\n");
	//venus_dump_packet(packet);

	return ret;
}

static void __set_queue_hdr_defaults(struct hfi_queue_header *q_hdr)
{
	q_hdr->qhdr_status = 0x1;
	q_hdr->qhdr_type = IFACEQ_DFLT_QHDR;
	q_hdr->qhdr_q_size = IFACEQ_QUEUE_SIZE / 4;
	q_hdr->qhdr_pkt_size = 0;
	q_hdr->qhdr_rx_wm = 0x1;
	q_hdr->qhdr_tx_wm = 0x1;
	q_hdr->qhdr_rx_req = 0x1;
	q_hdr->qhdr_tx_req = 0x0;
	q_hdr->qhdr_rx_irq_status = 0x0;
	q_hdr->qhdr_tx_irq_status = 0x0;
	q_hdr->qhdr_read_idx = 0x0;
	q_hdr->qhdr_write_idx = 0x0;
}

static void hfi_set_queue_header(u32 queue_id, struct iface_q_info *iface_q)
{
	__set_queue_hdr_defaults(iface_q->qhdr);
	iface_q->qhdr->qhdr_start_addr = iface_q->q_array.device_addr;
	iface_q->qhdr->qhdr_type |= queue_id;

	/*
	 * Set receive request to zero on debug queue as there is no
	 * need of interrupt from video hardware for debug messages
	 */
	if (queue_id == IFACEQ_DBGQ_ID)
		iface_q->qhdr->qhdr_rx_req = 0;
}

static void __queue_init(struct mem_desc *q_table, u32 queue_id, struct iface_q_info *iface_q)
{
	unsigned int offset = 0;

	offset = q_table->size + (queue_id * IFACEQ_QUEUE_SIZE);
	iface_q->q_array.device_addr = q_table->device_addr + offset;
	iface_q->q_array.kernel_vaddr =
			(void *)((char *)q_table->kernel_vaddr + offset);
	iface_q->q_array.size = IFACEQ_QUEUE_SIZE;
	iface_q->qhdr =
		IFACEQ_GET_QHDR_START_ADDR(q_table->kernel_vaddr, queue_id);

	memset(iface_q->qhdr, 0, sizeof(struct hfi_queue_header));

	hfi_set_queue_header(queue_id, iface_q);
}

int hfi_queue_init(struct device *dev, struct mem_desc *q_table, struct mem_desc *sfr,
		   struct iface_q_info *cmd_q, struct iface_q_info *msg_q,
		   struct iface_q_info *dbg_q, void *daddr)
{
	struct hfi_queue_table_header *q_tbl_hdr;

	if (q_table->kernel_vaddr) {
		hfi_set_queue_header(IFACEQ_CMDQ_ID, cmd_q);
		hfi_set_queue_header(IFACEQ_MSGQ_ID, msg_q);
		hfi_set_queue_header(IFACEQ_DBGQ_ID, dbg_q);
		return 0;
	}

	q_table->kernel_vaddr = dma_alloc_attrs(dev, ALIGNED_QUEUE_SIZE,
						&q_table->device_addr,
						GFP_KERNEL, DMA_ATTR_WRITE_COMBINE);
	if (!q_table->kernel_vaddr) {
		dev_err(dev, "%s: queues alloc and map failed\n", __func__);
		return -ENOMEM;
	}

	sfr->kernel_vaddr = dma_alloc_attrs(dev, ALIGNED_SFR_SIZE,
					    &sfr->device_addr,
					    GFP_KERNEL, DMA_ATTR_WRITE_COMBINE);
	if (!sfr->kernel_vaddr) {
		dev_err(dev, "%s: sfr alloc and map failed\n", __func__);
		return -ENOMEM;
	}

	q_table->size = IFACEQ_TABLE_SIZE;

	__queue_init(q_table, IFACEQ_CMDQ_ID, cmd_q);
	__queue_init(q_table, IFACEQ_MSGQ_ID, msg_q);
	__queue_init(q_table, IFACEQ_DBGQ_ID, dbg_q);

	q_tbl_hdr = (struct hfi_queue_table_header *)q_table->kernel_vaddr;
	q_tbl_hdr->qtbl_version = 0;
	q_tbl_hdr->device_addr = (void *)daddr;
	strscpy(q_tbl_hdr->name, "hfi-queues", sizeof(q_tbl_hdr->name));
	q_tbl_hdr->qtbl_size = IFACEQ_TABLE_SIZE;
	q_tbl_hdr->qtbl_qhdr0_offset = sizeof(*q_tbl_hdr);
	q_tbl_hdr->qtbl_qhdr_size = sizeof(struct hfi_queue_header);
	q_tbl_hdr->qtbl_num_q = IFACEQ_NUMQ;
	q_tbl_hdr->qtbl_num_active_q = IFACEQ_NUMQ;

	sfr->size = ALIGNED_SFR_SIZE;
	/* Write sfr size in first word to be used by firmware */
	*((u32 *)sfr->kernel_vaddr) = sfr->size;

	return 0;
}

static void __queue_deinit(struct iface_q_info *iface_q)
{
	iface_q->qhdr = NULL;
	iface_q->q_array.kernel_vaddr = NULL;
	iface_q->q_array.device_addr = 0;
}

void hfi_queue_deinit(struct device *dev, struct mem_desc *q_table, struct mem_desc *sfr,
		      struct iface_q_info *cmd_q, struct iface_q_info *msg_q,
		      struct iface_q_info *dbg_q)
{
	if (!q_table->kernel_vaddr)
		return;

	dma_free_attrs(dev, q_table->size, q_table->kernel_vaddr,
		       q_table->device_addr, q_table->attrs);

	dma_free_attrs(dev, sfr->size, sfr->kernel_vaddr,
		       sfr->device_addr, sfr->attrs);

	__queue_deinit(cmd_q);
	__queue_deinit(msg_q);
	__queue_deinit(dbg_q);

	q_table->kernel_vaddr = NULL;
	q_table->device_addr = 0;

	sfr->kernel_vaddr = NULL;
	sfr->device_addr = 0;
}
