// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2013 Red Hat
 * Author: Rob Clark <robdclark@gmail.com>
 */

#include "drm/drm_drv.h"

#include "msm_gpu.h"
#include "msm_gem.h"
#include "msm_mmu.h"
#include "msm_fence.h"
#include "msm_gpu_trace.h"
//#include "adreno/adreno_gpu.h"

#include <linux/string_helpers.h>
#include <linux/devcoredump.h>
#include <linux/sched/task.h>
#include <uapi/linux/sched/types.h>
#include <linux/sched/mm.h>
#include <linux/utsname.h>

/*
 * Power Management:
 */

static int enable_pwrrail(struct msm_gpu *gpu)
{
	struct drm_device *dev = gpu->dev;
	int ret = 0;

	if (gpu->gpu_reg) {
		ret = regulator_enable(gpu->gpu_reg);
		if (ret) {
			DRM_DEV_ERROR(dev->dev, "failed to enable 'gpu_reg': %d\n", ret);
			return ret;
		}
	}

	if (gpu->gpu_cx) {
		ret = regulator_enable(gpu->gpu_cx);
		if (ret) {
			DRM_DEV_ERROR(dev->dev, "failed to enable 'gpu_cx': %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int disable_pwrrail(struct msm_gpu *gpu)
{
	if (gpu->gpu_cx)
		regulator_disable(gpu->gpu_cx);
	if (gpu->gpu_reg)
		regulator_disable(gpu->gpu_reg);
	return 0;
}

static int enable_clk(struct msm_gpu *gpu)
{
	if (gpu->core_clk && gpu->fast_rate)
		dev_pm_opp_set_rate(&gpu->pdev->dev, gpu->fast_rate);

	/* Set the RBBM timer rate to 19.2Mhz */
	if (gpu->rbbmtimer_clk)
		clk_set_rate(gpu->rbbmtimer_clk, 19200000);

	return clk_bulk_prepare_enable(gpu->nr_clocks, gpu->grp_clks);
}

static int disable_clk(struct msm_gpu *gpu)
{
	clk_bulk_disable_unprepare(gpu->nr_clocks, gpu->grp_clks);

	/*
	 * Set the clock to a deliberately low rate. On older targets the clock
	 * speed had to be non zero to avoid problems. On newer targets this
	 * will be rounded down to zero anyway so it all works out.
	 */
	if (gpu->core_clk)
		dev_pm_opp_set_rate(&gpu->pdev->dev, 27000000);

	if (gpu->rbbmtimer_clk)
		clk_set_rate(gpu->rbbmtimer_clk, 0);

	return 0;
}

static int enable_axi(struct msm_gpu *gpu)
{
	return clk_prepare_enable(gpu->ebi1_clk);
}

static int disable_axi(struct msm_gpu *gpu)
{
	clk_disable_unprepare(gpu->ebi1_clk);
	return 0;
}

int msm_gpu_pm_resume(struct msm_gpu *gpu)
{
	int ret;

	DBG("%s", gpu->name);
	trace_msm_gpu_resume(0);

	ret = enable_pwrrail(gpu);
	if (ret)
		return ret;

	ret = enable_clk(gpu);
	if (ret)
		return ret;

	ret = enable_axi(gpu);
	if (ret)
		return ret;

	msm_devfreq_resume(gpu);

	gpu->needs_hw_init = true;

	return 0;
}

int msm_gpu_pm_suspend(struct msm_gpu *gpu)
{
	int ret;

	DBG("%s", gpu->name);
	trace_msm_gpu_suspend(0);

	msm_devfreq_suspend(gpu);

	ret = disable_axi(gpu);
	if (ret)
		return ret;

	ret = disable_clk(gpu);
	if (ret)
		return ret;

	ret = disable_pwrrail(gpu);
	if (ret)
		return ret;

	gpu->suspend_count++;

	return 0;
}

void msm_gpu_show_fdinfo(struct msm_gpu *gpu, struct msm_context *ctx,
			 struct drm_printer *p)
{
	drm_printf(p, "drm-engine-gpu:\t%llu ns\n", ctx->elapsed_ns);
	drm_printf(p, "drm-cycles-gpu:\t%llu\n", ctx->cycles);
	drm_printf(p, "drm-maxfreq-gpu:\t%u Hz\n", gpu->fast_rate);
}

int msm_gpu_hw_init(struct msm_gpu *gpu)
{
	int ret;

	WARN_ON(!mutex_is_locked(&gpu->lock));

	if (!gpu->needs_hw_init)
		return 0;

	disable_irq(gpu->irq);
	ret = gpu->funcs->hw_init(gpu);
	if (!ret)
		gpu->needs_hw_init = false;
	enable_irq(gpu->irq);

	return ret;
}

#ifdef CONFIG_DEV_COREDUMP
static ssize_t __coredump_print(char *buffer, struct msm_gpu_state *state, loff_t offset,
		size_t count, void *data)
{
	struct msm_gpu *gpu = data;
	struct drm_print_iterator iter;
	struct drm_printer p;

	iter.data = buffer;
	iter.offset = 0;
	iter.start = offset;
	iter.remain = count;

	p = drm_coredump_printer(&iter);

	drm_printf(&p, "---\n");
	drm_printf(&p, "kernel: %s\n", init_utsname()->release);
	drm_printf(&p, "module: " KBUILD_MODNAME "\n");
	drm_printf(&p, "time: %ptSp\n", &state->time);
	if (state->comm)
		drm_printf(&p, "comm: %s\n", state->comm);
	if (state->cmd)
		drm_printf(&p, "cmdline: %s\n", state->cmd);

	gpu->funcs->show(gpu, state, &p);

	return count - iter.remain;
}


static void coredump_print(struct msm_gpu *gpu, struct msm_gpu_state *state)
{
	//without writing to anything, check size of devcoredump to get size for allocation
	gpu->devcd_size = __coredump_print(NULL, state, 0, INT_MAX, (void*)gpu); 

	//now allocate and write the contents into a buffer
	gpu->devcd_saved_buffer = kvmalloc(gpu->devcd_size, GFP_KERNEL);
	ssize_t devcd_size_count_read;
	devcd_size_count_read = __coredump_print(gpu->devcd_saved_buffer, state, 0, gpu->devcd_size, (void*)gpu); 
}
	
static ssize_t msm_gpu_devcoredump_read(char *buffer, loff_t offset, size_t count, void *data, size_t datalen)
{
	struct msm_gpu *gpu = data;

	if (!gpu->devcd_saved_buffer)
	{
		guard(mutex)(&gpu->lock);

		if (!gpu->crashstate)
			return 0;

		coredump_print(gpu, gpu->crashstate);
		if (gpu->funcs->gpu_state_put(gpu->crashstate))
			gpu->crashstate = NULL;
	}

	// Failed allocation so read nothing
	if (!gpu->devcd_saved_buffer)
		return 0;

	ssize_t remain = gpu->devcd_size - offset;
	ssize_t read_count = 0;
	if( remain > 0 && gpu->devcd_size > 0 )
	{
		read_count = remain > count ? count : remain;

		//printk(KERN_INFO "msm_gpu devcd: offset = %d, count = %d, remain = %d", offset, count, remain);

		memcpy(buffer, gpu->devcd_saved_buffer + offset, read_count);
	}

	return read_count;
}

static void msm_gpu_devcoredump_free(void *data)
{
	struct msm_gpu *gpu = data;
	kvfree(gpu->devcd_saved_buffer);
	gpu->devcd_saved_buffer = NULL;
	gpu->devcd_size = 0;
}

static void msm_gpu_crashstate_get_bo(struct msm_gpu_state *state,
				      struct drm_gem_object *obj, u64 iova,
				      bool full, size_t offset, size_t size)
{
	struct msm_gpu_state_bo *state_bo = &state->bos[state->nr_bos];
	struct msm_gem_object *msm_obj = to_msm_bo(obj);

	/* Don't record write only objects */
	state_bo->size = size;
	state_bo->flags = msm_obj->flags;
	state_bo->iova = iova;

	BUILD_BUG_ON(sizeof(state_bo->name) != sizeof(msm_obj->name));

	memcpy(state_bo->name, msm_obj->name, sizeof(state_bo->name));

	if (full) {
		void *ptr;

		state_bo->data = kvmalloc(size, GFP_KERNEL);
		if (!state_bo->data)
			goto out;

		ptr = msm_gem_get_vaddr_active(obj);
		if (IS_ERR(ptr)) {
			kvfree(state_bo->data);
			state_bo->data = NULL;
			goto out;
		}

		memcpy(state_bo->data, ptr + offset, size);
		msm_gem_put_vaddr_locked(obj);
	}
out:
	state->nr_bos++;
}

static void crashstate_get_bos(struct msm_gpu_state *state, struct msm_gem_submit *submit)
{
	extern bool rd_full;

	if (msm_context_is_vmbind(submit->queue->ctx)) {
		struct drm_exec exec;
		struct drm_gpuva *vma;
		unsigned cnt = 0;

		drm_exec_init(&exec, DRM_EXEC_IGNORE_DUPLICATES, 0);
		drm_exec_until_all_locked(&exec) {
			cnt = 0;

			drm_exec_lock_obj(&exec, drm_gpuvm_resv_obj(submit->vm));
			drm_exec_retry_on_contention(&exec);

			drm_gpuvm_for_each_va (vma, submit->vm) {
				if (!vma->gem.obj)
					continue;

				cnt++;
				drm_exec_lock_obj(&exec, vma->gem.obj);
				drm_exec_retry_on_contention(&exec);
			}

		}

		drm_gpuvm_for_each_va (vma, submit->vm)
			cnt++;

		state->bos = kcalloc(cnt, sizeof(struct msm_gpu_state_bo), GFP_KERNEL);

		if (state->bos)
			drm_gpuvm_for_each_va(vma, submit->vm) {
				bool dump = rd_full || (vma->flags & MSM_VMA_DUMP);

				/* Skip MAP_NULL/PRR VMAs: */
				if (!vma->gem.obj)
					continue;

				msm_gpu_crashstate_get_bo(state, vma->gem.obj, vma->va.addr,
							  dump, vma->gem.offset, vma->va.range);
			}

		drm_exec_fini(&exec);
	} else {
		state->bos = kcalloc(submit->nr_bos,
			sizeof(struct msm_gpu_state_bo), GFP_KERNEL);

		for (int i = 0; state->bos && i < submit->nr_bos; i++) {
			struct drm_gem_object *obj = submit->bos[i].obj;
			bool dump = rd_full || (submit->bos[i].flags & MSM_SUBMIT_BO_DUMP);

			msm_gem_lock(obj);
			msm_gpu_crashstate_get_bo(state, obj, submit->bos[i].iova,
						  dump, 0, obj->size);
			msm_gem_unlock(obj);
		}
	}
}

static void crashstate_get_vm_logs(struct msm_gpu_state *state, struct msm_gem_vm *vm)
{
	uint32_t vm_log_len = (1 << vm->log_shift);
	uint32_t vm_log_mask = vm_log_len - 1;
	int first;

	/* Bail if no log, or empty log: */
	if (!vm->log || !vm->log[0].op)
		return;

	mutex_lock(&vm->mmu_lock);

	/*
	 * log_idx is the next entry to overwrite, meaning it is the oldest, or
	 * first, entry (other than the special case handled below where the
	 * log hasn't wrapped around yet)
	 */
	first = vm->log_idx;

	if (!vm->log[first].op) {
		/*
		 * If the next log entry has not been written yet, then only
		 * entries 0 to idx-1 are valid (ie. we haven't wrapped around
		 * yet)
		 */
		state->nr_vm_logs = MAX(0, first - 1);
		first = 0;
	} else {
		state->nr_vm_logs = vm_log_len;
	}

	state->vm_logs = kmalloc_objs(vm->log[0], state->nr_vm_logs);
	if (!state->vm_logs) {
		state->nr_vm_logs = 0;
	}

	for (int i = 0; i < state->nr_vm_logs; i++) {
		int idx = (i + first) & vm_log_mask;

		state->vm_logs[i] = vm->log[idx];
	}

	mutex_unlock(&vm->mmu_lock);
}

static void msm_gpu_crashstate_capture(struct msm_gpu *gpu,
		struct msm_gem_submit *submit, struct msm_gpu_fault_info *fault_info,
		char *comm, char *cmd)
{
	struct msm_gpu_state *state;

	/* Check if the target supports capturing crash state */
	if (!gpu->funcs->gpu_state_get)
		return;

	/* Only save one crash state at a time */
	if (gpu->crashstate || gpu->devcd_saved_buffer)
		return;

	state = gpu->funcs->gpu_state_get(gpu);
	if (IS_ERR_OR_NULL(state))
		return;

	/* Fill in the additional crash state information */
	state->comm = kstrdup(comm, GFP_KERNEL);
	state->cmd = kstrdup(cmd, GFP_KERNEL);
	if (fault_info)
		state->fault_info = *fault_info;

	if (submit && state->fault_info.ttbr0) {
		struct msm_gpu_fault_info *info = &state->fault_info;
		struct msm_mmu *mmu = to_msm_vm(submit->vm)->mmu;

		msm_iommu_pagetable_params(mmu, &info->pgtbl_ttbr0,
					   &info->asid);
		msm_iommu_pagetable_walk(mmu, info->iova, info->ptes);
	}

	if (submit) {
		crashstate_get_vm_logs(state, to_msm_vm(submit->vm));
		crashstate_get_bos(state, submit);
	}

	/* Set the active crash state to be dumped on failure */
	gpu->crashstate = state;

	dev_coredumpm(&gpu->pdev->dev, THIS_MODULE, gpu, 0, GFP_KERNEL,
		msm_gpu_devcoredump_read, msm_gpu_devcoredump_free);
}
#else
static void msm_gpu_crashstate_capture(struct msm_gpu *gpu,
		struct msm_gem_submit *submit, struct msm_gpu_fault_info *fault_info,
		char *comm, char *cmd)
{
}
#endif

/*
 * Hangcheck detection for locked gpu:
 */

static struct msm_gem_submit *
find_submit(struct msm_ringbuffer *ring, uint32_t fence)
{
	struct msm_gem_submit *submit;
	unsigned long flags;

	spin_lock_irqsave(&ring->submit_lock, flags);
	list_for_each_entry(submit, &ring->submits, node) {
		if (submit->seqno == fence) {
			spin_unlock_irqrestore(&ring->submit_lock, flags);
			return submit;
		}
	}
	spin_unlock_irqrestore(&ring->submit_lock, flags);

	return NULL;
}

static void retire_submits(struct msm_gpu *gpu);

static void get_comm_cmdline(struct msm_gem_submit *submit, char **comm, char **cmd)
{
	struct msm_context *ctx = submit->queue->ctx;
	struct task_struct *task;

	WARN_ON(!mutex_is_locked(&submit->gpu->lock));

	/* Note that kstrdup will return NULL if argument is NULL: */
	*comm = kstrdup(ctx->comm, GFP_KERNEL);
	*cmd  = kstrdup(ctx->cmdline, GFP_KERNEL);

	task = get_pid_task(submit->pid, PIDTYPE_PID);
	if (!task)
		return;

	if (!*comm)
		*comm = kstrdup(task->comm, GFP_KERNEL);

	if (!*cmd)
		*cmd = kstrdup_quotable_cmdline(task, GFP_KERNEL);

	put_task_struct(task);
}

static void hangcheck_ring_timer_reset(struct msm_gpu *gpu, struct msm_ringbuffer *ring)
{
	struct msm_drm_private *priv = gpu->dev->dev_private;
	mod_timer(&ring->hangcheck_timer,
			  round_jiffies_up(jiffies + msecs_to_jiffies(priv->hangcheck_period)));
}

static void recover_worker(struct kthread_work *work)
{
	struct msm_gpu *gpu = container_of(work, struct msm_gpu, recover_work);
	struct drm_device *dev = gpu->dev;
	struct msm_drm_private *priv = dev->dev_private;
	struct msm_gem_submit *submit;
	struct msm_ringbuffer *cur_ring;
	char *comm = NULL, *cmd = NULL;
	unsigned int noreclaim_flag;
	struct task_struct *task;
	bool panic_ack_timeout = false;
	int i;

	mutex_lock(&gpu->lock);

	DRM_DEV_ERROR(dev->dev, "%s: hangcheck recover!\n", gpu->name);

	cur_ring = gpu->hung_ring ?
			gpu->hung_ring : gpu->funcs->active_ring(gpu);

	if (gpu->hung_ring) {
		gpu->hung_ring = NULL;

		/*
		 * before doing anything make sure other rings are in a safe state to
		 * reset.
		 *
		 * Set all rings in panic mode.
		 */
		for (i = 0; i < gpu->nr_rings + !!gpu->lpac_rb; i++) {
			struct msm_ringbuffer *ring = i < gpu->nr_rings ? gpu->rb[i] : gpu->lpac_rb;

			ring->memptrs->panic_mode = 0x1;
		}

		uint64_t begin, end;

		begin = ktime_get_boottime_ns();

		/* Busy wait for the other ring to ack it */
		for (;;) {
			bool acked = true;
			for (i = 0; i < gpu->nr_rings + !!gpu->lpac_rb; i++) {
				struct msm_ringbuffer *ring = i < gpu->nr_rings ? gpu->rb[i] : gpu->lpac_rb;

				if (cur_ring == ring)
					continue;

				if (!ring->memptrs->panic_mode_ack)
					acked = false;
			}
			if (acked)
				break;

			end = ktime_get_boottime_ns();

			if ((end - begin) > priv->hangcheck_period * 1000000) {
				panic_ack_timeout = true;
				break;
			}
		}

		end = ktime_get_boottime_ns();
	}

	submit = find_submit(cur_ring, cur_ring->memptrs->fence + 1);

	/*
	 * If the submit retired while we were waiting for the worker to run,
	 * or waiting to acquire the gpu lock, then nothing more to do.
	 */
	if (!submit) {
		for (i = 0; i < gpu->nr_rings + !!gpu->lpac_rb; i++) {
			struct msm_ringbuffer *ring = i < gpu->nr_rings ? gpu->rb[i] : gpu->lpac_rb;

			ring->memptrs->panic_mode = 0x0;
		}

		goto out_unlock;
	}

	/* Increment the fault counts */
	submit->queue->faults++;

	task = get_pid_task(submit->pid, PIDTYPE_PID);
	if (!task)
		gpu->global_faults++;
	else {
		struct msm_gem_vm *vm = to_msm_vm(submit->vm);

		vm->faults++;

		/*
		 * If userspace has opted-in to VM_BIND (and therefore userspace
		 * management of the VM), faults mark the VM as unusable. This
		 * matches vulkan expectations (vulkan is the main target for
		 * VM_BIND).
		 */
		if (!vm->managed)
			msm_gem_vm_unusable(submit->vm);

		put_task_struct(task);
	}

	noreclaim_flag = memalloc_noreclaim_save();

	get_comm_cmdline(submit, &comm, &cmd);

	if (comm && cmd) {
		DRM_DEV_ERROR(dev->dev, "%s: offending task: %s (%s)\n",
			      gpu->name, comm, cmd);

		msm_rd_dump_submit(priv->hangrd, submit,
				   "offending task: %s (%s)", comm, cmd);
	} else {
		DRM_DEV_ERROR(dev->dev, "%s: offending task: unknown\n", gpu->name);

		msm_rd_dump_submit(priv->hangrd, submit, NULL);
	}

	/* Record the crash state */
	pm_runtime_get_sync(&gpu->pdev->dev);
	msm_gpu_crashstate_capture(gpu, submit, NULL, comm, cmd);

	memalloc_noreclaim_restore(noreclaim_flag);

	kfree(cmd);
	kfree(comm);

	/*
	 * Update all the rings with the latest and greatest fence.. this
	 * needs to happen after msm_rd_dump_submit() to ensure that the
	 * bo's referenced by the offending submit are still around.
	 */
	for (i = 0; i < gpu->nr_rings + !!gpu->lpac_rb; i++) {
		struct msm_ringbuffer *ring = i < gpu->nr_rings ? gpu->rb[i] : gpu->lpac_rb;

		uint32_t fence = ring->memptrs->fence;

		/*
		 * For the current (faulting?) ring/submit advance the fence by
		 * one more to clear the faulting submit
		 */
		if (ring == cur_ring)
			ring->memptrs->fence = ++fence;

		msm_update_fence(ring->fctx, fence);
	}

	priv->disable_err_irq = false;

	gpu->funcs->recover(gpu);

	/* retire completed submits, plus the one that hung: */
	retire_submits(gpu);

	/*
	 * Replay all remaining submits starting with highest priority
	 * ring
	 */
	for (i = 0; i < gpu->nr_rings + !!gpu->lpac_rb; i++) {
		struct msm_ringbuffer *ring = i < gpu->nr_rings ? gpu->rb[i] : gpu->lpac_rb;
		unsigned long flags;

		spin_lock_irqsave(&ring->submit_lock, flags);
		list_for_each_entry(submit, &ring->submits, node) {
			/*
			 * If the submit uses an unusable vm make sure
			 * we don't actually run it
			 */
			if (to_msm_vm(submit->vm)->unusable)
				submit->nr_cmds = 0;
			gpu->funcs->submit(gpu, submit);
		}
		spin_unlock_irqrestore(&ring->submit_lock, flags);
	}

	pm_runtime_put(&gpu->pdev->dev);

out_unlock:
	/* make sure hangcheck timers are active on active rings */
	for (i = 0; i < gpu->nr_rings + !!gpu->lpac_rb; i++) {
		struct msm_ringbuffer *ring = i < gpu->nr_rings ? gpu->rb[i] : gpu->lpac_rb;
		if (fence_after(ring->fctx->last_fence, ring->hangcheck_fence))
			hangcheck_ring_timer_reset(gpu, ring);
	}

	mutex_unlock(&gpu->lock);

	msm_gpu_retire(gpu);
}

void msm_gpu_fault_crashstate_capture(struct msm_gpu *gpu, struct msm_gpu_fault_info *fault_info)
{
	struct msm_gem_submit *submit;
	struct msm_ringbuffer *cur_ring = gpu->funcs->active_ring(gpu);
	char *comm = NULL, *cmd = NULL;
	unsigned int noreclaim_flag;

	mutex_lock(&gpu->lock);

	submit = find_submit(cur_ring, cur_ring->memptrs->fence + 1);
	if (submit && submit->fault_dumped)
		goto resume_smmu;

	noreclaim_flag = memalloc_noreclaim_save();

	if (submit) {
		get_comm_cmdline(submit, &comm, &cmd);

		/*
		 * When we get GPU iova faults, we can get 1000s of them,
		 * but we really only want to log the first one.
		 */
		submit->fault_dumped = true;
	}

	/* Record the crash state */
	pm_runtime_get_sync(&gpu->pdev->dev);
	msm_gpu_crashstate_capture(gpu, submit, fault_info, comm, cmd);
	pm_runtime_put_sync(&gpu->pdev->dev);

	memalloc_noreclaim_restore(noreclaim_flag);

	kfree(cmd);
	kfree(comm);

resume_smmu:
	mutex_unlock(&gpu->lock);
}

static bool made_progress(struct msm_gpu *gpu, struct msm_ringbuffer *ring)
{
	if (ring->hangcheck_progress_retries >= DRM_MSM_HANGCHECK_PROGRESS_RETRIES)
		return false;

	if (!gpu->funcs->progress)
		return false;

	if (!gpu->funcs->progress(gpu, ring))
		return false;

	ring->hangcheck_progress_retries++;
	return true;
}

static bool check_ring_timeout(struct msm_ringbuffer *ring, unsigned long timeout)
{
	struct msm_gpu *gpu = ring->gpu;
	u64 start, end;
	int ret;

	if (!gpu->funcs->get_ctx_timestamp)
		return !made_progress(gpu, ring);

	start = ring->memptrs->last_job_start_ctx;

	if (gpu->funcs->get_ctx_timestamp(ring, &end))
		end = ring->memptrs->last_active_ctxcycles;

	if (end >= start)
		return (end - start) > timeout;
	else
		return false;
}

static void hangcheck_handler(struct timer_list *t)
{
	struct msm_ringbuffer *ring = timer_container_of(ring, t, hangcheck_timer);
	struct msm_gpu *gpu = ring->gpu;
	struct msm_drm_private *priv = gpu->dev->dev_private;
	struct drm_device *dev = gpu->dev;
	uint32_t fence = ring->memptrs->fence;

	if (fence != ring->hangcheck_fence) {
		/* some progress has been made.. ya! */
		ring->hangcheck_fence = fence;
		ring->hangcheck_progress_retries = 0;
	} else if (fence_before(fence, ring->fctx->last_fence) &&
			check_ring_timeout(ring, priv->hangcheck_period * 192000)) {
		/* no progress and not done.. hung! */
		ring->hangcheck_fence = fence;
		ring->hangcheck_progress_retries = 0;
		DRM_DEV_ERROR(dev->dev, "%s: hangcheck detected gpu lockup rb %d!\n",
				gpu->name, ring->id);
		DRM_DEV_ERROR(dev->dev, "%s:     completed fence: %u\n",
				gpu->name, fence);
		DRM_DEV_ERROR(dev->dev, "%s:     submitted fence: %u\n",
				gpu->name, ring->fctx->last_fence);

		gpu->hung_ring = ring;
		kthread_queue_work(gpu->worker, &gpu->recover_work);
	}

	/* if still more pending work, reset the hangcheck timer: */
	if (fence_after(ring->fctx->last_fence, ring->hangcheck_fence))
		hangcheck_ring_timer_reset(gpu, ring);

	/* workaround for missing irq: */
	msm_gpu_retire(gpu);
}

/*
 * Cmdstream submission/retirement:
 */

static void retire_submit(struct msm_gpu *gpu, struct msm_ringbuffer *ring,
		struct msm_gem_submit *submit)
{
	int index = submit->seqno % MSM_GPU_SUBMIT_STATS_COUNT;
	volatile struct msm_gpu_submit_stats *stats;
	u64 elapsed, clock = 0, cycles;
	unsigned long flags;

	msm_submitqueue_retire_timestamp(submit->queue, submit->timestamp);

	stats = &ring->memptrs->stats[index];
	/* Convert 19.2Mhz alwayson ticks to nanoseconds for elapsed time */
	elapsed = (stats->alwayson_end - stats->alwayson_start) * 10000;
	do_div(elapsed, 192);

	cycles = stats->cpcycles_end - stats->cpcycles_start;

	/* Calculate the clock frequency from the number of CP cycles */
	if (elapsed) {
		clock = cycles * 1000;
		do_div(clock, elapsed);
	}

	submit->queue->ctx->elapsed_ns += elapsed;
	submit->queue->ctx->cycles     += cycles;

	gpu->elapsed_ns += elapsed;

	trace_msm_gpu_submit_retired(submit, elapsed, clock,
		stats->alwayson_start, stats->alwayson_end);

	msm_submit_retire(submit);

	pm_runtime_mark_last_busy(&gpu->pdev->dev);

	spin_lock_irqsave(&ring->submit_lock, flags);
	list_del(&submit->node);
	spin_unlock_irqrestore(&ring->submit_lock, flags);

	/* Update devfreq on transition from active->idle: */
	mutex_lock(&gpu->active_lock);
	gpu->active_submits--;
	WARN_ON(gpu->active_submits < 0);
	if (!gpu->active_submits) {
		msm_devfreq_idle(gpu);
		pm_runtime_put_autosuspend(&gpu->pdev->dev);
	}

	mutex_unlock(&gpu->active_lock);

	msm_gem_submit_put(submit);
}

static void retire_submits(struct msm_gpu *gpu)
{
	int i;

	/* Retire the commits starting with highest priority */
	for (i = 0; i < gpu->nr_rings + !!gpu->lpac_rb; i++) {
		struct msm_ringbuffer *ring = i < gpu->nr_rings ? gpu->rb[i] : gpu->lpac_rb;

		while (true) {
			struct msm_gem_submit *submit = NULL;
			unsigned long flags;

			spin_lock_irqsave(&ring->submit_lock, flags);
			submit = list_first_entry_or_null(&ring->submits,
					struct msm_gem_submit, node);
			spin_unlock_irqrestore(&ring->submit_lock, flags);

			/*
			 * If no submit, we are done.  If submit->fence hasn't
			 * been signalled, then later submits are not signalled
			 * either, so we are also done.
			 */
			if (submit && dma_fence_is_signaled(submit->hw_fence)) {
				retire_submit(gpu, ring, submit);
			} else {
				break;
			}
		}
	}

	wake_up_all(&gpu->retire_event);
}

static void retire_worker(struct kthread_work *work)
{
	struct msm_gpu *gpu = container_of(work, struct msm_gpu, retire_work);

	retire_submits(gpu);
}

/* call from irq handler to schedule work to retire bo's */
void msm_gpu_retire(struct msm_gpu *gpu)
{
	int i;

	for (i = 0; i < gpu->nr_rings; i++)
		msm_update_fence(gpu->rb[i]->fctx, gpu->rb[i]->memptrs->fence);

	if (gpu->lpac_rb)
		msm_update_fence(gpu->lpac_rb->fctx, gpu->lpac_rb->memptrs->fence);

	kthread_queue_work(gpu->worker, &gpu->retire_work);
}

/* add bo's to gpu's ring, and kick gpu: */
void msm_gpu_submit(struct msm_gpu *gpu, struct msm_gem_submit *submit)
{
	struct msm_ringbuffer *ring = submit->ring;
	unsigned long flags;

	WARN_ON(!mutex_is_locked(&gpu->lock));

	pm_runtime_get_sync(&gpu->pdev->dev);

	msm_gpu_hw_init(gpu);

	submit->seqno = submit->hw_fence->seqno;

	/*
	 * ring->submits holds a ref to the submit, to deal with the case
	 * that a submit completes before msm_ioctl_gem_submit() returns.
	 */
	msm_gem_submit_get(submit);

	spin_lock_irqsave(&ring->submit_lock, flags);
	list_add_tail(&submit->node, &ring->submits);
	spin_unlock_irqrestore(&ring->submit_lock, flags);

	/* Update devfreq on transition from idle->active: */
	mutex_lock(&gpu->active_lock);
	if (!gpu->active_submits) {
		pm_runtime_get(&gpu->pdev->dev);
		msm_devfreq_active(gpu);
	}
	gpu->active_submits++;
	mutex_unlock(&gpu->active_lock);

	gpu->funcs->submit(gpu, submit);
	submit->ring->cur_ctx_seqno = submit->queue->ctx->seqno;

	pm_runtime_put(&gpu->pdev->dev);
	hangcheck_ring_timer_reset(gpu, submit->ring);
}

/*
 * Init/Cleanup:
 */

static irqreturn_t irq_handler(int irq, void *data)
{
	struct msm_gpu *gpu = data;
	return gpu->funcs->irq(gpu);
}

static int get_clocks(struct platform_device *pdev, struct msm_gpu *gpu)
{
	int ret = devm_clk_bulk_get_all(&pdev->dev, &gpu->grp_clks);

	if (ret < 1) {
		gpu->nr_clocks = 0;
		return ret;
	}

	gpu->nr_clocks = ret;

	gpu->core_clk = msm_clk_bulk_get_clock(gpu->grp_clks,
		gpu->nr_clocks, "core");

	gpu->rbbmtimer_clk = msm_clk_bulk_get_clock(gpu->grp_clks,
		gpu->nr_clocks, "rbbmtimer");

	return 0;
}

/* Return a new address space for a msm_drm_private instance */
struct drm_gpuvm *
msm_gpu_create_private_vm(struct msm_gpu *gpu, struct task_struct *task,
			  bool kernel_managed)
{
	struct drm_gpuvm *vm = NULL;

	if (!gpu)
		return NULL;

	/*
	 * If the target doesn't support private address spaces then return
	 * the global one
	 */
	if (gpu->funcs->create_private_vm) {
		vm = gpu->funcs->create_private_vm(gpu, kernel_managed);
		if (!IS_ERR(vm))
			to_msm_vm(vm)->pid = get_pid(task_pid(task));
	}

	if (IS_ERR_OR_NULL(vm))
		vm = drm_gpuvm_get(gpu->vm);

	return vm;
}

int msm_gpu_init(struct drm_device *drm, struct platform_device *pdev,
		struct msm_gpu *gpu, const struct msm_gpu_funcs *funcs,
		const char *name, struct msm_gpu_config *config)
{
	struct msm_drm_private *priv = drm->dev_private;
	extern int enable_lpac;
	int i, ret, nr_rings = config->nr_rings;
	void *memptrs;
	uint64_t memptrs_iova;

	gpu->dev = drm;
	gpu->funcs = funcs;
	gpu->name = name;

	gpu->worker = kthread_run_worker(0, "gpu-worker");
	if (IS_ERR(gpu->worker)) {
		ret = PTR_ERR(gpu->worker);
		gpu->worker = NULL;
		goto fail;
	}

	sched_set_fifo_low(gpu->worker->task);

	/* With kernel.sched_util_clamp_min_rt_default = 64, RT tasks are not guaranteed
	 * the top OPP while runnable. Give gpu-worker a custom floor of 512, which was
	 * experimentally determined to give it sufficient bandwidth to complete its work.
	 */
	{
		struct sched_attr ua = {
			.sched_flags = SCHED_FLAG_KEEP_ALL | SCHED_FLAG_UTIL_CLAMP_MIN,
			.sched_util_min = SCHED_CAPACITY_SCALE / 2,
		};

		if (sched_setattr_nocheck(gpu->worker->task, &ua))
			pr_warn_once("msm: could not set uclamp_min on gpu-worker\n");
	}

	mutex_init(&gpu->active_lock);
	mutex_init(&gpu->lock);
	init_waitqueue_head(&gpu->retire_event);
	kthread_init_work(&gpu->retire_work, retire_worker);
	kthread_init_work(&gpu->recover_work, recover_worker);

	priv->hangcheck_period = DRM_MSM_HANGCHECK_DEFAULT_PERIOD;

	/*
	 * If progress detection is supported, halve the hangcheck timer
	 * duration, as it takes two iterations of the hangcheck handler
	 * to detect a hang.
	 */
	if (funcs->progress)
		priv->hangcheck_period /= 2;

	spin_lock_init(&gpu->id_lock);

	/* Map registers: */
	gpu->mmio = msm_ioremap(pdev, config->ioname);
	if (IS_ERR(gpu->mmio)) {
		ret = PTR_ERR(gpu->mmio);
		goto fail;
	}

	/* Get Interrupt: */
	gpu->irq = platform_get_irq(pdev, 0);
	if (gpu->irq < 0) {
		ret = gpu->irq;
		goto fail;
	}

	ret = devm_request_irq(&pdev->dev, gpu->irq, irq_handler,
			IRQF_TRIGGER_HIGH, "gpu-irq", gpu);
	if (ret) {
		DRM_DEV_ERROR(drm->dev, "failed to request IRQ%u: %d\n", gpu->irq, ret);
		goto fail;
	}

	ret = get_clocks(pdev, gpu);
	if (ret)
		goto fail;

	gpu->ebi1_clk = msm_clk_get(pdev, "bus");
	DBG("ebi1_clk: %p", gpu->ebi1_clk);
	if (IS_ERR(gpu->ebi1_clk))
		gpu->ebi1_clk = NULL;

	/* Acquire regulators: */
	gpu->gpu_reg = devm_regulator_get(&pdev->dev, "vdd");
	DBG("gpu_reg: %p", gpu->gpu_reg);
	if (IS_ERR(gpu->gpu_reg))
		gpu->gpu_reg = NULL;

	gpu->gpu_cx = devm_regulator_get(&pdev->dev, "vddcx");
	DBG("gpu_cx: %p", gpu->gpu_cx);
	if (IS_ERR(gpu->gpu_cx))
		gpu->gpu_cx = NULL;

	platform_set_drvdata(pdev, &gpu->adreno_smmu);
	if (gpu->lpac_pdev)
		platform_set_drvdata(gpu->lpac_pdev, &gpu->lpac_adreno_smmu);

	msm_devfreq_init(gpu);

	gpu->vm = gpu->funcs->create_vm(gpu, pdev);
	if (IS_ERR(gpu->vm)) {
		ret = PTR_ERR(gpu->vm);
		goto fail;
	}

	memptrs = msm_gem_kernel_new(drm,
		sizeof(struct msm_rbmemptrs) * (nr_rings + config->has_lpac),
		check_apriv(gpu, MSM_BO_WC), gpu->vm, &gpu->memptrs_bo,
		&memptrs_iova);

	if (IS_ERR(memptrs)) {
		ret = PTR_ERR(memptrs);
			DRM_DEV_ERROR(drm->dev, "could not allocate memptrs: %d\n", ret);
			goto fail;
		}

		msm_gem_object_set_name(gpu->memptrs_bo, "memptrs");

		if (nr_rings > ARRAY_SIZE(gpu->rb)) {
			DRM_DEV_INFO_ONCE(drm->dev, "Only creating %zu ringbuffers\n",
				ARRAY_SIZE(gpu->rb));
		nr_rings = ARRAY_SIZE(gpu->rb);
	}

	/* Create ringbuffer(s): */
	for (i = 0; i < nr_rings; i++) {
		gpu->rb[i] = msm_ringbuffer_new(gpu, i, memptrs, memptrs_iova);

		if (IS_ERR(gpu->rb[i])) {
			ret = PTR_ERR(gpu->rb[i]);
			gpu->rb[i] = NULL;
			DRM_DEV_ERROR(drm->dev,
				"could not create ringbuffer %d: %d\n", i, ret);
			goto fail;
		}

		timer_setup(&gpu->rb[i]->hangcheck_timer, hangcheck_handler, 0);

		memptrs += sizeof(struct msm_rbmemptrs);
		memptrs_iova += sizeof(struct msm_rbmemptrs);
	}

	if (config->has_lpac && enable_lpac != 0) {
		gpu->lpac_rb = msm_ringbuffer_new(gpu, nr_rings, memptrs, memptrs_iova);

		if (IS_ERR(gpu->lpac_rb)) {
			ret = PTR_ERR(gpu->lpac_rb);
			gpu->lpac_rb = NULL;
			DRM_DEV_ERROR(drm->dev,
						  "could not create lpac ringbuffer %d\n", ret);
			goto fail;
		}

		timer_setup(&gpu->lpac_rb->hangcheck_timer, hangcheck_handler, 0);
	}

	gpu->nr_rings = nr_rings;

	refcount_set(&gpu->sysprof_active, 1);

	mutex_init(&gpu->perfcntr_lock);

	if (gpu->num_perfcntr_groups > 0) {
		gpu->perfcntrs = msm_perfcntr_init(gpu);
		if (IS_ERR(gpu->perfcntrs)) {
			ret = PTR_ERR(gpu->perfcntrs);
			gpu->perfcntrs = NULL;
			goto fail;
		}
	}

	return 0;

fail:
	for (i = 0; i < ARRAY_SIZE(gpu->rb); i++)  {
		msm_ringbuffer_destroy(gpu->rb[i]);
		gpu->rb[i] = NULL;
	}

	if (gpu->lpac_rb) {
		msm_ringbuffer_destroy(gpu->lpac_rb);
		gpu->lpac_rb = NULL;
	}

	msm_gem_kernel_put(gpu->memptrs_bo, gpu->vm);

	platform_set_drvdata(pdev, NULL);
	if (gpu->lpac_pdev)
		platform_set_drvdata(gpu->lpac_pdev, NULL);
	return ret;
}

void msm_gpu_cleanup(struct msm_gpu *gpu)
{
	int i;

	DBG("%s", gpu->name);

	for (i = 0; i < ARRAY_SIZE(gpu->rb); i++) {
		msm_ringbuffer_destroy(gpu->rb[i]);
		gpu->rb[i] = NULL;
	}

	if (gpu->lpac_rb)
		msm_ringbuffer_destroy(gpu->lpac_rb);
	gpu->lpac_rb = NULL;

	msm_gem_kernel_put(gpu->memptrs_bo, gpu->vm);

	if (!IS_ERR_OR_NULL(gpu->vm)) {
		struct msm_mmu *mmu = to_msm_vm(gpu->vm)->mmu;
		mmu->funcs->detach(mmu);
		drm_gpuvm_put(gpu->vm);
	}

	if (gpu->worker) {
		kthread_destroy_worker(gpu->worker);
	}

	msm_devfreq_cleanup(gpu);
	msm_perfcntr_cleanup(gpu);

	platform_set_drvdata(gpu->pdev, NULL);
	if (gpu->lpac_pdev)
		platform_set_drvdata(gpu->lpac_pdev, NULL);
}
