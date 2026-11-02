/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file cpuload.cpp
 *
 * Measurement of CPU load of each individual task.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Petri Tanskanen <petri.tanskanen@inf.ethz.ch>
 */
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform/cpuload.h>

#include <drivers/drv_hrt.h>
#include <nuttx/spinlock.h>

#if defined(__PX4_NUTTX) && defined(CONFIG_SCHED_INSTRUMENTATION)
__BEGIN_DECLS

#if defined(CONFIG_SMP) && defined(CONFIG_ARCH_CHIP_RP23XX)
/* Lock behind px4::atomic read-modify-writes on the RP2350, which has no
 * global exclusive monitor (see px4_platform_common/atomic.h). */
spinlock_t g_px4_atomic_lock = SP_UNLOCKED;
#endif

#ifdef CONFIG_SMP
/* The hooks below run from nxsched_switch_context(), which under SMP always
 * executes with g_cpu_irqlock held by this CPU. In nxtask_exit() it is held
 * through the exiting task while the incoming task's nesting count is still
 * zero, so enter_critical_section() from a hook spins on its own core
 * forever (seen on the RP2350 at the first thread exit). The lock already
 * excludes every reader that takes the critical section, so the 64-bit
 * timestamp is stored directly. */
static inline void cpuload_store_time(volatile hrt_abstime *t)
{
	*t = hrt_absolute_time();
}
#else
static inline void cpuload_store_time(volatile hrt_abstime *t)
{
	hrt_store_absolute_time(t);
}
#endif
# include <nuttx/sched_note.h>

__EXPORT struct system_load_s system_load;

#if defined(CONFIG_SEGGER_SYSVIEW)
#  include <nuttx/note/note_sysview.h>
#  ifndef CONFIG_SEGGER_SYSVIEW_PREFIX
#   error Systemview enabled but prefix is not
#  endif
#endif

static px4::atomic_int cpuload_monitor_all_count{0};

void cpuload_monitor_start()
{
	if (cpuload_monitor_all_count.fetch_add(1) == 0) {
		// if the count was previously 0 (idle threads only) then clear any existing runtime data
#ifdef CONFIG_SMP
		// the chip-wide lock: sched_lock() does not exclude the other core
		irqstate_t flags = enter_critical_section();
#else
		sched_lock();
#endif

		system_load.start_time = hrt_absolute_time();

		// never the idle slots: their counters run from boot
		for (int i = CPULOAD_NCPUS; i < CONFIG_FS_PROCFS_MAX_TASKS; i++) {
			system_load.tasks[i].total_runtime = 0;
			system_load.tasks[i].curr_start_time = 0;
		}

#ifdef CONFIG_SMP
		leave_critical_section(flags);
#else
		sched_unlock();
#endif
	}
}

uint64_t cpuload_idle_runtime(int cpu, uint64_t now)
{
	const system_load_taskinfo_s &idle = system_load.tasks[cpu];
	uint64_t runtime = idle.total_runtime;

	if (idle.valid && idle.tcb != nullptr && idle.tcb->task_state == TSTATE_TASK_RUNNING
	    && now > idle.curr_start_time) {
		runtime += now - idle.curr_start_time;	// the interval still open on that core
	}

	return runtime;
}

void cpuload_monitor_stop()
{
	if (cpuload_monitor_all_count.fetch_sub(1) <= 1) {
		// don't allow the count to go negative
		cpuload_monitor_all_count.store(0);
	}
}

void cpuload_initialize_once()
{
	for (auto &task : system_load.tasks) {
		task.valid = false;
	}

	// threads that exist before this runs, with consecutive PIDs: one idle per
	// core, then init
	int static_tasks_count = CPULOAD_NCPUS + 1;

#ifdef CONFIG_PAGING
	static_tasks_count++;	// include paging thread in initialization
#endif /* CONFIG_PAGING */
#if CONFIG_SCHED_WORKQUEUE
	static_tasks_count++;	// include high priority work0 thread in initialization
#endif /* CONFIG_SCHED_WORKQUEUE */
#if CONFIG_SCHED_LPWORK
	static_tasks_count++;	// include low priority work1 thread in initialization
#endif /* CONFIG_SCHED_WORKQUEUE */

	// perform static initialization of "system" threads
	for (system_load.total_count = 0; system_load.total_count < static_tasks_count; system_load.total_count++) {
		system_load.tasks[system_load.total_count].total_runtime = 0;
		system_load.tasks[system_load.total_count].curr_start_time = 0;
		system_load.tasks[system_load.total_count].tcb = nxsched_get_tcb(
					system_load.total_count);	// it is assumed that these static threads have consecutive PIDs
		system_load.tasks[system_load.total_count].valid = true;
	}

	system_load.initialized = true;
}

void sched_note_start(FAR struct tcb_s *tcb)
{
	// find first free slot
	if (system_load.initialized) {
#ifdef CONFIG_SMP
		irqstate_t flags = enter_critical_section();
#endif

		for (auto &task : system_load.tasks) {
			if (!task.valid) {
				// slot is available
				task.total_runtime = 0;
				task.curr_start_time = 0;
				task.tcb = tcb;
				task.valid = true;
				system_load.total_count++;
				break;
			}
		}

#ifdef CONFIG_SMP
		leave_critical_section(flags);
#endif
	}

#ifdef CONFIG_SEGGER_SYSVIEW
	sysview_sched_note_start(tcb);
#endif
}

/* Lifetime of what readers of the table dereference, under SMP. Both exit
 * paths (task_exit.c, task_terminate.c) call this hook before
 * nxsched_release_tcb() frees the stack and the TCB, so a reader that checks
 * `valid` and finishes its whole read, stack scan included, inside one
 * critical section can never have them freed underneath: this hook takes
 * the same critical section to invalidate the slot. The task group is torn
 * down earlier, in nxtask_exithook before this hook, so readers must not
 * touch tcb->group once TCB_FLAG_EXIT_PROCESSING is set. The callers: task
 * exit holds the lock through the exiting task and bumped the incoming
 * task's nesting count first, task_terminate calls from outside the lock,
 * so taking it here nests or acquires, never spins on itself. */
void sched_note_stop(FAR struct tcb_s *tcb)
{
	if (system_load.initialized) {
#ifdef CONFIG_SMP
		irqstate_t flags = enter_critical_section();
#endif

		for (auto &task : system_load.tasks) {
			if (task.tcb && task.tcb->pid == tcb->pid) {
				// mark slot as free
				task.valid = false;
				task.total_runtime = 0;
				task.curr_start_time = 0;
				task.tcb = nullptr;
				system_load.total_count--;
				break;
			}
		}

#ifdef CONFIG_SMP
		leave_critical_section(flags);
#endif
	}

#ifdef CONFIG_SEGGER_SYSVIEW
	sysview_sched_note_stop(tcb);
#endif
}

void sched_note_suspend(FAR struct tcb_s *tcb)
{
	if (system_load.initialized) {
		if (cpuload_is_idle(tcb)) {
			// one slot per core; the switch path runs under the critical section, which readers take
			system_load.tasks[tcb->pid].total_runtime += hrt_elapsed_time(&system_load.tasks[tcb->pid].curr_start_time);
			return;

		} else {
			if (cpuload_monitor_all_count.load() == 0) {
				return;
			}
		}

		for (auto &task : system_load.tasks) {
			// Task ending its current scheduling run
			if (task.valid && (task.curr_start_time > 0)
			    && task.tcb && task.tcb->pid == tcb->pid) {

				task.total_runtime += hrt_elapsed_time(&task.curr_start_time);
				break;
			}
		}
	}

#ifdef CONFIG_SEGGER_SYSVIEW
	sysview_sched_note_suspend(tcb);
#endif
}

void sched_note_resume(FAR struct tcb_s *tcb)
{
	if (system_load.initialized) {
		if (cpuload_is_idle(tcb)) {
			cpuload_store_time(&system_load.tasks[tcb->pid].curr_start_time);
			return;

		} else {
			if (cpuload_monitor_all_count.load() == 0) {
				return;
			}
		}

		for (auto &task : system_load.tasks) {
			if (task.valid && task.tcb && task.tcb->pid == tcb->pid) {
				// curr_start_time is accessed from an IRQ handler (in logger), so we need
				// to make the update atomic
				cpuload_store_time(&task.curr_start_time);
				break;
			}
		}
	}

#ifdef CONFIG_SEGGER_SYSVIEW
	sysview_sched_note_resume(tcb);
#endif
}

#ifdef CONFIG_SMP
/* NuttX calls these from the CPU start path under CONFIG_SMP with
 * SCHED_INSTRUMENTATION; PX4 keeps no per-CPU trace, so they are no-ops. */
void sched_note_cpu_start(FAR struct tcb_s *tcb, int cpu) { (void)tcb; (void)cpu; }
void sched_note_cpu_started(FAR struct tcb_s *tcb) { (void)tcb; }
void sched_note_cpu_pause(FAR struct tcb_s *tcb, int cpu) { (void)tcb; (void)cpu; }
void sched_note_cpu_paused(FAR struct tcb_s *tcb) { (void)tcb; }
void sched_note_cpu_resume(FAR struct tcb_s *tcb, int cpu) { (void)tcb; (void)cpu; }
void sched_note_cpu_resumed(FAR struct tcb_s *tcb) { (void)tcb; }
#endif

#ifdef CONFIG_SEGGER_SYSVIEW

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
void sched_note_irqhandler(int irq, FAR void *handler, bool enter)
{
	sysview_sched_note_irqhandler(irq, handler, enter);
}
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_SYSCALL
void sched_note_syscall_enter(int nr);
{
	sysview_sched_note_syscall_enter(nr);
}

void sched_note_syscall_enter(int nr);
{
	sysview_sched_note_syscall_enter(nr);
}
#endif

#endif

__END_DECLS
#endif // PX4_NUTTX && CONFIG_SCHED_INSTRUMENTATION
