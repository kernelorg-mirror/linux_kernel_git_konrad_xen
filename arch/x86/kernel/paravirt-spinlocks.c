/*
 * Split spinlock implementation out into its own file, so it can be
 * compiled in a FTRACE-compatible way.
 */
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/jump_label.h>

#include <asm/paravirt.h>

u64 taken_slow;
u64 released_slow;

EXPORT_SYMBOL(taken_slow);
EXPORT_SYMBOL(released_slow);

static void test_lock_spinning(struct arch_spinlock *lock, __ticket_t want)
{
       taken_slow++;
}
PV_CALLEE_SAVE_REGS_THUNK(test_lock_spinning);

static void test_unlock_kick(struct arch_spinlock *lock, __ticket_t ticket)
{
       released_slow++;
}

struct pv_lock_ops pv_lock_ops = {
#ifdef CONFIG_SMP
	.lock_spinning = __PV_IS_CALLEE_SAVE(test_lock_spinning),
	.unlock_kick = test_unlock_kick,
#endif
};
EXPORT_SYMBOL(pv_lock_ops);

struct static_key paravirt_ticketlocks_enabled = STATIC_KEY_INIT_TRUE;
EXPORT_SYMBOL(paravirt_ticketlocks_enabled);
