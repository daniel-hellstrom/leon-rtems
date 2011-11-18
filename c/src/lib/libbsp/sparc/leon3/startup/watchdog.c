#include <bsp.h>

extern volatile LEON3_Timer_Regs_Map *LEON3_Timer_Regs;

struct gptimer_watchdog_priv {
	LEON3_Timer_Regs_Map *regs;
	LEON3_Timer_SubType *timer;
	int timerno;
};

struct gptimer_watchdog_priv bsp_watchdogs[1];
int bsp_watchdog_count = 0;

void bsp_watchdog_init(void)
{
	int timercnt;

	/* Get Watchdogs in system, this is implemented for one GPTIMER
	 * core only.
	 *
	 * First watchdog is a special case, we can get the first tmer core by
	 * looking at LEON3_Timer_Regs, the watchdog within a timer core is always
	 * the last timer. Unfortunately we can not know it the watchdog functionality
	 * is available or not, we assume that it is if we reached this function.
	 */
	bsp_watchdogs[0].regs = (LEON3_Timer_Regs_Map *)LEON3_Timer_Regs;

	/* Find Timer that has watchdog functionality */
	timercnt = bsp_watchdogs[0].regs->status & 0x7;
	if ( timercnt < 2 ) /* First timer system clock timer */
		return;

	bsp_watchdogs[0].timerno = timercnt - 1;
	bsp_watchdogs[0].timer = &bsp_watchdogs[0].regs->timer[bsp_watchdogs[0].timerno];

	bsp_watchdog_count = 1;
}

void bsp_watchdog_reload(int watchdog, unsigned int reload_value)
{
	if ( bsp_watchdog_count == 0 ) {
		bsp_watchdog_init();
	}

	if ( bsp_watchdog_count <= watchdog ) {
		return;
	}

	/* Kick watchdog, and clear interrupt pending bit */
	bsp_watchdogs[watchdog].timer->reload = reload_value;
	bsp_watchdogs[watchdog].timer->conf = (LEON3_GPTIMER_LD | LEON3_GPTIMER_EN) |
	    (bsp_watchdogs[watchdog].timer->conf & ~(1<<4));
}

void bsp_watchdog_stop(int watchdog)
{
	if ( bsp_watchdog_count == 0 ) {
		bsp_watchdog_init();
	}

	if ( bsp_watchdog_count <= watchdog ) {
		return;
	}

	/* Stop watchdog timer */
	bsp_watchdogs[watchdog].timer->conf = 0;
}
