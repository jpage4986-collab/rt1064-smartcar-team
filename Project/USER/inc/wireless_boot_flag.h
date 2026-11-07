#ifndef WIRELESS_BOOT_FLAG_H
#define WIRELESS_BOOT_FLAG_H
/* Match NXP SNVS_LP_Init: a latched power glitch zeroizes LPGPR until PGD
 * is initialized and its previous status acknowledged. Do not disable tamper
 * protection or alter unrelated status bits. */
static inline void su7_boot_flag_init(void)
{
    CLOCK_EnableClock(kCLOCK_SnvsHp);
    CLOCK_EnableClock(kCLOCK_SnvsLp);
    SNVS->HPCOMR |= SNVS_HPCOMR_NPSWA_EN_MASK;
    SNVS->LPPGDR = 0x41736166U;
    SNVS->LPSR = SNVS_LPSR_PGD_MASK;
    __DSB();
}
#endif
