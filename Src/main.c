/**
 ******************************************************************************
 * @file    main.c
 * @brief   Bare-metal LED bring-up (Nucleo-144 MB1549, STM32U5)
 *
 * LD1: PC7  (GPIOC)
 * LD2: PB7  (GPIOB)
 * LD3: PG2  (GPIOG, requires VDDIO2 validation via PWR->SVMCR.IO2SV)
 ******************************************************************************
 */

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning "FPU is not initialized, but the project is compiling for an FPU."
#endif

/* ----------------------------- MMIO helpers ------------------------------ */

#define MMIO32(addr) (*(volatile uint32_t *)(addr))

#define BIT(n)       (1UL << (n))

/* Delay/ordering barrier after enabling clocks (flush write buffer). */
#define RCC_READBACK(reg) do { (void)(reg); } while (0)

/* ------------------------------ RCC registers ---------------------------- */

#define RCC_BASE            (0x56020C00UL)

#define RCC_AHB2ENR1        MMIO32(RCC_BASE + 0x08CUL)  /* GPIOxEN bits */
#define RCC_AHB3ENR         MMIO32(RCC_BASE + 0x094UL)  /* PWREN bit     */

#define RCC_AHB3ENR_PWREN   BIT(2)                      /* PWREN bit pos */

/* AHB2ENR1 GPIO enable bits */
#define RCC_AHB2ENR1_GPIOBEN BIT(1)
#define RCC_AHB2ENR1_GPIOCEN BIT(2)
#define RCC_AHB2ENR1_GPIOGEN BIT(6)

/* ------------------------------ PWR registers ---------------------------- */

#define PWR_BASE            (0x56020800UL)

#define PWR_SVMCR           MMIO32(PWR_BASE + 0x10UL)
#define PWR_SVMCR_IO2SV     BIT(29)  /* Validate VDDIO2 for PG[15:2] */

/* ------------------------------ GPIO registers --------------------------- */

#define GPIOB_BASE          (0x52020400UL)
#define GPIOC_BASE          (0x52020800UL)
#define GPIOG_BASE          (0x52021800UL)

#define GPIO_MODER(base)    MMIO32((base) + 0x00UL)
#define GPIO_OTYPER(base)   MMIO32((base) + 0x04UL)
#define GPIO_BSRR(base)     MMIO32((base) + 0x18UL)

/* ------------------------------ GPIO bitfields --------------------------- */

#define GPIO_PIN(n)         (1UL << (n))

/* MODER: 2 bits per pin */
#define GPIO_MODER_SHIFT(pin)   ((pin) * 2U)
#define GPIO_MODER_MASK(pin)    (0x3UL << GPIO_MODER_SHIFT(pin))
#define GPIO_MODER_OUTPUT(pin)  (0x1UL << GPIO_MODER_SHIFT(pin)) /* 01 */

/* OTYPER: 1 bit per pin (0=push-pull) */
#define GPIO_OTYPER_MASK(pin)   (1UL << (pin))

/* BSRR: atomic set/reset */
#define GPIO_BSRR_SET(pin)      (1UL << (pin))
#define GPIO_BSRR_RESET(pin)    (1UL << ((pin) + 16U))

/* ------------------------------ Board LEDs ------------------------------- */

#define LD1_PIN             7U  /* PC7 */
#define LD2_PIN             7U  /* PB7 */
#define LD3_PIN             2U  /* PG2 */

/* ------------------------------ Local helpers ---------------------------- */

static inline void gpio_enable_clock(uint32_t rcc_ahb2enr1_mask)
{
    RCC_AHB2ENR1 |= rcc_ahb2enr1_mask;
    RCC_READBACK(RCC_AHB2ENR1);
}

static inline void pwr_enable_clock(void)
{
    RCC_AHB3ENR |= RCC_AHB3ENR_PWREN;
    RCC_READBACK(RCC_AHB3ENR);
}

static inline void vddio2_validate(void)
{
    /* Mandatory to use PG[15:2] */
    PWR_SVMCR |= PWR_SVMCR_IO2SV;
    (void)PWR_SVMCR;
}

static inline void gpio_config_output_pp(uint32_t gpio_base, uint32_t pin)
{
    /* Output mode */
    GPIO_MODER(gpio_base) =
        (GPIO_MODER(gpio_base) & ~GPIO_MODER_MASK(pin)) | GPIO_MODER_OUTPUT(pin);

    /* Push-pull */
    GPIO_OTYPER(gpio_base) &= ~GPIO_OTYPER_MASK(pin);
}

static inline void gpio_set(uint32_t gpio_base, uint32_t pin)
{
    GPIO_BSRR(gpio_base) = GPIO_BSRR_SET(pin);
}

/* ------------------------------ main ------------------------------------- */

int main(void)
{
    /* LD1 (PC7) */
    gpio_enable_clock(RCC_AHB2ENR1_GPIOCEN);
    gpio_config_output_pp(GPIOC_BASE, LD1_PIN);
    gpio_set(GPIOC_BASE, LD1_PIN);

    /* LD2 (PB7) */
    gpio_enable_clock(RCC_AHB2ENR1_GPIOBEN);
    gpio_config_output_pp(GPIOB_BASE, LD2_PIN);
    gpio_set(GPIOB_BASE, LD2_PIN);

    /* LD3 (PG2) - requires VDDIO2 validation */
    pwr_enable_clock();
    vddio2_validate();
    gpio_enable_clock(RCC_AHB2ENR1_GPIOGEN);
    gpio_config_output_pp(GPIOG_BASE, LD3_PIN);
    gpio_set(GPIOG_BASE, LD3_PIN);

    for (;;)
    {
        /* idle */
    }
}
