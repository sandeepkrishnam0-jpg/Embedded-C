/*
 ============================================================================
  Power Window Switch (Driver Door)  --  STM32F103 + CAN 2.0B
 ============================================================================
  Target MCU : STM32F103C8 (ARM Cortex-M3, "Blue Pill"), 72 MHz
  Bus        : Body CAN, 500 kbit/s, 11-bit standard identifiers
  Role       : This ECU sits inside the driver's door. It debounces the
               UP/DOWN rocker switch and transmits a CAN frame to the
               Window Motor ECU whenever the switch state changes.

  Hardware wiring:
    PA0  <- UP button   (to GND when pressed, internal pull-up enabled)
    PA1  <- DOWN button (to GND when pressed, internal pull-up enabled)
    PA11 <- CAN_RX      (from transceiver R pin -- e.g. TJA1050)
    PA12 -> CAN_TX      (to   transceiver D pin)
    PC13 -> Status LED  (blinks on TX for diagnostics)

  CAN frame we transmit (11-bit ID = 0x2A0, DLC = 2):
       Byte 0 : window select   0x00=driver, 0x01=passenger,
                                0x02=rear-left, 0x03=rear-right
       Byte 1 : command         0x00=stop, 0x01=up, 0x02=down,
                                0x03=auto-up, 0x04=auto-down

  All register offsets and reset values below are taken from ST document
  RM0008 (STM32F10xxx Reference Manual). Page numbers cited inline.
 ============================================================================
*/

#include <stdint.h>
#include <stdbool.h>

/* ==========================================================================
   1. PERIPHERAL BASE ADDRESSES  (RM0008 sec 3.3 "Memory map", p.50)
   --------------------------------------------------------------------------
   The Cortex-M3 memory map places all peripherals in a fixed region starting
   at 0x40000000. Each peripheral gets a contiguous block; its registers
   are read/written as ordinary memory. That is what "memory-mapped I/O" means.
   ========================================================================== */

#define PERIPH_BASE       0x40000000UL        /* start of peripheral region  */

#define APB1_BASE         (PERIPH_BASE + 0x00000000UL)  /* slow peripheral bus */
#define APB2_BASE         (PERIPH_BASE + 0x00010000UL)  /* fast peripheral bus */
#define AHB_BASE          (PERIPH_BASE + 0x00020000UL)  /* system bus         */

#define RCC_BASE          (AHB_BASE  + 0x00001000UL)    /* 0x40021000 */
#define GPIOA_BASE        (APB2_BASE + 0x00000800UL)    /* 0x40010800 */
#define GPIOC_BASE        (APB2_BASE + 0x00001000UL)    /* 0x40011000 */
#define AFIO_BASE         (APB2_BASE + 0x00000000UL)    /* 0x40010000 */
#define CAN1_BASE         (APB1_BASE + 0x00006400UL)    /* 0x40006400 */

/* ==========================================================================
   2. REGISTER MACROS
   --------------------------------------------------------------------------
   The idiom
       #define REG (*(volatile uint32_t *)ADDR)
   builds a compile-time constant pointer to a 32-bit hardware register and
   dereferences it. `volatile` tells the compiler:
       "Do NOT optimize this access away. The value can change or have side
        effects even without visible code touching it."
   Without volatile, GCC would happily hoist reads outside loops and break
   every driver you write.
   ========================================================================== */

/* --- RCC (Reset & Clock Control), RM0008 sec 7.3, p.99 ------------------- */
#define RCC_CR       (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_CFGR     (*(volatile uint32_t *)(RCC_BASE + 0x04))
#define RCC_APB2ENR  (*(volatile uint32_t *)(RCC_BASE + 0x18))  /* GPIO/AFIO clocks */
#define RCC_APB1ENR  (*(volatile uint32_t *)(RCC_BASE + 0x1C))  /* CAN clock        */

/* --- GPIOA, GPIOC (General Purpose I/O), RM0008 sec 9.2, p.171 ----------- */
#define GPIOA_CRL    (*(volatile uint32_t *)(GPIOA_BASE + 0x00)) /* pins 0..7  */
#define GPIOA_CRH    (*(volatile uint32_t *)(GPIOA_BASE + 0x04)) /* pins 8..15 */
#define GPIOA_IDR    (*(volatile uint32_t *)(GPIOA_BASE + 0x08)) /* input data */
#define GPIOA_ODR    (*(volatile uint32_t *)(GPIOA_BASE + 0x0C)) /* output data*/
#define GPIOA_BSRR   (*(volatile uint32_t *)(GPIOA_BASE + 0x10)) /* atomic set/reset */

#define GPIOC_CRH    (*(volatile uint32_t *)(GPIOC_BASE + 0x04))
#define GPIOC_BSRR   (*(volatile uint32_t *)(GPIOC_BASE + 0x10))

/* --- CAN1, RM0008 sec 24.9, p.679 ---------------------------------------- */
#define CAN1_MCR     (*(volatile uint32_t *)(CAN1_BASE + 0x000))  /* master ctrl   */
#define CAN1_MSR     (*(volatile uint32_t *)(CAN1_BASE + 0x004))  /* master status */
#define CAN1_TSR     (*(volatile uint32_t *)(CAN1_BASE + 0x008))  /* TX status     */
#define CAN1_BTR     (*(volatile uint32_t *)(CAN1_BASE + 0x01C))  /* bit timing    */

/* TX mailbox 0 (three mailboxes exist at 0x180, 0x190, 0x1A0). */
#define CAN1_TI0R    (*(volatile uint32_t *)(CAN1_BASE + 0x180))  /* identifier    */
#define CAN1_TDT0R   (*(volatile uint32_t *)(CAN1_BASE + 0x184))  /* DLC / TGT     */
#define CAN1_TDL0R   (*(volatile uint32_t *)(CAN1_BASE + 0x188))  /* data low 4B   */
#define CAN1_TDH0R   (*(volatile uint32_t *)(CAN1_BASE + 0x18C))  /* data high 4B  */

/* Acceptance filter registers (we open a permissive filter). */
#define CAN1_FMR     (*(volatile uint32_t *)(CAN1_BASE + 0x200))
#define CAN1_FA1R    (*(volatile uint32_t *)(CAN1_BASE + 0x21C))
#define CAN1_F0R1    (*(volatile uint32_t *)(CAN1_BASE + 0x240))
#define CAN1_F0R2    (*(volatile uint32_t *)(CAN1_BASE + 0x244))

/* --- SysTick, ARMv7-M spec (not RM0008) ---------------------------------- */
#define STK_CTRL     (*(volatile uint32_t *)0xE000E010)
#define STK_LOAD     (*(volatile uint32_t *)0xE000E014)
#define STK_VAL      (*(volatile uint32_t *)0xE000E018)

/* ==========================================================================
   3. BIT-FIELD CONSTANTS  (only the ones this driver actually touches)
   ========================================================================== */
#define RCC_APB2ENR_IOPAEN   (1U << 2)   /* enable GPIOA clock  */
#define RCC_APB2ENR_IOPCEN   (1U << 4)   /* enable GPIOC clock  */
#define RCC_APB2ENR_AFIOEN   (1U << 0)   /* enable alt-func I/O */
#define RCC_APB1ENR_CAN1EN   (1U << 25)  /* enable CAN1 clock   */

#define CAN_MCR_INRQ         (1U << 0)   /* request init mode   */
#define CAN_MCR_SLEEP        (1U << 1)   /* sleep mode          */
#define CAN_MSR_INAK         (1U << 0)   /* init mode confirmed */
#define CAN_MSR_SLAK         (1U << 1)   /* sleep mode confirmed*/
#define CAN_TSR_TME0         (1U << 26)  /* mailbox 0 empty     */
#define CAN_TI0R_TXRQ        (1U << 0)   /* transmit request    */

#define CAN_FMR_FINIT        (1U << 0)   /* filter init mode    */

/* ==========================================================================
   4. APPLICATION-LEVEL CONSTANTS
   ========================================================================== */
#define CAN_ID_WINDOW_CMD    0x2A0U      /* 11-bit standard ID  */
#define WINDOW_DRIVER        0x00U
#define CMD_STOP             0x00U
#define CMD_UP               0x01U
#define CMD_DOWN             0x02U

#define PIN_UP               0U          /* PA0 */
#define PIN_DOWN             1U          /* PA1 */

#define DEBOUNCE_MS          30U

/* ==========================================================================
   5. SYSTICK  --  simple millisecond time base
   --------------------------------------------------------------------------
   SysTick is a 24-bit down-counter built into every Cortex-M. When it hits
   zero it reloads and (optionally) fires an interrupt. We poll the COUNTFLAG
   bit and increment a software counter, so millis() below is 100% analogous
   to Arduino's millis().
   ========================================================================== */

static volatile uint32_t g_ms_ticks = 0;

static void systick_init_1ms(void)
{
    /* AHB clock is 72 MHz after PLL config. LOAD = 72_000 - 1 gives a
       1 ms period (down-counter from LOAD to 0 = LOAD+1 cycles). */
    STK_LOAD = 72000UL - 1UL;
    STK_VAL  = 0;
    STK_CTRL = (1U << 2) | (1U << 0);   /* CLKSOURCE = HCLK, ENABLE */
}

static uint32_t millis(void)
{
    /* COUNTFLAG (bit 16) reads 1 if timer wrapped since last read. */
    if (STK_CTRL & (1U << 16)) {
        g_ms_ticks++;
    }
    return g_ms_ticks;
}

/* ==========================================================================
   6. CLOCK & GPIO INITIALIZATION
   ========================================================================== */

static void clocks_enable(void)
{
    /* Every peripheral is gated off after reset to save power. We must turn
       on the bus clock for each peripheral before its registers respond. */
    RCC_APB2ENR |= RCC_APB2ENR_IOPAEN
                |  RCC_APB2ENR_IOPCEN
                |  RCC_APB2ENR_AFIOEN;
    RCC_APB1ENR |= RCC_APB1ENR_CAN1EN;
}

static void gpio_init(void)
{
    /* --- PA0, PA1 : inputs with pull-up (buttons) --------------------
       Each pin gets a 4-bit CNF/MODE field in CRL (pins 0..7) or CRH (8..15).
       Encoding for "input with pull-up/pull-down": CNF=10, MODE=00 -> 0b1000 = 0x8.
       Then the PxODR bit selects pull-UP (1) or pull-DOWN (0). */
    GPIOA_CRL &= ~((0xFU << (PIN_UP   * 4)) | (0xFU << (PIN_DOWN * 4)));
    GPIOA_CRL |=  ((0x8U << (PIN_UP   * 4)) | (0x8U << (PIN_DOWN * 4)));
    GPIOA_ODR |=  (1U << PIN_UP) | (1U << PIN_DOWN);   /* pull-UP */

    /* --- PA11 : CAN_RX (input floating). CNF=01, MODE=00 -> 0b0100 = 0x4 --- */
    GPIOA_CRH &= ~(0xFU << ((11 - 8) * 4));
    GPIOA_CRH |=  (0x4U << ((11 - 8) * 4));

    /* --- PA12 : CAN_TX (alt-func push-pull, 50 MHz). CNF=10, MODE=11 -> 0b1011 = 0xB --- */
    GPIOA_CRH &= ~(0xFU << ((12 - 8) * 4));
    GPIOA_CRH |=  (0xBU << ((12 - 8) * 4));

    /* --- PC13 : general purpose push-pull output, 2 MHz. CNF=00, MODE=10 -> 0b0010 = 0x2 --- */
    GPIOC_CRH &= ~(0xFU << ((13 - 8) * 4));
    GPIOC_CRH |=  (0x2U << ((13 - 8) * 4));
    GPIOC_BSRR = (1U << (13 + 16));   /* upper half = reset -> LED off (active low) */
}

/* ==========================================================================
   7. CAN CONTROLLER INITIALIZATION  (bxCAN, RM0008 sec 24.4, p.646)
   --------------------------------------------------------------------------
   Bit timing math for 500 kbit/s from a 36 MHz APB1 clock:

       BaudRate = f_APB1 / ( BRP * (1 + TS1 + TS2) )
                = 36e6   / ( 9   * (1 + 6   + 1)  )
                = 500 000  bits/s

       Sample point = (1 + TS1) / (1 + TS1 + TS2) = 7/9 = 78 %   (good)

   Register-field convention: each field stores (value - 1).
       BRP register = 9 - 1 = 8
       TS1 register = 6 - 1 = 5
       TS2 register = 1 - 1 = 0
       SJW register = 1 - 1 = 0
   ========================================================================== */

static void can_init_500kbps(void)
{
    /* 1) Exit sleep mode (default power-on state), then request init mode. */
    CAN1_MCR &= ~CAN_MCR_SLEEP;
    CAN1_MCR |=  CAN_MCR_INRQ;
    while (!(CAN1_MSR & CAN_MSR_INAK)) { /* wait for hw ack */ }

    /* 2) Program bit timing while in init mode (BTR is otherwise write-locked). */
    CAN1_BTR = (0U << 24)   /* SJW  = 1 tq          */
             | (0U << 20)   /* TS2  = 1 tq          */
             | (5U << 16)   /* TS1  = 6 tq          */
             | (8U <<  0);  /* BRP  = 9 (prescale)  */

    /* 3) Configure filter bank 0 : 32-bit mask mode, accept everything.
          Filters must be turned off (FINIT=1) before being modified. */
    CAN1_FMR |=  CAN_FMR_FINIT;
    CAN1_FA1R &= ~(1U << 0);          /* deactivate filter 0 */
    CAN1_F0R1  = 0x00000000UL;        /* ID    = 0 */
    CAN1_F0R2  = 0x00000000UL;        /* mask  = 0 -> "don't care" all bits */
    CAN1_FA1R |=  (1U << 0);          /* activate filter 0 */
    CAN1_FMR &= ~CAN_FMR_FINIT;       /* filters now live */

    /* 4) Leave init mode. The controller synchronizes to the bus and is ready. */
    CAN1_MCR &= ~CAN_MCR_INRQ;
    while (CAN1_MSR & CAN_MSR_INAK) { /* wait for hw to leave init */ }
}

/* ==========================================================================
   8. CAN TRANSMIT  --  send one standard frame from mailbox 0
   --------------------------------------------------------------------------
   bxCAN offers 3 TX mailboxes. We only use mailbox 0. The layout of
   CAN_TIxR (RM0008 p.681) is:

     bits 31..21 : STID  (standard 11-bit ID)
     bit  2      : IDE   (0 = standard identifier)
     bit  1      : RTR   (0 = data frame)
     bit  0      : TXRQ  (write 1 to launch transmission)
   ========================================================================== */

static bool can_send(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
    if (!(CAN1_TSR & CAN_TSR_TME0)) {
        return false;                  /* mailbox 0 busy, caller should retry */
    }

    /* Load identifier and control bits, but do NOT set TXRQ yet. */
    CAN1_TI0R = ((uint32_t)(std_id & 0x7FFU) << 21);

    /* DLC lives in the lower 4 bits of TDT (Data Length Code, 0..8). */
    CAN1_TDT0R = (uint32_t)(dlc & 0x0FU);

    /* Pack up to 8 payload bytes into two 32-bit data registers. */
    uint32_t low = 0, high = 0;
    for (uint8_t i = 0; i < dlc && i < 4; i++) { low  |= ((uint32_t)data[i])     << (i * 8); }
    for (uint8_t i = 4; i < dlc && i < 8; i++) { high |= ((uint32_t)data[i]) << ((i - 4) * 8); }
    CAN1_TDL0R = low;
    CAN1_TDH0R = high;

    /* Fire the transmission. Hardware handles arbitration, ACK, retry. */
    CAN1_TI0R |= CAN_TI0R_TXRQ;
    return true;
}

/* ==========================================================================
   9. DEBOUNCE  --  same idea as the traffic-light project, but per-pin
   ========================================================================== */

typedef struct {
    uint8_t  stable;          /* last confirmed level (1 = released, 0 = pressed) */
    uint8_t  last_raw;        /* most recent raw sample                             */
    uint32_t last_change_ms;  /* when raw last flipped                              */
} debounce_t;

static debounce_t db_up   = { 1, 1, 0 };
static debounce_t db_down = { 1, 1, 0 };

/* Returns +1 on a fresh press, -1 on release, 0 otherwise. */
static int8_t debounce_edge(debounce_t *db, uint8_t raw, uint32_t now)
{
    if (raw != db->last_raw) {
        db->last_raw       = raw;
        db->last_change_ms = now;
        return 0;
    }
    if ((now - db->last_change_ms) < DEBOUNCE_MS) return 0;
    if (raw == db->stable) return 0;

    db->stable = raw;
    return (raw == 0) ? +1 : -1;   /* active-low: raw==0 means pressed */
}

/* ==========================================================================
   10. MAIN
   ========================================================================== */

int main(void)
{
    clocks_enable();
    gpio_init();
    systick_init_1ms();
    can_init_500kbps();

    uint8_t last_cmd = CMD_STOP;

    for (;;) {
        uint32_t now = millis();

        /* Read the two button pins from the input-data register.
           Active-low: pin reads 0 when the button is pressed to GND. */
        uint8_t raw_up   = (GPIOA_IDR >> PIN_UP  ) & 1U;
        uint8_t raw_down = (GPIOA_IDR >> PIN_DOWN) & 1U;

        int8_t ev_up   = debounce_edge(&db_up,   raw_up,   now);
        int8_t ev_down = debounce_edge(&db_down, raw_down, now);

        /* Decide the new command from the debounced state. Only transmit
           if it differs from what we already sent (event-driven bus). */
        uint8_t cmd = CMD_STOP;
        if (db_up.stable   == 0) cmd = CMD_UP;    /* held UP    */
        if (db_down.stable == 0) cmd = CMD_DOWN;  /* held DOWN  */
        if (db_up.stable == 0 && db_down.stable == 0) cmd = CMD_STOP; /* both = safety stop */

        (void)ev_up; (void)ev_down;   /* silence unused-var if not needed here */

        if (cmd != last_cmd) {
            uint8_t payload[2] = { WINDOW_DRIVER, cmd };
            if (can_send(CAN_ID_WINDOW_CMD, payload, 2)) {
                last_cmd = cmd;
                /* Blink status LED on successful queue (PC13 is active-low). */
                GPIOC_BSRR = (1U << (13 + 16)); /* LED on  */
                for (volatile uint32_t d = 0; d < 20000; d++) { }
                GPIOC_BSRR = (1U <<  13);       /* LED off */
            }
        }
    }
}

/*
 ============================================================================
  RECEIVING SIDE (for reference, not compiled here)
 ============================================================================
  The Window Motor ECU listens for ID 0x2A0 with a hardware filter and
  drives an H-bridge based on byte 1:

      case CMD_UP:    hbridge_drive(+PWM_DUTY);  break;
      case CMD_DOWN:  hbridge_drive(-PWM_DUTY);  break;
      case CMD_STOP:  hbridge_drive(0);          break;

  In production it also checks a Hall-effect current sensor to detect
  pinch (e.g. a finger caught in the window) and reverses immediately --
  this is FMVSS 118 / UNECE R21 mandated behavior.
 ============================================================================
  WHY EVENT-DRIVEN INSTEAD OF PERIODIC?
 ============================================================================
  A body CAN bus is shared by dozens of ECUs (BCM, doors, seats, mirrors,
  lighting, climate). Flooding it with 10 ms periodic frames per switch
  would starve safety-critical traffic. Convention on comfort networks is:
    - transmit on state change (this file's approach), AND
    - transmit a "heartbeat" copy every 500-1000 ms so a receiver that
      just booted knows the current state without waiting for a press.
 ============================================================================
*/
