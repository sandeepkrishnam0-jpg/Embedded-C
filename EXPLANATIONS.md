# Embedded C Concepts — Companion Guide

This document walks through every concept illustrated by the two projects in
this repository: the Arduino **traffic light controller** and the bare-metal
STM32 **power window switch on CAN bus**.

It is meant to be read alongside the source files.

---

## Table of Contents

1. [Project 1 — Pedestrian Traffic Light (Arduino)](#project-1--pedestrian-traffic-light-arduino)
2. [Project 2 — Power Window Switch (STM32 + CAN)](#project-2--power-window-switch-stm32--can)
3. [Language Concepts](#language-concepts)
   - [Fixed-Width Integer Types](#fixed-width-integer-types)
   - [Signed vs Unsigned](#signed-vs-unsigned)
   - [Integer Literal Suffixes (`UL`, `U`, `LL`, `ULL`)](#integer-literal-suffixes-ul-u-ll-ull)
   - [Logical AND `&&` (with short-circuit evaluation)](#logical-and--with-short-circuit-evaluation)
   - [Left Shift `<<` and Bit Masks](#left-shift--and-bit-masks)
   - [Bitwise OR-Assign `|=` — Setting Bits](#bitwise-or-assign--setting-bits)
   - [Bitwise AND-NOT `&= ~mask` — Clearing Bits](#bitwise-and-not---mask--clearing-bits)
4. [Embedded Systems Patterns](#embedded-systems-patterns)
   - [State Machines: Entry vs Update Code](#state-machines-entry-vs-update-code)
   - [Non-Blocking Timing with `millis()` / SysTick](#non-blocking-timing-with-millis--systick)
   - [Software Debouncing](#software-debouncing)
   - [The Clear-Then-Set Pattern for Multi-Bit Fields](#the-clear-then-set-pattern-for-multi-bit-fields)
5. [Hardware Register Programming](#hardware-register-programming)
   - [Memory-Mapped I/O and `volatile`](#memory-mapped-io-and-volatile)
   - [Peripheral Base Addresses (Building Them With `+`)](#peripheral-base-addresses-building-them-with-)
   - [Where Register Info Comes From — the Reference Manual](#where-register-info-comes-from--the-reference-manual)
   - [Reset Values (Why We Don't Write "Reset Code")](#reset-values-why-we-dont-write-reset-code)
   - [Three Init Techniques: Direct Write, RMW, Full Overwrite](#three-init-techniques-direct-write-rmw-full-overwrite)
6. [CAN Bus Deep Dive](#can-bus-deep-dive)
   - [Bit Timing Math](#bit-timing-math)
   - [Event-Driven vs Periodic Transmission](#event-driven-vs-periodic-transmission)

---

## Project 1 — Pedestrian Traffic Light (Arduino)

**File:** `traffic_light/traffic_light.ino`

A traffic light for cars (RED/YELLOW/GREEN) with a pedestrian WALK button.
When someone presses the button, the light cycles through YELLOW → RED,
lets pedestrians walk, then returns to GREEN.

### Hardware

- RED LED → pin 8 (+ 220 Ω to GND)
- YELLOW LED → pin 9
- GREEN LED → pin 10
- WALK LED → pin 11
- Push button → pin 2 (uses `INPUT_PULLUP`)

### Why Four States for Three Colors?

A real traffic light only has 3 colors, but the state machine has 4 states:

| State | RED | YELLOW | GREEN | WALK | Meaning | Duration |
|---|---|---|---|---|---|---|
| `STATE_GREEN` | off | off | **on** | off | Cars go | ≥5 s |
| `STATE_YELLOW` | off | **on** | off | off | Cars slow | 2 s |
| `STATE_RED_WALK` | **on** | off | off | **on** | Pedestrians walk | 4 s |
| `STATE_RED_HOLD` | **on** | off | off | off | Clear the crosswalk | 1 s |

The RED phase is split into two states because the **outputs differ** (WALK
LED on vs off) and the **durations differ** (4 s vs 1 s). Any time two
"phases" have different outputs or different exit conditions, they should
be different states.

**FSM design rule:** *A new state is worth adding whenever the outputs OR
the exit conditions change.*

---

## Project 2 — Power Window Switch (STM32 + CAN)

**File:** `power_window_switch/power_window_switch.c`

The same conceptual project (a switch reporting button events) but ported
to the way an automotive supplier would actually ship it: bare-metal STM32
firmware sending CAN messages to a window motor ECU.

### Hardware

- MCU: STM32F103C8 (Cortex-M3 @ 72 MHz)
- PA0 ← UP button (internal pull-up)
- PA1 ← DOWN button (internal pull-up)
- PA11 ← CAN_RX (from TJA1050 transceiver)
- PA12 → CAN_TX (to TJA1050 transceiver)
- PC13 → status LED (blinks on TX)

### CAN Frame Format

| Field | Value |
|---|---|
| ID | 0x2A0 (11-bit standard) |
| DLC | 2 bytes |
| Byte 0 | window select: 0=driver, 1=passenger, 2=RL, 3=RR |
| Byte 1 | command: 0=stop, 1=up, 2=down, 3=auto-up, 4=auto-down |

### Comparison with Project 1

|  | Traffic Light | Power Window Switch |
|---|---|---|
| Target | Arduino Uno (AVR, 16 MHz) | STM32F103 (Cortex-M3, 72 MHz) |
| Toolchain | Arduino IDE | `arm-none-eabi-gcc` |
| I/O method | `digitalWrite(pin, val)` | Raw register writes |
| Communication | Serial print | **CAN 2.0B at 500 kbit/s** |
| Time base | Arduino's `millis()` | Custom SysTick |
| Abstraction | Everything hidden | Nothing hidden |

---

## Language Concepts

### Fixed-Width Integer Types

Standard header: `<stdint.h>`. Always prefer these on embedded targets:

| Signed | Unsigned | Bits | Range (unsigned) |
|---|---|---|---|
| `int8_t`  | `uint8_t`  | 8  | 0..255 |
| `int16_t` | `uint16_t` | 16 | 0..65,535 |
| `int32_t` | `uint32_t` | 32 | 0..4,294,967,295 |
| `int64_t` | `uint64_t` | 64 | 0..~1.8×10¹⁹ |

**Why not just use `int`?** Because `int` is **16 bits on Arduino Uno but
32 bits on most PCs**. Code breaks silently when you move it. `uint16_t` is
16 bits *everywhere*.

---

### Signed vs Unsigned

A byte holds 256 possible bit patterns. What those patterns mean depends on
the type:

| Type | Bits | Range | Purpose |
|---|---|---|---|
| `uint8_t` | 8 | **0 to 255** | Only non-negative values |
| `int8_t`  | 8 | **−128 to +127** | Can be negative |

The **same bits in memory** mean different things depending on the type you
declared. `1111 1111` is either **255** (unsigned) or **−1** (signed,
two's complement).

**Use unsigned for:** counters, sizes, addresses, timers, register values,
anything you'll do bitwise ops on.

**Use signed for:** sensor deltas, PID errors, coordinates that can go
negative, anything meaningfully bidirectional.

**Classic bug — signed/unsigned mixup:**

```c
uint8_t volume = 0;
volume = volume - 1;   // Not -1! It becomes 255 (wraps around)
```

Unsigned types **wrap around silently** on underflow. Signed overflow is
*undefined behavior* in C. Never mix the two in an expression without a
cast.

---

### Integer Literal Suffixes (`UL`, `U`, `LL`, `ULL`)

Suffixes on numeric constants tell the compiler what type the literal is:

| Suffix | Meaning | Example | Type (typical) |
|---|---|---|---|
| *(none)* | plain int | `5000` | `int` |
| `U` | Unsigned | `5000U` | `unsigned int` |
| `L` | Long | `5000L` | `long` |
| `UL` | Unsigned Long | `5000UL` | `unsigned long` |
| `LL` | Long Long | `5000LL` | `long long` (64-bit) |
| `ULL` | Unsigned Long Long | `5000ULL` | `unsigned long long` |

**Why `T_GREEN_MS 5000UL` in the traffic light code?** Because `millis()`
returns `uint32_t`. Without `UL`, the constant would be a 16-bit `int` on
AVR, and any timing value > 32,767 (~32 seconds) would misbehave.

**Bug this prevents:**

```c
uint32_t ms = 1000 * 60 * 60;      // WRONG: overflows to 30720 on AVR
uint32_t ms = 1000UL * 60 * 60;    // RIGHT: promotes to 32-bit
```

**Rule of thumb:** whenever a constant will interact with a `uint32_t`
(timers, addresses, register values), give it a `UL` suffix.

---

### Logical AND `&&` (with short-circuit evaluation)

`A && B` is true only if **both** A and B are true.

```c
if (walk_requested && elapsed >= T_GREEN_MS) {
    enter_state(STATE_YELLOW);
}
```

The light changes only when someone pressed the button **AND** at least
5 seconds have passed on green.

**Short-circuit evaluation:** C evaluates `&&` left-to-right and **stops
early** if the answer is already known. If `walk_requested` is false, C
never evaluates the right side. This is why the following is safe:

```c
if (ptr != NULL && ptr->value > 0) { ... }
```

The NULL check must come first — `ptr->value` would crash otherwise.

**Don't confuse with `&` (single ampersand):**

| Operator | Name | Operates on |
|---|---|---|
| `&&` | Logical AND | true/false conditions |
| `&` | Bitwise AND | individual bits |

---

### Left Shift `<<` and Bit Masks

`<<` slides bits left, filling with zeros on the right. `1U << N` puts a
single `1` bit at position N — a mask targeting exactly one bit.

```
1U        =  0000 0000 0000 0000 0000 0000 0000 0001
1U << 2   =  0000 0000 0000 0000 0000 0000 0000 0100
1U << 25  =  0000 0010 0000 0000 0000 0000 0000 0000
```

**Why we need this:** hardware registers pack many controls into one word.
The STM32's `RCC_APB2ENR` has 32 different enable bits — one for each
peripheral. To turn on just GPIOA (bit 2) without touching the other 31
features, you need a mask with only bit 2 set: `1U << 2 = 0x00000004`.

**Why `1U` and not `1`?**

- `1 << 31` is **undefined behavior** — shifting into the sign bit of a
  signed int.
- On 16-bit compilers, `1 << 25` overflows.
- `1U` is at least 16-bit unsigned; combined with `|=` on a 32-bit register
  it gets promoted correctly.

**Multi-bit fields:** `0xFU << 4` creates a 4-bit-wide mask starting at
bit 4. This is how we address GPIO configuration slots (4 bits per pin).

---

### Bitwise OR-Assign `|=` — Setting Bits

`x |= y` is shorthand for `x = x | y`. Turns bits ON without touching the
others.

```c
RCC_APB2ENR |= (1U << 2);   // turn ON GPIOA clock
```

Trace:

```
current value:      xxxx xxxx xxxx xxxx xxxx xxxx xxx0 xxxx
mask (1U << 2):     0000 0000 0000 0000 0000 0000 0000 0100
                                                       OR
result:             xxxx xxxx xxxx xxxx xxxx xxxx xxx1 xxxx
                                                       ^
                                         only bit 2 changed
```

**Why not plain `=`?** Because that would wipe out every other bit in the
register.

```c
RCC_APB2ENR = (1U << 2);   // WRONG — turns off every other peripheral!
```

**Golden rule:** in a shared hardware register, always use `|=` to set
bits and `&= ~` to clear bits — never plain `=`.

---

### Bitwise AND-NOT `&= ~mask` — Clearing Bits

The mirror image of `|=`. Turns bits OFF without disturbing others.

```c
CAN1_MCR &= ~CAN_MCR_SLEEP;   // clear the SLEEP bit
```

Two operators combined:

1. `~mask` flips every bit — every `1` becomes `0`, every `0` becomes `1`.
2. `&=` AND-assigns.

Trace with `CAN_MCR_SLEEP = (1U << 1)`:

```
mask (1U << 1):     0000 0000 0000 0000 0000 0000 0000 0010
~mask:              1111 1111 1111 1111 1111 1111 1111 1101
                                                        ^
                                              the only 0 is at bit 1
current CAN1_MCR:   xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxxx
                                                        AND
result:             xxxx xxxx xxxx xxxx xxxx xxxx xxxx xxx0
                                                        ^
                                          bit 1 forced to 0
```

Every `x` bit stayed as `x` (because `x AND 1 = x`), but bit 1 got forced
to `0` (because `x AND 0 = 0`).

**The complete bit-manipulation vocabulary:**

| Goal | Idiom | Example |
|---|---|---|
| **Set** a bit | `REG \|= mask` | `RCC_APB2ENR \|= RCC_APB2ENR_IOPAEN;` |
| **Clear** a bit | `REG &= ~mask` | `CAN1_MCR &= ~CAN_MCR_SLEEP;` |
| **Toggle** a bit | `REG ^= mask` | `GPIOC_ODR ^= (1U << 13);` |
| **Test** a bit | `if (REG & mask)` | `while (!(CAN1_MSR & CAN_MSR_INAK));` |

Master these four and you can drive any peripheral on any MCU.

---

## Embedded Systems Patterns

### State Machines: Entry vs Update Code

Every FSM has two categories of work:

| Category | When it runs | Example |
|---|---|---|
| **Entry actions** | ONCE, when we enter a state | Turn LEDs on, start a timer, log |
| **Update actions** | EVERY loop iteration | Check exit conditions |

**Wrong (everything in `loop()`):**

```c
if (state == STATE_GREEN) {
    set_lights(LOW, LOW, HIGH, LOW);   // runs 10,000× per second
    Serial.println("-> GREEN");        // spams the log
    if (walk_requested) state = STATE_YELLOW;
}
```

**Right (entry actions in a dedicated function):**

```c
static void enter_state(traffic_state_t next) {
  state = next;
  state_entered_ms = millis();
  switch (state) {
    case STATE_GREEN:
      set_lights(LOW, LOW, HIGH, LOW);  // fires ONCE per transition
      Serial.println("-> GREEN");
      break;
    ...
  }
}

void loop() {
  switch (state) {
    case STATE_GREEN:
      if (walk_requested && elapsed >= T_GREEN_MS) {
        enter_state(STATE_YELLOW);
      }
      break;
  }
}
```

Benefits:
- **Correctness** — `state_entered_ms` resets exactly on transition
- **Performance** — no wasted `digitalWrite()` calls
- **Clean logs** — one line per transition, not a flood
- **Debuggability** — breakpoint on `enter_state()` catches every change

---

### Non-Blocking Timing with `millis()` / SysTick

**The golden rule of embedded firmware:** `loop()` finishes fast and comes
right back. **Never use `delay()`** in production code — it freezes the CPU
for the duration of the delay, blocking all other work.

The idiom:

```c
uint32_t now = millis();
if ((now - last_event_ms) >= INTERVAL_MS) {
    last_event_ms = now;
    do_something();
}
```

This runs `do_something()` every `INTERVAL_MS` while allowing the loop to
service other things (buttons, sensors, radios) in between.

**On STM32 there's no built-in `millis()`** — you build one from **SysTick**,
the 24-bit down-counter inside every ARM Cortex-M. Configure it to reload
every 72,000 CPU cycles (= 1 ms at 72 MHz) and count ticks in a variable.

---

### Software Debouncing

A mechanical switch's contacts physically bounce for a few milliseconds
when pressed — one press can look like 5–10 presses to the CPU. Software
must filter this noise.

**The pattern:** only accept a new "stable" reading after it has held
unchanged for `DEBOUNCE_MS` (typically 20–50 ms).

```c
if (raw != last_raw) {
    last_raw = raw;
    last_change_ms = now;   // still bouncing — restart timer
    return;
}
if ((now - last_change_ms) < DEBOUNCE_MS) {
    return;                 // not stable long enough yet
}
if (raw != stable) {
    stable = raw;           // accept the new state
    // fire event here
}
```

This exact pattern appears in both projects.

---

### The Clear-Then-Set Pattern for Multi-Bit Fields

Used to write into hardware register fields that are **wider than 1 bit**.
Example — configuring a 4-bit GPIO pin slot:

```c
GPIOA_CRL &= ~((0xFU << (PIN_UP * 4)) | (0xFU << (PIN_DOWN * 4)));  // clear
GPIOA_CRL |=  ((0x8U << (PIN_UP * 4)) | (0x8U << (PIN_DOWN * 4)));  // set
```

**Why two steps?** Because `|=` alone can only turn bits ON, never OFF. If
a slot was `0b0100` (floating input, reset default) and you tried
`|= 0b1000`, you'd get `0b1100` — which encodes something entirely
different from what you wanted.

**The ritual:**

```c
REG &= ~mask;       // 1. wipe the field to zeros
REG |= new_value;   // 2. write the value in fresh
```

For **single-bit** operations (like setting one clock-enable bit), `|=`
alone is fine because there's no other bit in the "field" to worry about.

---

## Hardware Register Programming

### Memory-Mapped I/O and `volatile`

Every peripheral register lives at a **fixed memory address**. Reading or
writing that address goes straight to the hardware, not to RAM.

```c
#define GPIOA_IDR  (*(volatile uint32_t *)0x40010808)
```

Breaking it down:

- `0x40010808` — the physical address of GPIOA's input data register
- `(volatile uint32_t *)` — cast to a pointer to a 32-bit unsigned value
- `*` — dereference to read/write the value at that address
- The outer parentheses form a valid lvalue: `GPIOA_IDR = 0x1234;` writes,
  `x = GPIOA_IDR;` reads.

**Why `volatile`?**

Without it, the compiler could optimize your driver like this:

```c
while (CAN1_MSR & CAN_MSR_INAK) { }   // wait for hw ack
```

The compiler would notice `CAN1_MSR` isn't modified inside the loop and
transform it into an infinite loop (because "the value can't change").
`volatile` tells the compiler: **the value can change without visible
code touching it** — reload it from memory every time.

**Every hardware register pointer must be `volatile`.** Forgetting this is
one of the top three causes of "worked in debug, broke in release" bugs.

---

### Peripheral Base Addresses (Building Them With `+`)

Addresses are constructed hierarchically to match the chip's memory map:

```c
#define PERIPH_BASE   0x40000000UL
#define APB2_BASE    (PERIPH_BASE + 0x00010000UL)
#define GPIOA_BASE   (APB2_BASE   + 0x00000800UL)
```

Expands to `0x40000000 + 0x10000 + 0x800 = 0x40010800` — the actual
address where the STM32F103's GPIOA registers begin.

Layered like this:

```
0x40000000 ─┬─ PERIPH_BASE
            ├─ +0x00000 APB1 bus  (TIM2, USART2, CAN1, I2C1)
            │      └─ +0x6400  CAN1_BASE   = 0x40006400
            ├─ +0x10000 APB2 bus  (GPIO ports, ADC, USART1, SPI1)
            │      ├─ +0x0000  AFIO_BASE   = 0x40010000
            │      ├─ +0x0800  GPIOA_BASE  = 0x40010800
            │      └─ +0x1000  GPIOC_BASE  = 0x40011000
            └─ +0x20000 AHB bus  (DMA, RCC, Flash)
                   └─ +0x1000  RCC_BASE    = 0x40021000
```

All the additions happen at **compile time**. The generated assembly is
identical to writing the full address by hand — you get readability
without paying for it in speed or code size.

---

### Where Register Info Comes From — the Reference Manual

Every chip vendor publishes two key documents:

| Document | Contents | Size |
|---|---|---|
| **Datasheet** | Electrical specs, pinout, packages, power consumption | ~120 pages |
| **Reference Manual** | Every peripheral, every register, every bit | **~1100 pages** |

For STM32F103: **RM0008**. Free download at st.com. Google "RM0008 pdf".

**Workflow:**

1. **Section 3.3 "Memory map"** (p. 50) — get base addresses.
2. **Per-peripheral chapter** — get register offsets and bit descriptions.
3. **"Configuration procedure"** subsections — get the required init
   sequence. For example, RM0008 §24.4.2 spells out the exact steps for
   bringing up CAN.

**The layer cake of embedded abstraction:**

```
┌─────────────────────────────────────────┐
│  Your application code                  │  "turn on the LED"
├─────────────────────────────────────────┤
│  HAL_GPIO_WritePin(GPIOA, LD2, SET)     │  ST HAL library (bloated)
├─────────────────────────────────────────┤
│  LL_GPIO_SetOutputPin(GPIOA, LD2)       │  ST Low-Level lib (thin)
├─────────────────────────────────────────┤
│  GPIOA->BSRR = (1U << 5);               │  CMSIS registers (production)
├─────────────────────────────────────────┤
│  *(volatile uint32_t *)0x40010810 = 32; │  Raw addresses (this repo)
├─────────────────────────────────────────┤
│  ARM Cortex-M3 memory bus               │  Silicon
└─────────────────────────────────────────┘
```

Arduino sits at the very top and hides everything below it. Automotive
and aerospace often work at layers 2–3 for predictable code size,
deterministic timing, and MISRA / ISO 26262 certifiability.

---

### Reset Values (Why We Don't Write "Reset Code")

Every register has a **hardware-defined default state** that appears the
moment power is applied — **before a single line of your code executes**.

The reference manual specifies it per register:

```
GPIOx_CRL register
Address offset: 0x00
Reset value: 0x4444 4444
```

So `GPIOA_CRL` is *automatically* `0x44444444` at boot. No code required.

**The boot sequence:**

```
Power-on ─▶ Reset vector ─▶ Startup code ─▶ main()
             │                │                │
             │                │                └── your C runs
             │                └── copies globals, zeros .bss
             │                    (from startup_stm32f103.s)
             └── all registers snap to reset values
                 (silicon guarantee, no code required)
```

**Why hardware has default values** — chip designers pick reset values so
the chip powers up in a **safe, predictable, low-power state**:

| Register | Reset value | Rationale |
|---|---|---|
| `GPIOA_CRL` = `0x44444444` | Every pin = floating input | Safest — no shorts |
| `RCC_APB2ENR` = `0x00000000` | Every peripheral OFF | Lowest power |
| `CAN1_MCR` = `0x00010002` | Sleep mode + auto-wake | No bus disruption |

**Traps to watch out for:**

1. **Warm reset ≠ cold reset.** Software reset preserves some peripherals
   (RTC, backup domain).
2. **Bootloader / RTOS already ran.** Registers you inherit are not at
   their reset values.
3. **Mid-run reconfiguration** — obviously not at reset values anymore.

---

### Three Init Techniques: Direct Write, RMW, Full Overwrite

**Technique 1 — Direct Write (assume reset value, ignore contents)**

Fast; only safe at boot before anything has touched the register.

```c
CAN1_BTR = (0U << 24) | (0U << 20) | (5U << 16) | (8U << 0);
```

Notice `=`, not `|=`. We slam the whole 32-bit register with our value.
Safe here because BTR is only writable while CAN is in initialization
mode (which we just entered).

**Technique 2 — Read-Modify-Write (RMW) with clear-then-set**

Slower, but safe anytime.

```c
GPIOA_CRL &= ~((0xFU << 0) | (0xFU << 4));   // clear
GPIOA_CRL |=  ((0x8U << 0) | (0x8U << 4));   // set
```

Used in this repo for GPIOA_CRL because that register controls 8 pins —
a direct write would clobber the other 6.

**Technique 3 — Full-register direct write**

Direct write, but for a **complete configuration** across every field.

```c
GPIOA_CRL = (0x8U << 0) | (0x8U << 4) | (0x4U << 8) | (0x4U << 12)
          | (0x4U << 16) | (0x4U << 20) | (0x4U << 24) | (0x4U << 28);
```

One atomic write, no intermediate state, easier to audit — used in
production BSPs that own the whole peripheral.

**Rule of thumb:** if the register mixes multiple unrelated controls
(GPIO port, RCC clock gates, NVIC), use RMW. If it's single-purpose
(SysTick reload, CAN bit timing during init mode), direct write is fine.

**Special case — write-1-to-clear (W1C) status flags:**

Some status bits are set by hardware and cleared by writing a **`1`** to
them (not a `0`). The rules above don't apply — you MUST use plain `=`:

```c
CAN1_TSR = CAN_TSR_RQCP0;   // write 1 to clear the completion flag
```

The reference manual marks these bits **"rc_w1"** or **"rs_w1"**.

---

## CAN Bus Deep Dive

### Bit Timing Math

CAN nodes must agree on the exact same baud rate. The STM32 CAN peripheral
doesn't have a "set baud rate to 500000" register — you tell it how to
build a single bit out of time quanta (tq) and it computes the baud rate.

**Anatomy of one CAN bit:**

```
 |─────────── ONE CAN BIT ───────────|
 |  Sync |     TS1      |    TS2     |
 |  1 tq |    6 tq      |    1 tq    |
                        ^
                        sample point
```

- **Sync** — always 1 tq, used for edge resynchronization.
- **TS1** — from end of sync up to the sample point.
- **TS2** — from sample point to end of bit.

**Formula:**

```
BaudRate = f_APB1 / ( BRP × (1 + TS1 + TS2) )
```

**Our numbers (500 kbit/s from 36 MHz APB1):**

```
BaudRate = 36,000,000 / ( 9 × (1 + 6 + 1) )
         = 36,000,000 / ( 9 ×    8       )
         = 500,000  bits/s   ✓

Sample point = (1 + 6) / (1 + 6 + 1) = 7/9 = 78 %   (Bosch recommends 75–87.5%)
```

**Register field convention:** each field stores **`value − 1`**. So
`BRP = 9` becomes register value `8`; `TS1 = 6` becomes `5`; `TS2 = 1`
becomes `0`. Never a division-by-zero risk.

**If you change any variable in the formula, EVERY other value must
change too.** Uncommented magic numbers in a CAN driver are how firmware
ships to production with a bus that "works on the bench but glitches in
the car." Always keep the calculation in a comment above the register
write.

---

### Event-Driven vs Periodic Transmission

A body/comfort CAN bus is shared by dozens of ECUs (BCM, doors, seats,
mirrors, lighting, climate). Flooding it with 10 ms periodic frames per
switch would starve safety-critical traffic.

**Convention on comfort networks:**

1. Transmit on **state change** (this repo's approach).
2. Transmit a **heartbeat** copy every 500–1000 ms so a receiver that just
   booted knows the current state without waiting for a press.

**Safety-critical networks (powertrain, chassis)** typically transmit
periodically at 10–100 ms rates so a receiver can flag "missing frame" as
a fault within one deadline.

---

## Suggested Learning Path

Once you're comfortable with both projects:

1. **Add exercise 1 from `traffic_light.ino`** — non-blocking blink for
   the WALK countdown. Practices `millis()` without `delay()`.
2. **Replace `poll_button()` with an interrupt** using
   `attachInterrupt()`. Learn why `walk_requested` must become `volatile`.
3. **Port the traffic light to STM32** using raw registers, mirroring the
   style of `power_window_switch.c`. This forces you to write GPIO,
   SysTick, and NVIC setup from scratch.
4. **Add a receiver ECU** to the CAN project — a second STM32 that filters
   for ID 0x2A0 and drives a fake motor.
5. **Add pinch detection** — a second CAN message from the motor ECU
   carrying motor current, triggering an emergency-reverse command from
   the switch when current spikes. This is the shape of real FMVSS 118 /
   UNECE R21 compliance code.

---

## References

- **RM0008** — STM32F10xxx Reference Manual (all register/bit info)
- **DS5319** — STM32F103xx Datasheet (pinout, electrical)
- **PM0056** — STM32F1 Cortex-M3 Programming Manual (NVIC, SysTick)
- **CAN 2.0B** — Bosch specification (frame formats, arbitration)
- **STM32CubeF1** on github.com/STMicroelectronics — official HAL source
  and working examples for every peripheral
