/*
  Pedestrian Traffic Light Controller
  -----------------------------------
  Teaches core embedded C patterns:
    - Fixed-width integer types (uint8_t / uint32_t)
    - #define for hardware configuration
    - Finite State Machine (FSM) using an enum
    - NON-BLOCKING timing with millis() (never use delay() in real firmware!)
    - Software debouncing of a mechanical push button
    - const for read-only lookup data (saves RAM on small MCUs)

  Hardware:
    RED    LED -> pin 8   (with 220 ohm series resistor to GND)
    YELLOW LED -> pin 9
    GREEN  LED -> pin 10
    WALK   LED -> pin 11
    BUTTON     -> pin 2  (other side to GND; uses internal pull-up)
*/

/* ---------- 1. Hardware configuration (compile-time constants) ---------- */
/* #define is a preprocessor macro -- it costs zero RAM and zero flash. */
#define PIN_RED     8
#define PIN_YELLOW  9
#define PIN_GREEN   10
#define PIN_WALK    11
#define PIN_BUTTON  2

/* Timing constants in milliseconds. uint32_t matches what millis() returns. */
#define T_GREEN_MS     5000UL   /* GREEN stays on at least this long           */
#define T_YELLOW_MS    2000UL   /* YELLOW transition                           */
#define T_WALK_MS      4000UL   /* Pedestrian WALK signal                      */
#define DEBOUNCE_MS    30UL     /* Ignore button changes faster than this      */

/* ---------- 2. The state machine ---------- */
/* An enum gives readable names to the states -- much better than magic
   numbers like `state = 2;`. The compiler picks the smallest int that fits. */
typedef enum {
  STATE_GREEN,
  STATE_YELLOW,
  STATE_RED_WALK,
  STATE_RED_HOLD
} traffic_state_t;

/* Current state of the FSM. uint8_t is plenty for a handful of states. */
static traffic_state_t state = STATE_GREEN;

/* Timestamp when we last entered the current state.
   uint32_t because millis() wraps around after ~49 days as a uint32_t. */
static uint32_t state_entered_ms = 0;

/* Pedestrian request latch: set when the button is pressed, cleared
   when we actually service the WALK phase. */
static bool walk_requested = false;

/* ---------- 3. Debounce bookkeeping ---------- */
static uint8_t  button_stable   = HIGH;   /* pull-up makes idle = HIGH   */
static uint8_t  button_last_raw = HIGH;
static uint32_t button_last_change_ms = 0;

/* ---------- 4. Setup: runs once at power-on ---------- */
void setup() {
  pinMode(PIN_RED,    OUTPUT);
  pinMode(PIN_YELLOW, OUTPUT);
  pinMode(PIN_GREEN,  OUTPUT);
  pinMode(PIN_WALK,   OUTPUT);

  /* INPUT_PULLUP enables the AVR's internal ~20k pull-up resistor so
     we don't need an external one. Button reads LOW when pressed. */
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println(F("Traffic light booted."));  /* F() keeps string in flash */

  enter_state(STATE_GREEN);
}

/* ---------- 5. Helper: drive the four LEDs from one call ---------- */
/* `const` in the parameter list is a promise: this function will not
   modify the caller's data. It also lets us pass ROM tables safely. */
static void set_lights(uint8_t red, uint8_t yellow, uint8_t green, uint8_t walk) {
  digitalWrite(PIN_RED,    red);
  digitalWrite(PIN_YELLOW, yellow);
  digitalWrite(PIN_GREEN,  green);
  digitalWrite(PIN_WALK,   walk);
}

/* ---------- 6. State entry: called ONCE when we transition ---------- */
static void enter_state(traffic_state_t next) {
  state = next;
  state_entered_ms = millis();

  switch (state) {
    case STATE_GREEN:
      set_lights(LOW,  LOW,  HIGH, LOW);
      Serial.println(F("-> GREEN"));
      break;
    case STATE_YELLOW:
      set_lights(LOW,  HIGH, LOW,  LOW);
      Serial.println(F("-> YELLOW"));
      break;
    case STATE_RED_WALK:
      set_lights(HIGH, LOW,  LOW,  HIGH);
      Serial.println(F("-> RED + WALK"));
      break;
    case STATE_RED_HOLD:
      set_lights(HIGH, LOW,  LOW,  LOW);
      Serial.println(F("-> RED (hold, walk over)"));
      break;
  }
}

/* ---------- 7. Debounce: called every loop iteration ---------- */
/* WHY DEBOUNCE?  A mechanical switch's contacts physically bounce for
   a few ms when pressed -- one press can look like 5-10 presses to the
   CPU. We only accept a new "stable" reading after it holds for DEBOUNCE_MS. */
static void poll_button(uint32_t now_ms) {
  uint8_t raw = digitalRead(PIN_BUTTON);

  if (raw != button_last_raw) {
    button_last_raw = raw;
    button_last_change_ms = now_ms;   /* signal is still bouncing */
    return;
  }

  if ((now_ms - button_last_change_ms) < DEBOUNCE_MS) {
    return;  /* not stable long enough yet */
  }

  /* Detect a HIGH->LOW edge on the debounced signal = press event. */
  if (raw != button_stable) {
    button_stable = raw;
    if (raw == LOW) {
      walk_requested = true;
      Serial.println(F("Button pressed (walk requested)"));
    }
  }
}

/* ---------- 8. Main loop: runs forever, must NEVER block ---------- */
/* The golden rule of embedded firmware: loop() finishes fast and comes
   right back. No delay(), no while-waiting. That way we can service the
   button, the lights, and future features (buzzer, LCD, radio) all at
   once with a single CPU. */
void loop() {
  uint32_t now = millis();
  uint32_t elapsed = now - state_entered_ms;

  poll_button(now);

  switch (state) {
    case STATE_GREEN:
      /* Leave GREEN only if someone asked to walk AND minimum time passed. */
      if (walk_requested && elapsed >= T_GREEN_MS) {
        enter_state(STATE_YELLOW);
      }
      break;

    case STATE_YELLOW:
      if (elapsed >= T_YELLOW_MS) {
        walk_requested = false;         /* consume the request */
        enter_state(STATE_RED_WALK);
      }
      break;

    case STATE_RED_WALK:
      if (elapsed >= T_WALK_MS) {
        enter_state(STATE_RED_HOLD);
      }
      break;

    case STATE_RED_HOLD:
      /* Short pause with WALK off before going back to GREEN. */
      if (elapsed >= 1000UL) {
        enter_state(STATE_GREEN);
      }
      break;
  }
}

/*
  ===================== LEARNING EXERCISES =====================
  Try these in order -- each one teaches a new embedded concept.

  1. Add a "walk countdown": blink the WALK LED at 2 Hz during the
     last 1500 ms of STATE_RED_WALK. (Practice: non-blocking blink
     using millis(), no delay!)

  2. Replace poll_button() with a hardware INTERRUPT using
     attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), isr, FALLING).
     You'll need to mark `walk_requested` as `volatile` -- learn why.

  3. Add a piezo buzzer that chirps once per second while WALK is lit.

  4. Log state changes with millis() timestamps over Serial and graph
     the timing in a spreadsheet -- verify your FSM is on schedule.

  5. Refactor the lights into a `const uint8_t patterns[4][4] PROGMEM`
     table indexed by state, so enter_state() just looks up the pattern.
     This is how production firmware stays small and fast.
  ===============================================================
*/
