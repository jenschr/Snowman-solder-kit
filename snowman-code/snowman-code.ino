/*
   Snowman - Red nose with cross-fading blue tummy LEDs
   Microcontroller: ATtiny412 (megaTinyCore)

   Plays a start animation, then fades smoothly between the three tummy LEDs.
   After SLEEP_TIMEOUT it goes into power-down sleep. The button wakes it.
   
   Double-clicking the button switches to LONG_SLEEP_TIMEOUT, counted from the double-click.
   A single click in that mode switches back to SLEEP_TIMEOUT, counted from the click.
   The press that wakes the snowman counts as the first click.

   Build setting:
   
   1. Tools > millis()/micros() Timer must be set to TCB0.
      The nose on PA6 uses TCD0 for PWM, which megaTinyCore uses for millis() by default.
   2. For optimum power savings, set the frequency to only 1Mhz in megaTinyCore.
   3. Be sure to set the BOD to 1.8V or 2.6V, or the device will not work when running
      off a 3V coin cell.

   See picture in repository of the settings for a visual guide.
*/

#include <avr/sleep.h>
#include <avr/interrupt.h>

#if defined(MILLIS_USE_TIMERD0)
  #error "Set Tools > millis()/micros() Timer to TCB0. The nose LED needs TCD0 for PWM."
#endif

// megaTinyCore Arduino pin numbers for the ATtiny412 port pins
#define PA6 0  // Red nose LED (TCD0 PWM, output WOA)
#define PA7 1  // Button
#define PA1 2  // Blue tummy LED 1 (TCA0 PWM)
#define PA2 3  // Blue tummy LED 2 (TCA0 PWM)
#define PA3 4  // Blue tummy LED 3 (TCA0 PWM)
#define PA0 5  // UPDI, used for programming only

#define LED1 PA6        // Red Nose
#define LED2 PA1        // Blue Tummy 1
#define LED3 PA2        // Blue Tummy 2
#define LED4 PA3        // Blue Tummy 3
#define BUTTON_PIN PA7

// LED brightness, 0 (off) to 255 (full)
const uint8_t NOSE_BRIGHTNESS = 255;
const uint8_t BLUE_BRIGHTNESS = 40;

// Time in ms for one tummy LED to fade over to the next
const unsigned long FADE_TIME = 800;

// Start animation: how long the twinkling lasts (ms), how often a frame is drawn (ms),
// and how likely a new sparkle is each frame (1 in TWINKLE_CHANCE)
const unsigned int TWINKLE_TIME = 3000;
const uint8_t TWINKLE_FRAME_TIME = 20;
const uint8_t TWINKLE_CHANCE = 12;

// Time in ms for the nose to fade in at the end of the start animation
const unsigned int START_FADE_TIME = 200;

const unsigned long SLEEP_TIMEOUT = 10000;
const unsigned long LONG_SLEEP_TIMEOUT = 60000*1;

const uint8_t tummyLEDs[3] = {LED2, LED3, LED4};

// TCD0 counts 0..NOSE_PWM_TOP, giving about 2.5 kHz PWM at 20 MHz (2 kHz at 16 MHz)
const uint8_t NOSE_PWM_TOP = 254;

// Button timing in ms
const unsigned long DEBOUNCE_TIME = 50;       // Edges this soon after the previous edge are contact bounce
const unsigned long DOUBLE_CLICK_TIME = 400;  // Longest gap between the two presses of a double-click

// The nose blinks to confirm a mode change: NOSE_BLINK_COUNT times for a double-click
// (long sleep mode) and once for a single click back to normal. Each off and each on lasts NOSE_BLINK_TIME ms.
const uint8_t NOSE_BLINK_COUNT = 2;
const unsigned int NOSE_BLINK_TIME = 100;

unsigned long fadeStartTime = 0;

// Shared with the button interrupt
volatile unsigned long lastActivityTime = 0;  // The sleep timeout counts from here
volatile unsigned long lastEdgeTime = 0;
volatile unsigned long lastPressTime = 0;
volatile bool firstClickPending = false;  // A press happened that a second press could pair with
volatile bool useLongSleep = false;       // Set by a double-click
volatile bool showDoubleClick = false;    // Set by a double-click, cleared once the nose blink starts

bool noseBlinking = false;
unsigned long noseBlinkStart = 0;
uint8_t noseBlinkCount = 0;
uint8_t noseBlinkPhase = 0xFF;

// Runs on every button edge (press and release). Ignores contact bounce,
// counts presses, and sets useLongSleep on a double-click.
void buttonISR() {
  PORTA.INTFLAGS = PIN7_bm; // Clear flag for PA7 (button)

  unsigned long now = millis();
  bool settled = (now - lastEdgeTime) >= DEBOUNCE_TIME;
  lastEdgeTime = now;

  // Only act on a clean press: the pin is low (pull-up, button to GND)
  if (!settled || digitalRead(BUTTON_PIN) == HIGH) {
    return;
  }

  if (firstClickPending && (now - lastPressTime) <= DOUBLE_CLICK_TIME) {
    useLongSleep = true;
    showDoubleClick = true;
    lastActivityTime = now;     // The long timeout counts from the double-click
    firstClickPending = false;  // A third quick press starts a new pair
  } else {
    firstClickPending = true;
  }
  lastPressTime = now;
}

// Sets TCD0 up as a PWM output on PA6 (WOA). The nose starts off.
void setupNosePWM() {
  TCD0.CTRLA = 0;                                   // TCD0 must be off while it is configured
  TCD0.CTRLB = TCD_WGMODE_ONERAMP_gc;
  TCD0.CMPBCLR = NOSE_PWM_TOP;                      // PWM period
  TCD0.CMPASET = 0x0FFF;                            // Never reached, so WOA stays low
  TCD0.CMPACLR = 0;
  _PROTECTED_WRITE(TCD0.FAULTCTRL, TCD_CMPAEN_bm);  // Let TCD0 drive PA6
  while (!(TCD0.STATUS & TCD_ENRDY_bm));
  TCD0.CTRLA = TCD_CLKSEL_20MHZ_gc | TCD_CNTPRES_DIV32_gc | TCD_ENABLE_bm;
}

// Sets the nose brightness, 0 (off) to 255 (full)
void setNoseBrightness(uint8_t value) {
  while (!(TCD0.STATUS & TCD_CMDRDY_bm));  // Wait until the previous change has been applied
  if (value == 0) {
    TCD0.CMPASET = 0x0FFF;                 // Never switch on
    TCD0.CMPACLR = 0;
  } else {
    TCD0.CMPASET = 0;                      // Switch on at the start of each cycle...
    TCD0.CMPACLR = value;                  // ...and off at `value` (255 is past the top, so always on)
  }
  TCD0.CTRLE = TCD_SYNCEOC_bm;             // Apply at the end of the current PWM cycle
}

// Sets any snowman LED to a brightness from 0 to 255
void setLED(uint8_t pin, uint8_t value) {
  if (pin == LED1) {
    setNoseBrightness(value);
  } else {
    analogWrite(pin, value);
  }
}

// Fades one LED from one brightness to another over `duration` ms
void fadeLED(uint8_t pin, uint8_t from, uint8_t to, unsigned int duration) {
  const uint8_t steps = 20;
  for (uint8_t i = 1; i <= steps; i++) {
    setLED(pin, from + ((int)to - from) * i / steps);
    delay(duration / steps);
  }
}

// Seeds random() from the noise in a few temperature sensor readings,
// so the twinkle pattern differs from one wake-up to the next
void seedRandom() {
  uint32_t seed = 0;
  for (uint8_t i = 0; i < 32; i++) {
    seed = (seed << 1) ^ analogRead(ADC_TEMPERATURE);
  }
  randomSeed(seed);
}

void doStartAnimation() {
  const uint8_t pins[4] = {LED1, LED2, LED3, LED4};
  const uint8_t maxBrightness[4] = {NOSE_BRIGHTNESS, BLUE_BRIGHTNESS, BLUE_BRIGHTNESS, BLUE_BRIGHTNESS};
  uint8_t sparkle[4] = {0, 0, 0, 0};  // How lit each LED is, 0 to 255 of its own maximum

  // Start with everything dark
  setNoseBrightness(0);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  delay(150);

  // Twinkle: random LEDs flash on and fade away quickly.
  // After TWINKLE_TIME no new sparkles start, and the last ones die out.
  const unsigned int twinkleFrames = TWINKLE_TIME / TWINKLE_FRAME_TIME;
  const unsigned int dieOutFrames = 25;
  for (unsigned int frame = 0; frame < twinkleFrames + dieOutFrames; frame++) {
    for (uint8_t i = 0; i < 4; i++) {
      sparkle[i] = sparkle[i] * 3 / 4;
    }
    if (frame < twinkleFrames && random(TWINKLE_CHANCE) == 0) {
      sparkle[random(4)] = 255;
    }
    for (uint8_t i = 0; i < 4; i++) {
      setLED(pins[i], (uint16_t)sparkle[i] * maxBrightness[i] / 255);
    }
    delay(TWINKLE_FRAME_TIME);
  }

  // The nose fades in and stays on; the cross-fade then starts on the tummy
  fadeLED(LED1, 0, NOSE_BRIGHTNESS, START_FADE_TIME);
}

// Cross-fades around the tummy LEDs: 1 -> 2 -> 3 -> 1 ...
// While one LED fades out, the next fades in, so the total brightness stays even.
void doContinuousAnimation() {
  unsigned long elapsed = millis() - fadeStartTime;
  uint8_t step = (elapsed / FADE_TIME) % 3;
  unsigned long position = elapsed % FADE_TIME;

  uint8_t fadeIn = (uint32_t)BLUE_BRIGHTNESS * position / FADE_TIME;
  uint8_t fadeOut = BLUE_BRIGHTNESS - fadeIn;

  // The start animation leaves every tummy LED off, so the very first fade
  // only fades in instead of jumping the "outgoing" LED to full brightness.
  if (elapsed < FADE_TIME) {
    fadeOut = 0;
  }

  analogWrite(tummyLEDs[step], fadeOut);
  analogWrite(tummyLEDs[(step + 1) % 3], fadeIn);
  analogWrite(tummyLEDs[(step + 2) % 3], 0);
}

// Starts blinking the nose `count` times. updateNoseBlink() does the blinking.
void startNoseBlink(uint8_t count) {
  noseBlinking = true;
  noseBlinkStart = millis();
  noseBlinkCount = count;
  noseBlinkPhase = 0xFF;
}

// Blinks the nose to confirm a mode change, without pausing the tummy cross-fade.
// A double-click during the start animation is shown once the animation has finished.
void updateNoseBlink() {
  if (showDoubleClick) {
    showDoubleClick = false;
    startNoseBlink(NOSE_BLINK_COUNT);
  }
  if (!noseBlinking) {
    return;
  }

  // Phases alternate off, on, off, on ... and the nose stays on afterwards
  uint8_t phase = (millis() - noseBlinkStart) / NOSE_BLINK_TIME;
  if (phase == noseBlinkPhase) {
    return;
  }
  noseBlinkPhase = phase;

  if (phase >= noseBlinkCount * 2) {
    setNoseBrightness(NOSE_BRIGHTNESS);
    noseBlinking = false;
  } else {
    setNoseBrightness(phase % 2 == 0 ? 0 : NOSE_BRIGHTNESS);
  }
}

// A press counts as a single click once DOUBLE_CLICK_TIME has passed without
// a second press. A single click in long sleep mode switches back to SLEEP_TIMEOUT,
// counted from the click, and blinks the nose once. In normal mode a single click does nothing.
void handleSingleClick() {
  bool leftLongSleep = false;

  noInterrupts();
  bool singleClick = firstClickPending && (millis() - lastPressTime > DOUBLE_CLICK_TIME);
  if (singleClick) {
    firstClickPending = false;
    if (useLongSleep) {
      useLongSleep = false;
      lastActivityTime = lastPressTime;
      leftLongSleep = true;
    }
  }
  interrupts();

  if (leftLongSleep) {
    startNoseBlink(1);
  }
}

void goToSleep() {
  // Turn the nose off and let the change take effect before stopping TCD0,
  // otherwise PA6 could be left high through sleep
  setNoseBrightness(0);
  while (!(TCD0.STATUS & TCD_CMDRDY_bm));
  delay(1);
  TCD0.CTRLA &= ~TCD_ENABLE_bm;

  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);

  PORTA.INTFLAGS = 0xFF;
  ADC0.CTRLA &= ~ADC_ENABLE_bm;

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  sleep_cpu();

  sleep_disable();
  ADC0.CTRLA |= ADC_ENABLE_bm;

  // Software reset, so the snowman starts again from setup() after waking
  _PROTECTED_WRITE(RSTCTRL.SWRR, 1);
}

void setup() {
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  setupNosePWM();

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // A software reset means we just woke from sleep (see goToSleep()), so the
  // button press that woke us counts as the first click of a double-click.
  // megaTinyCore copies the reset flags into GPIOR0 at startup.
  if (GPIOR0 & RSTCTRL_SWRF_bm) {
    firstClickPending = true;
    lastPressTime = millis();
    lastEdgeTime = millis();
  }

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, CHANGE);

  seedRandom();

  lastActivityTime = millis();
  doStartAnimation();
  fadeStartTime = millis();
}

void loop() {
  doContinuousAnimation();
  updateNoseBlink();
  handleSingleClick();

  // Read the values the button interrupt can change in one go, so they are consistent
  noInterrupts();
  unsigned long timerStart = lastActivityTime;
  unsigned long timeout = useLongSleep ? LONG_SLEEP_TIMEOUT : SLEEP_TIMEOUT;
  interrupts();

  if (millis() - timerStart >= timeout) {
    goToSleep();
  }
}
