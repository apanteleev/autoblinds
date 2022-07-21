/*
Copyright (C) 2022, Alexey Panteleev.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Include the header that defines system-specific constants: WiFi network parameters and MQTT settings.
// The file is not included in git, so use 'constants.example.h' as a template.
#include "constants.h"


// Status - sends "online" or "offline" as last-will
#define MQTT_TOPIC_STATUS (MQTT_TOPIC_ROOT "status")
// Command - receives "open", "close"
#define MQTT_TOPIC_COMMAND (MQTT_TOPIC_ROOT "command")
// Status - sends "open", "closed", "opening", "closing", "stopped"
#define MQTT_TOPIC_STATE (MQTT_TOPIC_ROOT "state")
// Position - sends position percentage, 0..100
#define MQTT_TOPIC_POSITION (MQTT_TOPIC_ROOT "position")
// Set position - receives position percentage to set, 0..100
#define MQTT_TOPIC_SET_POSITION (MQTT_TOPIC_ROOT "setposition")

#define PIN_BUTTON_UP D5
#define PIN_BUTTON_DOWN D6
#define PIN_MOTOR_DIR D3
#define PIN_MOTOR_STEP D2
#define PIN_MOTOR_NEN D1
#define PIN_LED LED_BUILTIN

#define LED_INTERVAL 100 // milliseconds

// LED blinking patterns.
// Each bit represents an interval of LED_INTERVAL milliseconds, displayed right to left.
#define LED_START_STOP  0b1
#define LED_LIMIT       0b101
#define LED_SAVE        0b1110011
#define LED_STARTUP     0b1111

class LedPulser {
private:
  unsigned long lastUpdateTime = 0;
  unsigned int pattern = 0;

  void advance() {
    lastUpdateTime = millis();
    digitalWrite(PIN_LED, !(pattern & 1));
    pattern = pattern >> 1;
  }
  
public:
  void setup() {
    pinMode(PIN_LED, OUTPUT);
    advance();
  }

  void loop() {
    if (millis() >= lastUpdateTime + LED_INTERVAL) {
      advance();
    }
  }

  void show(unsigned int _pattern) {
    pattern = _pattern;
    advance();
  }
};

LedPulser led;


#define BUTTON_PRESSED 1
#define BUTTON_RELEASED -1
#define BUTTON_DEBOUNCE_DELAY 5 // milliseconds
#define BUTTON_HOLD_DURATION 500

class ButtonWithDebounce {
  // Based on the code from https://docs.arduino.cc/built-in-examples/digital/Debounce 
private:
  int buttonPin;
  bool pullup;
  
  int buttonState = LOW;
  int lastButtonState = LOW;
  int change = 0;
  unsigned long lastDebounceTime = 0;
  
public:
  ButtonWithDebounce(int pin, bool pullup)
    : buttonPin(pin)
    , pullup(pullup)
  {}

  void setup() {
    if (pullup)
      pinMode(buttonPin, INPUT_PULLUP);
    else
      pinMode(buttonPin, INPUT);
  }

  void read() {
    int reading = digitalRead(buttonPin);
    if (pullup) {
        reading = !reading;
    }

    unsigned long currentTime = millis();
    
    if (reading != lastButtonState) {
      lastDebounceTime = currentTime;
    }
  
    if (reading != buttonState && (currentTime - lastDebounceTime) > BUTTON_DEBOUNCE_DELAY) {
      change = reading - buttonState;
      buttonState = reading;
    }
    else {
      change = 0;
    }
  
    lastButtonState = reading;
  }

  int getState() { return buttonState; } // returns LOW or HIGH, after pullup correction
  int getChange() { return change; } // returns BUTTON_PRESSED or BUTTON_RELEASED

  bool pressed() { return change == BUTTON_PRESSED; }
  bool released() { return change == BUTTON_RELEASED; }
};

ButtonWithDebounce buttonUp(PIN_BUTTON_UP, true);
ButtonWithDebounce buttonDown(PIN_BUTTON_DOWN, true);


#define EEPROM_MAGIC 0x5A0D3C67 // a random number

// A struct that is directly mapped to the simulated "EEPROM".
// On ESP8266, the EEPROM is just a block of flash memory that is read into a RAM segment and written at once.
struct Settings {
  unsigned long magic;
  long position;
  long positionTop;
  long positionBottom;
  
  static Settings* load() { 
    // Read the existing data from flash
    EEPROM.begin(sizeof(Settings));

    // Cast that data to our structure
    Settings* data = (Settings*)EEPROM.getDataPtr();

    // If the data is not valid, initialize with some values and write
    if (data->magic != EEPROM_MAGIC) {
      data->magic = EEPROM_MAGIC;
      data->position = 0;
      data->positionTop = 0;
      data->positionBottom = 0;
      EEPROM.commit();
    }

    return data;
  }

  void save() {
    EEPROM.getDataPtr(); // mark the data as dirty
    EEPROM.commit();
  }

  // Tells if the current position is above the top limit (minus margin) or under the bottom limit (plus margin)
  // Also supports a targetPosition parameter that can be received via a MQTT command
  bool isOffLimits(bool up, long margin, long targetPosition) {
    if (up && positionTop != 0 && position + margin >= positionTop)
      return true;
    if (!up && positionBottom != 0 && position - margin <= positionBottom)
      return true;
      
    if (targetPosition != 0) {
      if (up && position + margin >= targetPosition)
        return true;
      if (!up && position - margin <= targetPosition)
        return true;
    }
    
    return false;
  }

  // Saves the current position as either the top or the bottom one
  void saveLimit(bool up) {
    if (up)
      positionTop = (position == 0) ? 1L : position;
    else
      positionBottom = (position == 0) ? -1L : position;

    save();
  }
};

// Motor controller FSM states
#define MSTATE_IDLE 0
#define MSTATE_ACCELERATE 1
#define MSTATE_RUN 2
#define MSTATE_DECELERATE 3
#define MSTATE_BACKSPIN 4

#define MOTOR_MIN_SPEED 200     // steps per second
#define MOTOR_MAX_SPEED 3000    // steps per second
#define MOTOR_ACCELERATION 4000 // steps per sedond per second

// Number of steps to take in the opposite direction after decelerating and before disconnecting the motor, to release tension
#define MOTOR_BACKSPIN_STEPS 800
#define MOTOR_BACKSPIN_SPEED 800

// Solution for the distance of an accelerated object traveling between two speed values
#define MOTOR_ACCELERATION_STEPS ((MOTOR_MAX_SPEED * MOTOR_MAX_SPEED - MOTOR_MIN_SPEED * MOTOR_MIN_SPEED) / (2 * MOTOR_ACCELERATION))

#define MICROSECONDS_IN_SECOND 1000000L

// DIR pin values that correspond to up or down directions
#define MOTOR_DIRECTION_UP HIGH
#define MOTOR_DIRECTION_DOWN LOW

// ESP8266 hardware timer settings
#define TIMER_DIVIDER TIM_DIV16
#define TIMER_TICKS_PER_SECOND 5000000

// Forward declarations for global functions that are used in MotorDriver but also use the driver object itself
void onTimerInterrupt();
void sendMqttState();

class MotorDriver {
private:
  int state = MSTATE_IDLE;
  long speed = 0; // in steps per second
  unsigned long accelerateStartMicros = 0;
  int direction = 0;
  int backspinStepsLeft = 0;
  bool ignoreLimits = false;
  long targetPosition = 0;
  Settings* settings = NULL;

  // Performs a full stop - for internal MotorDriver use only, use softStop() from user side instead
  void stop() {
    speed = 0;
    state = MSTATE_IDLE;
    targetPosition = 0;
    digitalWrite(PIN_MOTOR_NEN, HIGH);
    digitalWrite(PIN_MOTOR_STEP, LOW);
    
    timer1_disable();

    // Save the position in non-volatile memory in case there is a power loss
    settings->save();

    sendMqttState();
  }

  // Sets the timer to wake up when it's time to do the next step.
  // The interval is adjusted by elapsedMicroseconds to account for the time spent in processTimer before calling this function.
  void scheduleStep(uint32_t elapsedMicroseconds = 0) {
    if (speed == 0)
      return;
      
    uint32_t stepInterval = TIMER_TICKS_PER_SECOND / speed;
    uint32_t elapsedTicks = elapsedMicroseconds * (TIMER_TICKS_PER_SECOND / MICROSECONDS_IN_SECOND);
    if (stepInterval > elapsedTicks)
      stepInterval -= elapsedTicks;
    else
      stepInterval = 1;

    // Wake up once after 'stepInterval' timer ticks
    timer1_enable(TIMER_DIVIDER, TIM_EDGE, TIM_SINGLE);
    timer1_write(stepInterval);
  }

public:
  void setup(Settings* _settings) {
    settings = _settings;
  
    pinMode(PIN_MOTOR_DIR, OUTPUT);
    pinMode(PIN_MOTOR_STEP, OUTPUT);
    pinMode(PIN_MOTOR_NEN, OUTPUT);
    digitalWrite(PIN_MOTOR_DIR, LOW);
    digitalWrite(PIN_MOTOR_STEP, LOW);
    digitalWrite(PIN_MOTOR_NEN, HIGH);

    timer1_attachInterrupt(onTimerInterrupt);
  }

  void processTimer() {
    if (state == MSTATE_IDLE)
      return;
      
    unsigned long entranceMicros = micros();

    // Pulse the step pin
    digitalWrite(PIN_MOTOR_STEP, HIGH);
    delayMicroseconds(20);
    digitalWrite(PIN_MOTOR_STEP, LOW);
    
    // Update the position
    if (direction == MOTOR_DIRECTION_UP)
      ++settings->position;
    else
      --settings->position;

    // Adjust the speed if we're accelerating or decelerating
    switch(state) {
    case MSTATE_ACCELERATE:
      speed = MOTOR_MIN_SPEED + ((entranceMicros - accelerateStartMicros) * MOTOR_ACCELERATION) / MICROSECONDS_IN_SECOND;
      if (speed >= MOTOR_MAX_SPEED) {
        speed = MOTOR_MAX_SPEED;
        state = MSTATE_RUN;
      }
      break;
    case MSTATE_DECELERATE:
      speed = MOTOR_MAX_SPEED - ((entranceMicros - accelerateStartMicros) * MOTOR_ACCELERATION) / MICROSECONDS_IN_SECOND;
      if (speed <= MOTOR_MIN_SPEED) {
        backspinStepsLeft = MOTOR_BACKSPIN_STEPS;
        state = MSTATE_BACKSPIN;
        direction = !direction;
        digitalWrite(PIN_MOTOR_DIR, direction);
      }
      break;
    case MSTATE_BACKSPIN:
      speed = MOTOR_BACKSPIN_SPEED;
      if (--backspinStepsLeft <= 0)
        stop();
      break;
    }

    if (!ignoreLimits && state != MSTATE_BACKSPIN) {
      // Stop at the limits, if they are set
      if (settings->isOffLimits(direction == MOTOR_DIRECTION_UP, MOTOR_ACCELERATION_STEPS, targetPosition))
        softStop();
    }

    // Set the timer for the next step pulse
    unsigned long exitMicros = micros();
    scheduleStep(exitMicros - entranceMicros);
  }

  void start(int _direction, bool _ignoreLimits) {
    if (state != MSTATE_IDLE)
      return;
      
    direction = _direction;
    ignoreLimits = _ignoreLimits;

    if (!ignoreLimits) {
      // Don't start if we're off limits
      if (settings->isOffLimits(direction == MOTOR_DIRECTION_UP, MOTOR_ACCELERATION_STEPS, targetPosition)) {
        led.show(LED_LIMIT);
        return;
      }
    }

    // Start up the driver
    digitalWrite(PIN_MOTOR_NEN, LOW);
    digitalWrite(PIN_MOTOR_DIR, direction);
    digitalWrite(PIN_MOTOR_STEP, LOW);
    
    // Make sure that the enable and direction pins are held in the right state for at least 1us
    // before giving any step commands
    delayMicroseconds(1);
    
    state = MSTATE_ACCELERATE;
    speed = MOTOR_MIN_SPEED;

    // Initialize the accelerator
    accelerateStartMicros = micros();

    // Start the timer
    scheduleStep();
    
    led.show(LED_START_STOP);
    
    sendMqttState();
  }

  void softStop() {
    if (state == MSTATE_ACCELERATE || state == MSTATE_RUN) {
      state = MSTATE_DECELERATE;
      accelerateStartMicros = micros();
      
      led.show(LED_START_STOP);
    }
  }

  bool isRunning() {
    return state != MSTATE_IDLE;
  }

  void saveLimitPosition(int direction) {
    if (isRunning())
      return;

    led.show(LED_SAVE);
    
    settings->saveLimit(direction == MOTOR_DIRECTION_UP);
  }

  int getDirection() { return direction; }

  bool getPositionPercentage(int& percent) {
    long positionDiff = settings->positionTop - settings->positionBottom - MOTOR_BACKSPIN_STEPS * 2;
    if (settings->positionTop == 0 || settings->positionBottom == 0 || positionDiff <= 0)
      return false;

    percent = ((settings->position - settings->positionBottom - MOTOR_BACKSPIN_STEPS) * 100 + (positionDiff >> 1)) / positionDiff;
    percent = max(0, min(100, percent));
    return true;
  }

  void startTo(int percent) {
    if (isRunning())
      return;
      
    if (settings->positionTop == 0 || settings->positionBottom == 0 || settings->positionTop == settings->positionBottom)
      return;

    if (percent < 0 || percent > 100)
      return;

    targetPosition = settings->positionBottom + ((settings->positionTop - settings->positionBottom) * percent) / 100;
    if (targetPosition < settings->position) {
      if (targetPosition == 0) targetPosition = -1;
      start(MOTOR_DIRECTION_DOWN, false);
    }
    else if (targetPosition > settings->position) {
      if (targetPosition == 0) targetPosition = 1;
      start(MOTOR_DIRECTION_UP, false);
    }
  }
};

MotorDriver motor;

void onTimerInterrupt() {
  motor.processTimer();
}


#define INPUT_IDLE 0
#define INPUT_UP 1
#define INPUT_UP_FORCED 2
#define INPUT_DOWN 3
#define INPUT_DOWN_FORCED 4

class InputHandler {
private:
  int inputState = INPUT_IDLE;
  unsigned long pressTime = 0;

public:
  void setup() {
    buttonUp.setup();
    buttonDown.setup();
  }
  
  void loop() {
    buttonUp.read();
    buttonDown.read();

    // Process the button states and decide what to do:
    // - Up or down button pressed briefly: start moving up or down when the button is released, or stop if already moving
    // - Up or down button held for more than 0.5 seconds: move up or down while the button is held, ignore the top and bottom limits
    // - Up button held, down button pressed: save the current position as the top limit
    // - Down button held, up button pressed: save the current position as the bottom limit
  
    switch(inputState) {
    case INPUT_IDLE:
      if (buttonUp.pressed()) {
        inputState = INPUT_UP;
        pressTime = millis();
      } else if (buttonDown.pressed()) {
        inputState = INPUT_DOWN;
        pressTime = millis();
      }
      break;
    case INPUT_UP:
      if (buttonUp.released()) {
        if (motor.isRunning())
          motor.softStop();
        else
          motor.start(MOTOR_DIRECTION_UP, false);
        inputState = INPUT_IDLE;
      }
      else if (buttonDown.pressed()) {
        motor.saveLimitPosition(MOTOR_DIRECTION_UP);
        inputState = INPUT_IDLE;
      }
      else if (!motor.isRunning() && millis() >= pressTime + BUTTON_HOLD_DURATION) {
        motor.start(MOTOR_DIRECTION_UP, true);
        inputState = INPUT_UP_FORCED;
      }
      break;
    case INPUT_UP_FORCED:
      if (buttonUp.released()) {
        motor.softStop();
        inputState = INPUT_IDLE;
      }
      break;
    case INPUT_DOWN:
      if (buttonDown.released()) {
        if (motor.isRunning())
          motor.softStop();
        else
          motor.start(MOTOR_DIRECTION_DOWN, false);
        inputState = INPUT_IDLE;
      }
      else if (buttonUp.pressed()) {
        motor.saveLimitPosition(MOTOR_DIRECTION_DOWN);
        inputState = INPUT_IDLE;
      }
      else if (!motor.isRunning() && millis() >= pressTime + BUTTON_HOLD_DURATION) {
        motor.start(MOTOR_DIRECTION_DOWN, true);
        inputState = INPUT_DOWN_FORCED;
      }
      break;
    case INPUT_DOWN_FORCED:
      if (buttonDown.released()) {
        motor.softStop();
        inputState = INPUT_IDLE;
      }
      break;
    }
  }
};

InputHandler input;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr = topic;
  String value = "";
  for (int i = 0; i < length; i++) {
      value += (char)payload[i];
  }
  
  Serial.println("MQTT Received: ");
  Serial.println(topicStr);
  Serial.println(value);
  Serial.flush();

  if (topicStr == MQTT_TOPIC_COMMAND) {
    if (value == "open" && !motor.isRunning()) {
      motor.start(MOTOR_DIRECTION_UP, false);
    }
    else if (value == "close" && !motor.isRunning()) {
      motor.start(MOTOR_DIRECTION_DOWN, false);
    }
    else if (value == "stop" && motor.isRunning()) {
      motor.softStop();
    }
  }
  else if (topicStr == MQTT_TOPIC_SET_POSITION) {
    int position = value.toInt();
    motor.startTo(position);
  }
}

#define NSTATE_DISCONNECTED 0
#define NSTATE_WIFI_CONNECTING 1
#define NSTATE_MQTT_CONNECTING 2
#define NSTATE_MQTT_CONNECTED 3

class NetworkDriver {
private:
  int state = NSTATE_DISCONNECTED;
  unsigned long lastConnectTime = 0;
  
public:
  void loop() {
    switch(state) {
    case NSTATE_DISCONNECTED:
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      state = NSTATE_WIFI_CONNECTING;
      break;
    case NSTATE_WIFI_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        mqttClient.setServer(MQTT_BROKER, 1883);
        mqttClient.setCallback(mqttCallback);
        state = NSTATE_MQTT_CONNECTING;
        WiFi.printDiag(Serial);
        Serial.flush();
      }
      break;
    case NSTATE_MQTT_CONNECTING:
      if (WiFi.status() != WL_CONNECTED) {
        state = NSTATE_WIFI_CONNECTING;
      } else {
        unsigned long currentTime = millis();
        if (lastConnectTime == 0 || currentTime - lastConnectTime >= 10000) {
          if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD, MQTT_TOPIC_STATUS, 0, true, "offline")) {
            lastConnectTime = 0;
            state = NSTATE_MQTT_CONNECTED;
            mqttClient.publish(MQTT_TOPIC_STATUS, "online", true);
            sendMqttState();
            mqttClient.subscribe(MQTT_TOPIC_COMMAND);
            mqttClient.subscribe(MQTT_TOPIC_SET_POSITION);
            Serial.println("Connected to MQTT.");
            Serial.flush();
          }
          else {
            lastConnectTime = currentTime;
          }
        }
      }
      break;
    case NSTATE_MQTT_CONNECTED:
      if (WiFi.status() != WL_CONNECTED) {
        state = NSTATE_WIFI_CONNECTING;
      } else {
        mqttClient.loop();
        if (!mqttClient.connected()) {
          state = NSTATE_MQTT_CONNECTING;
        }
      }
    }
  }

  bool connected() {
    return state = NSTATE_MQTT_CONNECTED;
  }
};

NetworkDriver network;

void sendMqttState() {
  if (!network.connected())
    return;

  String state;
  if (motor.isRunning()) {
    if (motor.getDirection() == MOTOR_DIRECTION_UP)
      state = "opening";
    else
      state = "closing";
  }
  else {
    int percentage;
    if (motor.getPositionPercentage(percentage)) {
      if (percentage == 0)
        state = "closed";
      else if (percentage == 100)
        state = "open";
      else
        state = "stopped";
      mqttClient.publish(MQTT_TOPIC_POSITION, String(percentage).c_str());
    }
    else
      state = "stopped";
  }

  mqttClient.publish(MQTT_TOPIC_STATE, state.c_str());
}


void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Starting up...");
  Serial.flush();
  
  Settings* settings = Settings::load();
  
  led.setup();
  input.setup();
  motor.setup(settings);

  led.show(LED_STARTUP);
}

void loop() {
  input.loop();
  network.loop();
  led.loop();
  yield();
}
