#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>  
#include <Adafruit_MAX31865.h>

// 128x64 OLED Configuration (SH1106 I2C)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64      
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); 

// MAX31865 Configuration (Hardware SPI)
#define RREF      430.0
#define RNOMINAL  100.0
#define MAX_CS    10
Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX_CS);

// Hardware Pins
#define SSR_PIN    6   
#define BUTTON_PIN 2   // Input pin for the momentary button

// Boost tracking
#define HISTORY_SIZE (6)
static float tempHistory[HISTORY_SIZE] = {0};
static int historyIndex = 0;
static bool historyFilled = false;
static unsigned long lastHistoryUpdate = 0;

// --- Custom PID & Controls Setup ---
float setpoint = 98.0; // Default starting setpoint
float input = 0.0;     
float output = 0.0;    

// PID Tuning Parameters
float kp = 17.0;
float ki = 1.0;
float kd = 0.0;

// Internal PID State Registers
float integralError = 0.0;
float lastError = 0.0;
unsigned long lastPIDTime = 0;

// Averaging and Filtering Variables
float alpha = 0.25;     

// Boost Mode Logic Variables
bool isBoosting = false;
unsigned long boostStartTime = 0;

// Boost Configuration Constants
const float DROP_THRESHOLD = 0.15;       
const unsigned long BOOST_MAX_TIME = 30000; 
const float BOOST_DEACTIVATION_ZONE = 3.0; 

// Slow PWM Window Configuration for SSR
unsigned long windowStartTime;
const unsigned long windowSize = 1000; 
unsigned long lastDisplayUpdate = 0;

// --- Button Timing Variables ---
unsigned long buttonPressedTime = 0;
bool buttonWasPressed = false;
const unsigned long BUTTON_HOLD_DURATION = 500; // Required hold time in ms

void setup() {
  Serial.begin(115200);
  
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);

  // Setup button with internal pull-up resistor (Button pin connects to GND when pressed)
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Initialize SH1106 OLED
  if(!display.begin(SCREEN_ADDRESS, true)) { 
    for(;;); 
  }
  display.clearDisplay();
  display.setTextColor(SH110X_WHITE); 

  // Initialize Temperature Sensor
  thermo.begin(MAX31865_3WIRE);

  windowStartTime = millis();
  lastPIDTime = millis();
}

void loop() {
  unsigned long now = millis();

  // -------------------------------------------------------------------------
  // BUTTON ROTATION LOGIC (0.5 Second Hold Verification)
  // -------------------------------------------------------------------------
  // Using INPUT_PULLUP means LOW = pressed, HIGH = released
  bool buttonIsPressedNow = (digitalRead(BUTTON_PIN) == LOW);

  if (buttonIsPressedNow) {
    if (!buttonWasPressed) {
      // Button was just pressed down; record the start time
      buttonPressedTime = now;
      buttonWasPressed = true;
    } else {
      // Button is being held down; check if it hit the 500ms threshold
      if (now - buttonPressedTime >= BUTTON_HOLD_DURATION) {
        // Toggle the setpoint between 93 and 98
        if (setpoint == 98.0) {
          setpoint = 93.0;
        } else {
          setpoint = 98.0;
        }
        
        // Force an immediate display redraw to show the user the update
        updateDisplay();
        
        // Block further processing until they let go of the button
        while(digitalRead(BUTTON_PIN) == LOW) {
          delay(10); 
        }
        
        // Reset tracking state once released
        buttonWasPressed = false;
      }
    }
  } else {
    // Reset state if button is released early before hitting 500ms
    buttonWasPressed = false;
  }

  // 1. Read and Apply Software Moving Average Filter
  uint8_t fault = thermo.readFault();
  if (!fault) {
    float rawTemp = thermo.temperature(RNOMINAL, RREF);
    
    if (input == 0.0) {
      input = rawTemp;
    } else {
      input = (alpha * rawTemp) + ((1.0 - alpha) * input);
    }
  } else {
    thermo.clearFault();
    digitalWrite(SSR_PIN, LOW); 
    isBoosting = false;
    drawFaultScreen(fault);
    return;
  }

  // 2. Automated Boost Mode Detection Loop
  if (now - lastHistoryUpdate >= 500) {
    lastHistoryUpdate = now;
    tempHistory[historyIndex] = input; 
    historyIndex++;

    if (historyIndex >= HISTORY_SIZE) {
      historyIndex = 0;
      historyFilled = true; 
    }
  }

  // Check for drop over the historical tracking window
  if (historyFilled && !isBoosting) {
    float tempThreeSecondsAgo = tempHistory[historyIndex];
    float tempDrop = tempThreeSecondsAgo - input;
    
    if (tempDrop >= DROP_THRESHOLD && input > (setpoint - 10.0)) {
      isBoosting = true;
      boostStartTime = now;
    }
  }

  // 3. Evaluate Boost Exit Conditions
  if (isBoosting) {
    if ((now - boostStartTime >= BOOST_MAX_TIME) || (input >= (setpoint + BOOST_DEACTIVATION_ZONE))) {
      isBoosting = false;
    }
  }

  // 4. Heat Duty Calculation (Custom PID vs. Boost Overdrive)
  if (isBoosting) {
    output = windowSize * 0.5; 
  } else {
    float dt = (float)(now - lastPIDTime) / 1000.0f; 
    if (dt >= 0.1) { 
      float error = setpoint - input;
      float pTerm = kp * error;
      
      if (abs(error) < 4.0) {
          integralError += error * dt;
      } else {
          integralError = 0.0; 
      }
      float iTerm = ki * integralError;
          
      if (iTerm > 50.0) { iTerm = 50.0; integralError = 50.0 / ki; } 
      if (iTerm < 0.0) { iTerm = 0.0; integralError = 0.0; }
      
      float dTerm = kd * ((error - lastError) / dt);

      output = pTerm + iTerm + dTerm;
      if (output > (float)windowSize) output = (float)windowSize;
      if (output < 0.0) output = 0.0;
      
      lastError = error;
      lastPIDTime = now;
    }
  }

  // 5. Time-Proportional Zero-Crossing SSR Control
  if (now - windowStartTime >= windowSize) {
    windowStartTime += windowSize; 
  }

  if (output > (float)(now - windowStartTime)) {
    digitalWrite(SSR_PIN, HIGH);
  } else {
    digitalWrite(SSR_PIN, LOW);
  }

  // 6. Update Interface Screen
  if (now - lastDisplayUpdate >= 250) {
    lastDisplayUpdate = now;
    updateDisplay();
  }
}

void updateDisplay() {
  display.clearDisplay();

  // --- TOP ROW: Main Temperature Readout (Centered) ---
  display.setTextSize(3);             
  display.setCursor(14, 4);           
  display.print(input, 1);
  display.setTextSize(1);             
  display.write(247); 
  display.setTextSize(3);
  display.print("C");

  // --- MIDDLE ROW: Visual Divider Line ---
  display.drawFastHLine(0, 34, 128, SH110X_WHITE);

  // --- BOTTOM LEFT: Telemetry Data ---
  display.setTextSize(1);
  
  // Setpoint Temperature Target
  display.setCursor(0, 42); 
  display.print("SV:  ");
  display.print(setpoint, 0);
  display.write(247);
  display.print("C");

  // System Power Duty Cycle Percent
  int pctPower = (output / (float)windowSize) * 100.0f;
  display.setCursor(0, 54);
  display.print("PWR: ");
  display.print(pctPower);
  display.print("%");

  // --- BOTTOM RIGHT: Persistent Boost Banner ---
  if (isBoosting) {
    display.fillRect(68, 42, 60, 19, SH110X_WHITE);
    display.setTextColor(SH110X_BLACK); 
    display.setCursor(77, 48);
    display.print("BOOST");
    display.setTextColor(SH110X_WHITE); 
  } else {
    display.drawRect(68, 42, 60, 19, SH110X_WHITE);
    display.setCursor(77, 48);
    display.print("READY");
  }

  display.display();
}

void drawFaultScreen(uint8_t faultCode) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(24, 10); 
  display.print("CRITICAL");
  display.setCursor(12, 30);
  display.print("TEMP FAULT");
  display.setTextSize(1);
  display.setCursor(32, 52); 
  display.print("Code: 0x");
  display.print(faultCode, HEX);
  display.display();
}
