#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MAX31865.h>

// 128x32 OLED Configuration (I2C)
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// MAX31865 Configuration (Hardware SPI)
#define RREF      430.0
#define RNOMINAL  100.0
#define MAX_CS    10
Adafruit_MAX31865 thermo = Adafruit_MAX31865(MAX_CS);

// Hardware Pins
#define SSR_PIN   6   // Digital pin controlling the zero-crossing SSR

// Boost tracking
#define HISTORY_SIZE (6)
static float tempHistory[HISTORY_SIZE] = {0};
static int historyIndex = 0;
static bool historyFilled = false;
static unsigned long lastHistoryUpdate = 0;

// --- Custom PID & Controls Setup ---
float setpoint = 93.0; 
float input = 0.0;     // Smoothed temperature
float output = 0.0;    // SSR Window Duty Time (0 to 1000ms)

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
const unsigned long BOOST_MAX_TIME = 22000; 
const float BOOST_DEACTIVATION_ZONE = 3.0; 

// Slow PWM Window Configuration for SSR
unsigned long windowStartTime;
const unsigned long windowSize = 1000; 
unsigned long lastDisplayUpdate = 0;

void setup() {
  Serial.begin(115200);
  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);

  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    for(;;); 
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Initialize Temperature Sensor
  thermo.begin(MAX31865_3WIRE);

  windowStartTime = millis();
  lastPIDTime = millis();
}

void loop() {
  unsigned long now = millis();

  // 1. Read and Apply Software Moving Average Filter
  uint8_t fault = thermo.readFault();
  if (!fault) {
    float rawTemp = thermo.temperature(RNOMINAL, RREF);
    
    // Initialize filter on first boot, otherwise smooth data
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
    tempHistory[historyIndex] = input; // input is current temp
    historyIndex++;

    if (historyIndex >= HISTORY_SIZE) {
      historyIndex = 0;
      historyFilled = true; // We now have 3 secs of data
    }
  }

  // Check for the 1.0 Deg C drop over the 3 sec window
  if (historyFilled && !isBoosting) {
    float tempThreeSecondsAgo = tempHistory[historyIndex];
    float tempDrop = tempThreeSecondsAgo - input;
    
    // If temperature drops rapidly during shot extraction
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
    output = windowSize * 0.3; // Force 80% full ON duty cycle (1000ms)
  } else {
    // Custom Time-Delta PID Calculation Loop
    float dt = (float)(now - lastPIDTime) / 1000.0f; // Calculate delta time in seconds
    if (dt >= 0.1) { // Compute every 100ms
      float error = setpoint - input;
      
      // Proportional term
      float pTerm = kp * error;
      
      // Integral term with an Integral Window (Only runs within 3°C of target)
      if (abs(error) < 3.0) {
          integralError += error * dt;
          
      } else {
          // Keep integral cleared while heating up to completely eliminate windup
          integralError = 0.0; 
      }
      float iTerm = ki * integralError;
          
      // Safe Clamping: Allow it to drop to 0.0 so overshoot stops!
      if (iTerm > 30.0) { iTerm = 30.0; integralError = 30.0 / ki; } 
      if (iTerm < 0.0) { iTerm = 0.0; integralError = 0.0; }
      
      // Derivative term
      float dTerm = kd * ((error - lastError) / dt);

      // Combine terms and constrain to our 0-1000ms window limits
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

  // 6. Update 128x32 Interface Screen
  if (now - lastDisplayUpdate >= 250) {
    lastDisplayUpdate = now;
    updateDisplay();
  }
}

void updateDisplay() {
  display.clearDisplay();

  // --- LEFT COLUMN: Filtered Temperature ---
  display.setTextSize(2);
  display.setCursor(0, 14);
  display.print(input, 1);
  display.setTextSize(1);
  display.write(247); 
  display.setTextSize(2);
  display.print("C");

  // --- RIGHT COLUMN: Machine Status & Modes ---
  display.setTextSize(1);
  
  // Setpoint Temperature Target
  display.setCursor(76, 8);
  display.print("SV:");
  display.print(setpoint, 0);
  display.write(247);

  // Status Indicator (Flashing "BOOST" if active, otherwise showing standard PID Power %)
  display.setCursor(76, 24);
  if (isBoosting) {
    if ((millis() / 250) % 2 == 0) {
      display.print("->BOOST<-");
    } else {
      display.print("  BOOST  ");
    }
  } else {
    int pctPower = (output / (float)windowSize) * 100.0f;
    display.print("PWR:");
    display.print(pctPower);
    display.print("%");
  }

  display.display();
}

void drawFaultScreen(uint8_t faultCode) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.print("ERR FLT");
  display.setTextSize(1);
  display.setCursor(0, 20);
  display.print("Code: 0x");
  display.print(faultCode, HEX);
  display.display();
}
