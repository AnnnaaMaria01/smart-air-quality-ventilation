#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <HardwareSerial.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// DHT11 sensor
#define DHTPIN 26
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// MQ-135 sensor
#define MQ135_PIN 34

// MH-Z19C CO2 sensor
HardwareSerial co2Serial(2);
byte cmd[9] = {0xFF, 0x01, 0x86, 0, 0, 0, 0, 0, 0x79};
byte response[9];

// Fan and L298N driver
#define FAN_ENA 25
#define FAN_IN1 32
#define FAN_IN2 33

// PWM setup using the ESP32 LEDC peripheral
const int PWM_FREQ = 1000;     // PWM frequency: 1 kHz
const int PWM_RES_BITS = 8;    // 8-bit resolution: values from 0 to 255
const int PWM_MAX = 255;

// PID controller
// CO2 setpoint used by the controller
double setpointCO2 = 1000.0;   // ppm
double co2Min = 1000.0;        // Lower limit defined for the control range
double co2Max = 2000.0;        // Upper limit defined for the control range

// PID gains were calculated from the identified model 
// and then adjusted during testing on the real setup
double Kp = 0.10;
double Ki = 0.002;
double Kd = 0.02;

double integral = 0.0;
double previousError = 0.0;
unsigned long lastPidTime = 0;

// Stores the output calculated during the previous step.
// The anti-windup logic uses it to detect
// when the controller output is already saturated
double lastOutput = 0.0;

// Individual PID terms are stored for
// serial logging and later analysis.
double lastPTerm = 0.0;
double lastITerm = 0.0;
double lastDTerm = 0.0;

// Limits applied to the integral contribution.
// They prevent excessive integral build-up.
const double INTEGRAL_MAX = 60.0;
const double INTEGRAL_MIN = 0.0;

// Sensor and actuator functions

// Reads the CO2 concentration from the MH-Z19C over UART
int readCO2() {
  // Clear any leftover bytes before starting a new reading
  while (co2Serial.available()) co2Serial.read();

  // Send the standard read command to the sensor
  co2Serial.write(cmd, 9);
  delay(200);

  if (co2Serial.available() >= 9) {
    co2Serial.readBytes(response, 9);
    // Check the MH-Z19C response header
    if (response[0] == 0xFF && response[1] == 0x86) {
      return response[2] * 256 + response[3];
    }
  }

  // A return value of 0 means no valid response was received.
  return 0;
}

// Minimum PWM level required to start the fan
// Below this value, the motor does not produce enough starting torque
const double FAN_MIN_PWM_PERCENT = 24.0;

// Applies the controller output to the fan as a percentage
// To compensate for the dead zone, non-zero commands in (0, 100]
// are remapped to [FAN_MIN_PWM_PERCENT, 100]
void setFanPercent(double percent) {
  percent = constrain(percent, 0.0, 100.0);

  double pwmPercent;
  if (percent <= 0.0) {
    pwmPercent = 0.0;
  } else {
    pwmPercent = FAN_MIN_PWM_PERCENT
                  + (percent / 100.0) * (100.0 - FAN_MIN_PWM_PERCENT);
  }

  // Convert the percentage command to an 8-bit PWM value
  int pwmValue = (int)round(pwmPercent * PWM_MAX / 100.0);

  // The fan is driven in one direction only
  digitalWrite(FAN_IN1, HIGH);
  digitalWrite(FAN_IN2, LOW);

  ledcWrite(FAN_ENA, pwmValue);
}

// PID calculation

// Calculates the fan command in the 0-100% range
// from the difference between the measured CO2 level and the setpoint
double computePID(double co2ppm, double dt) {
  double error = co2ppm - setpointCO2; // Positive error when CO2 is above the setpoint

  // Proportional term
  double pTerm = Kp * error;

  // The integral term uses two anti-windup measures:
  // 1. stop integration when the output is saturated and the error
  //    would push it further in the same direction;
  // 2. clamp the integral contribution directly
  bool saturatedHigh = (lastOutput >= 100.0 && error > 0);
  bool saturatedLow  = (lastOutput <= 0.0 && error < 0);

  if (!saturatedHigh && !saturatedLow) {
    integral += error * dt;
  }

  // Clamp the integral contribution to the allowed range
  double iTerm = Ki * integral;
  if (iTerm > INTEGRAL_MAX) {
    iTerm = INTEGRAL_MAX;
    integral = iTerm / Ki;
  } else if (iTerm < INTEGRAL_MIN) {
    iTerm = INTEGRAL_MIN;
    integral = iTerm / Ki;
  }

  // Derivative term calculated using a finite difference
  double derivative = (dt > 0) ? (error - previousError) / dt : 0.0;
  double dTerm = Kd * derivative;

  previousError = error;

  // Total controller output
  double output = pTerm + iTerm + dTerm;

  // Limit the output to the actuator range
  output = constrain(output, 0.0, 100.0);

  // Store the values for the next loop and for logging
  lastOutput = output;
  lastPTerm = pTerm;
  lastITerm = iTerm;
  lastDTerm = dTerm;

  return output;
}

// System setup

void setup() {
  // Start Serial communication for monitoring and logging
  Serial.begin(115200);

  // Start I2C and initialize the OLED display
  Wire.begin(21, 22);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Initialize the temperature and humidity sensor
  dht.begin();

  // Start UART communication with the MH-Z19C sensor
  co2Serial.begin(9600, SERIAL_8N1, 16, 17);

  // Configure the pins used by the L298N driver
  pinMode(FAN_IN1, OUTPUT);
  pinMode(FAN_IN2, OUTPUT);
  pinMode(FAN_ENA, OUTPUT);

  // Attach the ENA pin to the ESP32 PWM peripheral
  ledcAttach(FAN_ENA, PWM_FREQ, PWM_RES_BITS);

  lastPidTime = millis();

  // CSV header used when saving experimental data
  Serial.println("time_ms,co2_ppm,setpoint_ppm,fan_percent,p_term,i_term,d_term,temp_c,hum_pct,airq_raw");
}

// Main loop

void loop() {
  // Calculate the time elapsed since the previous loop
  unsigned long now = millis();
  double dt = (now - lastPidTime) / 1000.0; // seconds
  if (dt <= 0) dt = 0.001;
  lastPidTime = now;

  // Read all sensor values
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();
  int airQuality = analogRead(MQ135_PIN);
  int co2ppm = readCO2();

  // Calculate the PID output and apply it to the fan
  double fanPercent = computePID((double)co2ppm, dt);
  setFanPercent(fanPercent);

  // Update OLED display
  display.clearDisplay();
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("Temp: ");
  display.print(temp);
  display.print(" C");

  display.setCursor(0, 10);
  display.print("Hum : ");
  display.print(hum);
  display.print(" %");

  display.setCursor(0, 20);
  display.print("CO2 : ");
  display.print(co2ppm);
  display.print(" ppm");

  display.setCursor(0, 30);
  display.print("AirQ: ");
  display.print(airQuality);

  display.setCursor(0, 40);
  display.print("Fan : ");
  display.print(fanPercent, 1);
  display.print(" %");

  display.setCursor(0, 50);
  display.print("Set : ");
  display.print(setpointCO2, 0);
  display.print(" ppm");

  display.display();

  // Send data in CSV format
  Serial.print(now);
  Serial.print(",");
  Serial.print(co2ppm);
  Serial.print(",");
  Serial.print(setpointCO2, 0);
  Serial.print(",");
  Serial.print(fanPercent, 2);
  Serial.print(",");
  Serial.print(lastPTerm, 2);
  Serial.print(",");
  Serial.print(lastITerm, 2);
  Serial.print(",");
  Serial.print(lastDTerm, 2);
  Serial.print(",");
  Serial.print(temp);
  Serial.print(",");
  Serial.print(hum);
  Serial.print(",");
  Serial.println(airQuality);

  // Approximate sampling period of the control loop
  delay(1000); // ~1 s sampling interval used by the PID controller
}