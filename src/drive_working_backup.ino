#include <ECE3.h>

uint16_t sensorValues[8];

// ---------------------------------------------------------
// 1. CALIBRATION ARRAYS
// ---------------------------------------------------------
int min_values[8] = {639, 596, 527, 733, 688, 666, 852, 689};
int max_values[8] = {1861, 1796, 1794, 1542, 1681, 1834, 1648, 1811};
int weights[8] = {-8, -4, -2, -1, 1, 2, 4, 8};

// ---------------------------------------------------------
// 2. PIN DEFINITIONS
// ---------------------------------------------------------
const int left_dir_pin = 29; 
const int left_pwm_pin = 40;
const int left_sleep_pin = 31;  

const int right_dir_pin = 30;
const int right_pwm_pin = 39;
const int right_sleep_pin = 11; 

// ---------------------------------------------------------
// 3. BASE SPEED & PID TUNING PARAMETERS
// ---------------------------------------------------------
int base_speed = 80;    
float Kp = 0.1;        // Proportional (Steering power)
// float Kd = 0.1;        // Derivative (Dampening/Jitter reduction)
// float Ki = 0.05;       // Integral (Sustained curve correction)

// Memory variables for PID and Recovery
float previous_error = 0; 
float error_integral = 0; 
float last_known_error = 0; 

void setup() {
  ECE3_Init();
  Serial.begin(9600);
  
  // Initialize Pins
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);
  
  // Wake up motor drivers
  pinMode(left_sleep_pin, OUTPUT);     
  digitalWrite(left_sleep_pin, HIGH);  
  pinMode(right_sleep_pin, OUTPUT);    
  digitalWrite(right_sleep_pin, HIGH); 
  
  // Default to forward
  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);
  
  delay(2000); 
}

void loop() {
  // =========================================================
  // BLOCK 1: IR Sensors take reading
  // =========================================================
  ECE3_read_IR(sensorValues);

  // =========================================================
  // BLOCK 2: Generate Error Term (Normalize & Weight)
  // =========================================================
  float fused_error = 0;
  float total_sensor_sum = 0;

  for (int i = 0; i < 8; i++) {
    // Subtract minimum (floor at 0)
    float zeroed_value = sensorValues[i] - min_values[i];
    if (zeroed_value < 0) { zeroed_value = 0; }

    // Normalize per sensor to 1000
    float normalized_value = (zeroed_value * 1000.0) / max_values[i];
    
    total_sensor_sum += normalized_value;

    // Apply weighting scheme
    fused_error += (normalized_value * weights[i]);
  }
  
  // Weighted average divisor
  fused_error = fused_error / 8.0; 

  int left_speed;
  int right_speed;

  // =========================================================
  // BLOCK 3: Determine Wheel Speed Corrections
  // Sub-block A: Check for specific track conditions
  // =========================================================
  
  // Condition 1: Total loss of track (Recovery)
  if (total_sensor_sum < 400) {
    if (last_known_error < 0) {
      // Line was to the left, spin left
      digitalWrite(left_dir_pin, HIGH);  
      digitalWrite(right_dir_pin, LOW);  
      analogWrite(left_pwm_pin, 60);
      analogWrite(right_pwm_pin, 60);
      return; // Skip the rest of the loop
    } else {
      // Line was to the right, spin right
      digitalWrite(right_dir_pin, HIGH); 
      digitalWrite(left_dir_pin, LOW);   
      analogWrite(left_pwm_pin, 60);
      analogWrite(right_pwm_pin, 60);
      return; 
    }
  } 
  
  //Condition 2: Solid black line / Finish Line
  else if (total_sensor_sum > 9600) {
    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);
    Serial.println("Finish Line / Cross-Tee Detected!");
    while(true) {} // Freeze
  }

  // Update memory if we are safely on the track
  last_known_error = fused_error;

  // =========================================================
  // BLOCK 3: Determine Wheel Speed Corrections
  // Sub-block B: Compute steering changes with PID
  // =========================================================
  
  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);
  
  // 1. Proportional (P)
  float P_term = fused_error * Kp;
  
  // 2. Integral (I) - Accumulates error over time
  error_integral += fused_error;
  //float I_term = error_integral * Ki;
  
  // 3. Derivative (D) - Looks at the rate of change
  float derivative = fused_error - previous_error;
  //float D_term = derivative * Kd;
  
  // Total Steering Correction
  float steering_correction = P_term;
  
  // Save current error for the next loop's derivative math
  previous_error = fused_error;

  // =========================================================
  // BLOCK 4: Combine with Base Speed
  // =========================================================
  
  left_speed = base_speed - steering_correction;
  right_speed = base_speed + steering_correction;
  
  // Clamp speeds to valid PWM range (0-255)
  left_speed = constrain(left_speed, 0, 255);
  right_speed = constrain(right_speed, 0, 255);

  // Send to motors (Car Movement on Track)
  analogWrite(left_pwm_pin, left_speed);
  analogWrite(right_pwm_pin, right_speed);
}
