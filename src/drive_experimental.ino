#include <ECE3.h>

uint16_t sensorValues[8];

// ---------------------------------------------------------
// 1. CALIBRATION ARRAYS
// ---------------------------------------------------------
int min_values[8] = {639, 596, 527, 733, 688, 666, 852, 689};
int max_values[8] = {1861, 1796, 1794, 1542, 1681, 1834, 1648, 1811};
int weights[8] = {-16, -8, -4, -1, 1, 4, 8, 16};  // doubled outer weights

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
// 3. SPEED & PD TUNING
// ---------------------------------------------------------
int base_speed = 160;
float Kp = 0.13;
float Kd = 1.30;

float previous_error = 0; 
float last_known_error = 0; 

// ---------------------------------------------------------
// 4. BLACK LINE DETECTION
// ---------------------------------------------------------
int finish_counter = 0;
const int finish_confirm_needed = 3;
const int FINISH_THRESHOLD = 2200;

// ---------------------------------------------------------
// 5. STATE TRACKING
// ---------------------------------------------------------
bool hasReachedEnd = false;
bool inCooldown = false;
unsigned long cooldownStart = 0;
const unsigned long COOLDOWN_MS = 4000;

const int UTURN_SPEED = 120;
const int UTURN_DURATION_MS = 500;  // adjusted for speed 140

// ---------------------------------------------------------
// HELPER FUNCTIONS
// ---------------------------------------------------------
void stopMotors() {
  analogWrite(left_pwm_pin, 0);
  analogWrite(right_pwm_pin, 0);
}

bool finishLineDetected() {
  int left_votes = 0;
  int right_votes = 0;

  if (sensorValues[0] > FINISH_THRESHOLD) left_votes++;
  if (sensorValues[1] > FINISH_THRESHOLD) left_votes++;
  if (sensorValues[2] > FINISH_THRESHOLD) left_votes++;
  if (sensorValues[3] > FINISH_THRESHOLD) left_votes++;

  if (sensorValues[4] > FINISH_THRESHOLD) right_votes++;
  if (sensorValues[5] > FINISH_THRESHOLD) right_votes++;
  if (sensorValues[6] > FINISH_THRESHOLD) right_votes++;
  if (sensorValues[7] > FINISH_THRESHOLD) right_votes++;

  return (left_votes >= 2 && right_votes >= 2);
}

void performUTurn() {
  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, HIGH);
  analogWrite(left_pwm_pin, UTURN_SPEED);
  analogWrite(right_pwm_pin, UTURN_SPEED);

  delay(UTURN_DURATION_MS);

  stopMotors();
  delay(25);
  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);
}

// ---------------------------------------------------------
// SETUP
// ---------------------------------------------------------
void setup() {
  ECE3_Init();
  Serial.begin(9600);
  
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);
  
  pinMode(left_sleep_pin, OUTPUT);     
  digitalWrite(left_sleep_pin, HIGH);  
  pinMode(right_sleep_pin, OUTPUT);    
  digitalWrite(right_sleep_pin, HIGH); 
  
  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);
  
  delay(2000); 
}

// ---------------------------------------------------------
// MAIN LOOP
// ---------------------------------------------------------
void loop() {
  ECE3_read_IR(sensorValues);

  // -------------------------------------------------------
  // BLOCK 1: Cooldown timer check
  // -------------------------------------------------------
  if (inCooldown && (millis() - cooldownStart > COOLDOWN_MS)) {
    inCooldown = false;
  }

  // -------------------------------------------------------
  // BLOCK 2: Black line detection & state handling
  // -------------------------------------------------------
  if (!inCooldown && finishLineDetected()) {
    finish_counter++;
  } else {
    finish_counter = 0;
  }

  if (finish_counter >= finish_confirm_needed) {
    finish_counter = 0;

    if (!hasReachedEnd) {
      hasReachedEnd = true;
      stopMotors();
      delay(25);
      performUTurn();
      inCooldown = true;
      cooldownStart = millis();
    } else {
      stopMotors();
      while (true) {}
    }
  }

  // -------------------------------------------------------
  // BLOCK 3: Normalize sensors
  // -------------------------------------------------------
  float fused_error = 0;
  float total_sensor_sum = 0;
  float normalized[8]; 

  for (int i = 0; i < 8; i++) {
    float zeroed_value = sensorValues[i] - min_values[i];
    if (zeroed_value < 0) zeroed_value = 0;
    float normalized_value = (zeroed_value * 1000.0) / max_values[i];
    normalized_value = constrain(normalized_value, 0, 1000);
    normalized[i] = normalized_value;
    total_sensor_sum += normalized_value;
    fused_error += normalized_value * weights[i];
  }
  
  fused_error = fused_error / 8.0;

  // -------------------------------------------------------
  // BLOCK 4: Recovery if track is lost
  // -------------------------------------------------------
  if (total_sensor_sum < 400) {
    if (last_known_error < 0) {
      digitalWrite(left_dir_pin, HIGH);  
      digitalWrite(right_dir_pin, LOW);  
    } else {
      digitalWrite(right_dir_pin, HIGH); 
      digitalWrite(left_dir_pin, LOW);   
    }
    analogWrite(left_pwm_pin, 60);
    analogWrite(right_pwm_pin, 60);
    return;
  }

  last_known_error = fused_error;

  // -------------------------------------------------------
  // BLOCK 5: PD steering
  // -------------------------------------------------------
  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);
  
  float P_term = fused_error * Kp;
  float D_term = (fused_error - previous_error) * Kd;
  float steering_correction = P_term + D_term;
  previous_error = fused_error;

  int left_speed  = constrain(base_speed - steering_correction, 0, 255);
  int right_speed = constrain(base_speed + steering_correction, 0, 255);

  analogWrite(left_pwm_pin, left_speed);
  analogWrite(right_pwm_pin, right_speed);
}
