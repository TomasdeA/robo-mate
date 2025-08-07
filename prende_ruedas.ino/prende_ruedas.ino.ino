// Step 1: Define PWM pins
const int PIN_PWM_L1 = 5;
const int PIN_PWM_L2 = 6;
const int PIN_PWM_R1 = 9;
const int PIN_PWM_R2 = 10;

// Step 2: Define duty cycles (0-255)
const int DUTY_L1 = 0;   // 25%
const int DUTY_L2 = 200;  // 50%
const int DUTY_R1 = 0;  // 75%
const int DUTY_R2 = 200;  // 100%

void setup() {
  // Step 3: Set PWM pins as OUTPUT
  pinMode(PIN_PWM_L1, OUTPUT);
  pinMode(PIN_PWM_L2, OUTPUT);
  pinMode(PIN_PWM_R1, OUTPUT);
  pinMode(PIN_PWM_R2, OUTPUT);
}

void loop() {
  // Step 4: Write PWM signals with different duty cycles
  analogWrite(PIN_PWM_L1, DUTY_L1);
  analogWrite(PIN_PWM_L2, DUTY_L2);
  analogWrite(PIN_PWM_R1, DUTY_R1);
  analogWrite(PIN_PWM_R2, DUTY_R2);

  // Step 5: Keep running
  delay(1000); // Optional: keep the duty cycles steady
}
