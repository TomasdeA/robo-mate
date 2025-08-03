
//MCUs
#define NANO 328 //Atmega328p
#define ESP32 32 //ESP32

//Robot bodies with Atmega328p as MCU --> Select Arduino Nano as board
#define DIY 0        // DIY without PCB
#define PCB_V1 1     // DIY with PCB V1
#define PCB_V2 2     // DIY with PCB V2
#define RTR_TT 3     // Ready-to-Run with TT-motors
#define RC_CAR 4     // RC truck prototypes
#define LITE 5       // Smaller DIY version for education
//Robot bodies with ESP32 as MCU --> Select ESP32 Dev Module as board
#define RTR_TT2 6   // Ready-to-Run with TT-motors
#define RTR_520 7    // Ready-to-Run with 520-motors
#define MTV 8        // Multi Terrain Vehicle
#define DIY_ESP32 9  // DIY without PCB

//------------------------------------------------------//
// SETUP - Choose your body
//------------------------------------------------------//

// Setup the OpenBot version (DIY, PCB_V1, PCB_V2, RTR_TT, RC_CAR, LITE, RTR_TT2, RTR_520, DIY_ESP32)
#define OPENBOT DIY


#define NO_PHONE_MODE 1
#define DEBUG 1

// Enable/Disable coast mode (1,0)
// When no control is applied, the robot will either coast (1) or actively stop (0)
boolean coast_mode = 1;

//-------------------------DIY--------------------------//
#if (OPENBOT == DIY)
const String robot_type = "DIY";
#define MCU NANO
#define HAS_VOLTAGE_DIVIDER 0
const float VOLTAGE_DIVIDER_FACTOR = (20 + 10) / 10;
const float VOLTAGE_MIN = 2.5f;
const float VOLTAGE_LOW = 9.0f;
const float VOLTAGE_MAX = 12.6f;
const float ADC_FACTOR = 5.0 / 1023;
#define HAS_INDICATORS 0
#define HAS_SONAR 0
#define SONAR_MEDIAN 0
#define HAS_SPEED_SENSORS_FRONT 0
#define HAS_OLED 0
const int PIN_PWM_L1 = 5;
const int PIN_PWM_L2 = 6;
const int PIN_PWM_R1 = 9;
const int PIN_PWM_R2 = 10;
const int PIN_SPEED_LF = 2;
const int PIN_SPEED_RF = 3;
const int PIN_VIN = A7;
const int PIN_TRIGGER = 12;
const int PIN_ECHO = 11;
const int PIN_LED_LI = 4;
const int PIN_LED_RI = 7;

//-------------------------RTR_TT-----------------------//
#elif (OPENBOT == RTR_TT)
const String robot_type = "RTR_TT";
#define MCU NANO
#define HAS_VOLTAGE_DIVIDER 1
const float VOLTAGE_DIVIDER_FACTOR = (30 + 10) / 10;
const float VOLTAGE_MIN = 2.5f;
const float VOLTAGE_LOW = 9.0f;
const float VOLTAGE_MAX = 12.6f;
const float ADC_FACTOR = 5.0 / 1023;
#define HAS_INDICATORS 1
#define HAS_SONAR 1
#define SONAR_MEDIAN 0
#define HAS_BUMPER 1
#define HAS_SPEED_SENSORS_FRONT 1
#define HAS_SPEED_SENSORS_BACK 1
#define HAS_LEDS_FRONT 1
#define HAS_LEDS_BACK 1
#define HAS_LEDS_STATUS 1
const int PIN_PWM_L1 = 10;
const int PIN_PWM_L2 = 9;
const int PIN_PWM_R1 = 6;
const int PIN_PWM_R2 = 5;
const int PIN_SPEED_LF = A3;
const int PIN_SPEED_RF = 7;
const int PIN_SPEED_LB = A4;
const int PIN_SPEED_RB = 8;
const int PIN_VIN = A6;
const int PIN_TRIGGER = 4;
const int PIN_ECHO = 2;
const int PIN_LED_LI = A5;
const int PIN_LED_RI = 12;
const int PIN_LED_LB = A5;
const int PIN_LED_RB = 12;
const int PIN_LED_LF = 3;
const int PIN_LED_RF = 11;
const int PIN_LED_Y = 13;
const int PIN_LED_G = A0;
const int PIN_LED_B = A1;
const int PIN_BUMPER = A2;
const int BUMPER_NOISE = 512;
const int BUMPER_EPS = 10;
const int BUMPER_AF = 951;
const int BUMPER_BF = 903;
const int BUMPER_CF = 867;
const int BUMPER_LF = 825;
const int BUMPER_RF = 786;
const int BUMPER_BB = 745;
const int BUMPER_LB = 607;
const int BUMPER_RB = 561;

//-------------------------RC_CAR-----------------------//
#elif (OPENBOT == RC_CAR)
const String robot_type = "RC_CAR";
#define MCU NANO
#include <Servo.h>
Servo ESC;
Servo SERVO;
#define HAS_VOLTAGE_DIVIDER 0
const float VOLTAGE_DIVIDER_FACTOR = (20 + 10) / 10;
const float VOLTAGE_MIN = 0.0f;
const float VOLTAGE_LOW = 6.4f;
const float VOLTAGE_MAX = 8.4f;
const float ADC_FACTOR = 5.0 / 1023;
#define HAS_INDICATORS 0
#define HAS_SONAR 0
#define SONAR_MEDIAN 0
const int PIN_PWM_T = A0;
const int PIN_PWM_S = A1;
const int PIN_VIN = A7;
const int PIN_TRIGGER = 4;
const int PIN_ECHO = 4;
const int PIN_LED_LI = 7;
const int PIN_LED_RI = 8;
#endif
//------------------------------------------------------//

#if (HAS_BLUETOOTH)
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

BLEServer *bleServer = NULL;
BLECharacteristic *pTxCharacteristic;
BLECharacteristic *pRxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
const char *SERVICE_UUID = "61653dc3-4021-4d1e-ba83-8b4eec61d613";  // UART service UUID
const char *CHARACTERISTIC_UUID_RX = "06386c14-86ea-4d71-811c-48f97c58f8c9";
const char *CHARACTERISTIC_UUID_TX = "9bf1103b-834c-47cf-b149-c9e4bcf778a7";
#endif

enum msgParts {
  HEADER,
  BODY
};

msgParts msgPart = HEADER;
char header;
char endChar = '\n';
const char MAX_MSG_SZ = 60;
char msg_buf[MAX_MSG_SZ] = "";
int msg_idx = 0;

#if (HAS_BLUETOOTH)
void on_ble_rx(char inChar) {
  if (inChar != endChar) {
    switch (msgPart) {
      case HEADER:
        process_header(inChar);
        return;
      case BODY:
        process_body(inChar);
        return;
    }
  } else {
    msg_buf[msg_idx] = '\0';  // end of message
    parse_msg();
  }
}

//Initialization of classes for bluetooth
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *bleServer, esp_ble_gatts_cb_param_t *param) {
    deviceConnected = true;

    // // Set the preferred connection parameters
    // uint16_t minInterval = 0; // Minimum connection interval in 1.25 ms units (50 ms)
    // uint16_t maxInterval = 800; // Maximum connection interval in 1.25 ms units (1000 ms)
    // uint16_t latency = 0;       // Slave latency
    // uint16_t timeout = 5000;     // Supervision timeout in 10 ms units (50 seconds)

    // bleServer->updateConnParams(param->connect.remote_bda, minInterval, maxInterval, latency, timeout);

    Serial.println("BT Connected");
  };

  void onDisconnect(BLEServer *bleServer) {
    deviceConnected = false;
    Serial.println("BT Disconnected");
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string bleReceiver = pCharacteristic->getValue();
    if (bleReceiver.length() > 0) {
      for (int i = 0; i < bleReceiver.length(); i++) {
        on_ble_rx(bleReceiver[i]);
      }
    }
  }
};
#endif


//------------------------------------------------------//
// INITIALIZATION
//------------------------------------------------------//
#if (NO_PHONE_MODE)
unsigned long turn_direction_time = 0;
unsigned long turn_direction_interval = 5000;
unsigned int turn_direction = 0;
int ctrl_max = 192;
int ctrl_slow = 96;
int ctrl_min = 40;
#endif


//Vehicle Control
int ctrl_left = 0;
int ctrl_right = 0;

#if (HAS_VOLTAGE_DIVIDER)
// Voltage measurement
unsigned int vin_counter = 0;
const unsigned int vin_array_sz = 10;
int vin_array[vin_array_sz] = { 0 };
unsigned long voltage_interval = 1000;  //Interval for sending voltage measurements
unsigned long voltage_time = 0;
#endif


//Heartbeat
unsigned long heartbeat_interval = -1;
unsigned long heartbeat_time = 0;

//------------------------------------------------------//
// SETUP
//------------------------------------------------------//
void setup() {

  // Outputs
#if (OPENBOT == RC_CAR)
  pinMode(PIN_PWM_T, OUTPUT);
  pinMode(PIN_PWM_S, OUTPUT);
  // Attach the ESC and SERVO
  ESC.attach(PIN_PWM_T, 1000, 2000);    // (pin, min pulse width, max pulse width in microseconds)
  SERVO.attach(PIN_PWM_S, 1000, 2000);  // (pin, min pulse width, max pulse width in microseconds)
#endif
#if (MCU == NANO)
  pinMode(PIN_PWM_L1, OUTPUT);
  pinMode(PIN_PWM_L2, OUTPUT);
  pinMode(PIN_PWM_R1, OUTPUT);
  pinMode(PIN_PWM_R2, OUTPUT);
#endif
  // Initialize with the I2C addr 0x3C
  Serial.begin(115200, SERIAL_8N1);
  Serial.println('r');

#if (HAS_BLUETOOTH)
  String ble_name = "OpenBot: " + robot_type;
  BLEDevice::init(ble_name.c_str());
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = bleServer->createService(BLEUUID(SERVICE_UUID));

  pTxCharacteristic = pService->createCharacteristic(BLEUUID(CHARACTERISTIC_UUID_TX), BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  pRxCharacteristic = pService->createCharacteristic(BLEUUID(CHARACTERISTIC_UUID_RX), BLECharacteristic::PROPERTY_WRITE_NR);
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  pRxCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLEUUID(SERVICE_UUID));
  bleServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
#endif
}

//------------------------------------------------------//
//LOOP
//------------------------------------------------------//
void loop() {
  ctrl_max = 10;
  ctrl_slow = 5;
  ctrl_min = 2;
  Serial.print("ctrl_left: ");
  Serial.println(ctrl_left);
  Serial.print("ctrl_right: ");
  Serial.println(ctrl_right);
  Serial.print("ctrl_max: ");
  Serial.println(ctrl_max);    
  Serial.print("ctrl_min: ");
  Serial.println(ctrl_min);


  Serial.print("PIN_PWM_L1: ");
  Serial.println(analogRead(PIN_PWM_L1));
  Serial.print("PIN_PWM_R1: ");
  Serial.println(analogRead(PIN_PWM_R1));

  //analogWrite(PIN_PWM_L1, 200);
  //analogWrite(PIN_PWM_R1, 200);

  delay(1000);
#if (HAS_BLUETOOTH)
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                     // give the bluetooth stack the chance to get things ready
    bleServer->startAdvertising();  // restart advertising
    Serial.println("Waiting a client connection to notify...");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
#endif

#if (NO_PHONE_MODE)
  const unsigned int TURN_DISTANCE = 50;
  unsigned int distance_estimate = -1;    //cm

  if ((millis() - turn_direction_time) >= turn_direction_interval) {
    turn_direction_time = millis();
    turn_direction = random(2);  //Generate random number in the range [0,1]
  }
  // Drive forward
  if (distance_estimate > 3 * TURN_DISTANCE) {
    ctrl_left = distance_estimate;
    ctrl_right = ctrl_left;
    digitalWrite(PIN_LED_LI, LOW);
    digitalWrite(PIN_LED_RI, LOW);
  }
  // Turn slightly
  else if (distance_estimate > 2 * TURN_DISTANCE) {
    ctrl_left = distance_estimate;
    ctrl_right = ctrl_left / 2;
  }
  // Turn strongly
  else if (distance_estimate > TURN_DISTANCE) {
    ctrl_left = ctrl_max;
    ctrl_right = -ctrl_max;
  }
  // Drive backward slowly
  else {
    ctrl_left = -ctrl_slow;
    ctrl_right = -ctrl_slow;
    digitalWrite(PIN_LED_LI, HIGH);
    digitalWrite(PIN_LED_RI, HIGH);
  }
  // Flip controls if needed and set indicator light
  if (ctrl_left != ctrl_right) {
    if (turn_direction > 0) {
      int temp = ctrl_left;
      ctrl_left = ctrl_right;
      ctrl_right = temp;
      digitalWrite(PIN_LED_LI, HIGH);
      digitalWrite(PIN_LED_RI, LOW);
    } else {
      digitalWrite(PIN_LED_LI, LOW);
      digitalWrite(PIN_LED_RI, HIGH);
    }
  }

  // Enforce limits
  ctrl_left = ctrl_left > 0 ? max(ctrl_min, min(ctrl_left, ctrl_max)) : min(-ctrl_min, max(ctrl_left, -ctrl_max));
  ctrl_right = ctrl_right > 0 ? max(ctrl_min, min(ctrl_right, ctrl_max)) : min(-ctrl_min, max(ctrl_right, -ctrl_max));

  ctrl_left = ctrl_left > 200 ? ctrl_left:200;
  ctrl_right = ctrl_right > 200 ? ctrl_right:200;
#else  // Check for messages from the phone
  if (Serial.available() > 0) {
    on_serial_rx();
  }
  if (distance_estimate <= STOP_DISTANCE && ctrl_left > 0 && ctrl_right > 0) {
    ctrl_left = 0;
    ctrl_right = 0;
  }
  if ((millis() - heartbeat_time) >= heartbeat_interval) {
    ctrl_left = 0;
    ctrl_right = 0;
  }
#endif

  update_vehicle();
}

//------------------------------------------------------//
// FUNCTIONS
//------------------------------------------------------//

void update_vehicle() {
#if (OPENBOT == RC_CAR)
  update_throttle();
  update_steering();
#else
  update_left_motors();
  update_right_motors();
#endif
}

#if (OPENBOT == RC_CAR)
void update_throttle() {
  if (ctrl_left == 0 || ctrl_right == 0) {
    ESC.write(90);  //set throttle to zero
  } else {
    int throttle = map(ctrl_left + ctrl_right, -510, 510, 0, 180);
    ESC.write(throttle);
  }
}

void update_steering() {
  int steering = map(ctrl_left - ctrl_right, -510, 510, 0, 180);
  if (ctrl_left + ctrl_right < 0) {
    SERVO.write(steering);
  } else {
    SERVO.write(180 - steering);
  }
}

#else

void update_left_motors() {
  if (ctrl_left < 0) {
    analogWrite(PIN_PWM_L1, -ctrl_left);
    analogWrite(PIN_PWM_L2, 0);
  } else if (ctrl_left > 0) {
    analogWrite(PIN_PWM_L1, 0);
    analogWrite(PIN_PWM_L2, ctrl_left);
  } else {
    if (coast_mode) {
      coast_left_motors();
    } else {
      stop_left_motors();
    }
  }
}

void stop_left_motors() {
  analogWrite(PIN_PWM_L1, 255);
  analogWrite(PIN_PWM_L2, 255);
}

void coast_left_motors() {
  analogWrite(PIN_PWM_L1, 0);
  analogWrite(PIN_PWM_L2, 0);
}

void update_right_motors() {
  if (ctrl_right < 0) {
    analogWrite(PIN_PWM_R1, -ctrl_right);
    analogWrite(PIN_PWM_R2, 0);
  } else if (ctrl_right > 0) {
    analogWrite(PIN_PWM_R1, 0);
    analogWrite(PIN_PWM_R2, ctrl_right);
  } else {
    if (coast_mode) {
      coast_right_motors();
    } else {
      stop_right_motors();
    }
  }
}

void stop_right_motors() {
  analogWrite(PIN_PWM_R1, 255);
  analogWrite(PIN_PWM_R2, 255);
}

void coast_right_motors() {
  analogWrite(PIN_PWM_R1, 0);
  analogWrite(PIN_PWM_R2, 0);
}

#endif

boolean almost_equal(int a, int b, int eps) {
  return abs(a - b) <= eps;
}

void process_ctrl_msg() {
  char *tmp;                    // this is used by strtok() as an index
  tmp = strtok(msg_buf, ",:");  // replace delimiter with \0
  ctrl_left = atoi(tmp);        // convert to int
  tmp = strtok(NULL, ",:");     // continues where the previous call left off
  ctrl_right = atoi(tmp);       // convert to int
#if DEBUG
  Serial.print("Control: ");
  Serial.print(ctrl_left);
  Serial.print(",");
  Serial.println(ctrl_right);
#endif
}

#if (HAS_LEDS_FRONT || HAS_LEDS_BACK)
void process_light_msg() {
  char *tmp;                    // this is used by strtok() as an index
  tmp = strtok(msg_buf, ",:");  // replace delimiter with \0
  light_front = atoi(tmp);      // convert to int
  tmp = strtok(NULL, ",:");     // continues where the previous call left off
  light_back = atoi(tmp);       // convert to int
#if DEBUG
  Serial.print("Light: ");
  Serial.print(light_front);
  Serial.print(",");
  Serial.println(light_back);
#endif
  update_light();
}
#endif

void process_heartbeat_msg() {
  heartbeat_interval = atol(msg_buf);  // convert to long
  heartbeat_time = millis();
#if DEBUG
  Serial.print("Heartbeat Interval: ");
  Serial.println(heartbeat_interval);
#endif
}

#if HAS_INDICATORS

void process_indicator_msg() {
  char *tmp;                    // this is used by strtok() as an index
  tmp = strtok(msg_buf, ",:");  // replace delimiter with \0
  indicator_left = atoi(tmp);   // convert to int
  tmp = strtok(NULL, ",:");     // continues where the previous call left off
  indicator_right = atoi(tmp);  // convert to int
#if DEBUG
  Serial.print("Indicator: ");
  Serial.print(indicator_left);
  Serial.print(",");
  Serial.println(indicator_right);
#endif
}

#endif

#if HAS_LEDS_STATUS
void process_notification_msg() {
  char *tmp;                    // this is used by strtok() as an index
  tmp = strtok(msg_buf, ",:");  // replace delimiter with \0
  char led = tmp[0];
  tmp = strtok(NULL, ",:");  // continues where the previous call left off
  int state = atoi(tmp);     // convert to int
  switch (led) {
    case 'y':
      digitalWrite(PIN_LED_Y, state);
      break;
    case 'g':
      digitalWrite(PIN_LED_G, state);
      break;
    case 'b':
      digitalWrite(PIN_LED_B, state);
      break;
  }
#if DEBUG
  Serial.print("Notification: ");
  Serial.print(led);
  Serial.println(state);
#endif
}
#endif

#if HAS_BUMPER
void process_bumper_msg() {
  bumper_interval = atol(msg_buf);  // convert to long
}
#endif
#if HAS_SONAR

void process_sonar_msg() {
  sonar_interval = atol(msg_buf);  // convert to long
}

#endif

void process_voltage_msg() {
#if HAS_VOLTAGE_DIVIDER
  voltage_interval = atol(msg_buf);  // convert to long
#endif
  Serial.println(String("vmin:") + String(VOLTAGE_MIN, 2));
  Serial.println(String("vlow:") + String(VOLTAGE_LOW, 2));
  Serial.println(String("vmax:") + String(VOLTAGE_MAX, 2));
}

#if (HAS_SPEED_SENSORS_FRONT or HAS_SPEED_SENSORS_BACK or HAS_SPEED_SENSORS_MIDDLE)

void process_wheel_msg() {
  wheel_interval = atol(msg_buf);  // convert to long
}

#endif

void process_feature_msg() {
  String msg = "f" + robot_type + ":";
#if HAS_VOLTAGE_DIVIDER
  msg += "v:";
#endif
#if HAS_INDICATORS
  msg += "i:";
#endif
#if HAS_SONAR
  msg += "s:";
#endif
#if HAS_BUMPER
  msg += "b:";
#endif
#if HAS_SPEED_SENSORS_FRONT
  msg += "wf:";
#endif
#if HAS_SPEED_SENSORS_BACK
  msg += "wb:";
#endif
#if HAS_SPEED_SENSORS_MIDDLE
  msg += "wm:";
#endif
#if HAS_LEDS_FRONT
  msg += "lf:";
#endif
#if HAS_LEDS_BACK
  msg += "lb:";
#endif
#if HAS_LEDS_STATUS
  msg += "ls:";
#endif
  sendData(msg);
}

void on_serial_rx() {
  char inChar = Serial.read();
  if (inChar != endChar) {
    switch (msgPart) {
      case HEADER:
        process_header(inChar);
        return;
      case BODY:
        process_body(inChar);
        return;
    }
  } else {
    msg_buf[msg_idx] = '\0';  // end of message
    parse_msg();
  }
}

void process_header(char inChar) {
  header = inChar;
  msgPart = BODY;
}

void process_body(char inChar) {
  // Add the incoming byte to the buffer
  msg_buf[msg_idx] = inChar;
  msg_idx++;
}

void parse_msg() {
  switch (header) {
#if HAS_BUMPER
    case 'b':
      process_bumper_msg();
      break;
#endif
    case 'c':
      process_ctrl_msg();
      break;
    case 'f':
      process_feature_msg();
      break;
    case 'h':
      process_heartbeat_msg();
      break;
#if HAS_INDICATORS
    case 'i':
      process_indicator_msg();
      break;
#endif
#if (HAS_LEDS_FRONT || HAS_LEDS_BACK)
    case 'l':
      process_light_msg();
      break;
#endif
#if HAS_LEDS_STATUS
    case 'n':
      process_notification_msg();
      break;
#endif
#if HAS_SONAR
    case 's':
      process_sonar_msg();
      break;
#endif
#if HAS_VOLTAGE_DIVIDER
    case 'v':
      process_voltage_msg();
      break;
#endif
#if (HAS_SPEED_SENSORS_FRONT or HAS_SPEED_SENSORS_BACK or HAS_SPEED_SENSORS_MIDDLE)
    case 'w':
      process_wheel_msg();
      break;
#endif
  }
  msg_idx = 0;
  msgPart = HEADER;
  header = '\0';
}

#if HAS_OLED
// Function for drawing a string on the OLED display
void drawString(String line1, String line2, String line3, String line4) {
  display.clearDisplay();
  // set text color
  display.setTextColor(WHITE);
  // set text size
  display.setTextSize(1);
  // set text cursor position
  display.setCursor(1, 0);
  // show text
  display.println(line1);
  display.setCursor(1, 8);
  // show text
  display.println(line2);
  display.setCursor(1, 16);
  // show text
  display.println(line3);
  display.setCursor(1, 24);
  // show text
  display.println(line4);
  display.display();
}
#endif

#if (HAS_OLED || DEBUG)

void display_vehicle_data() {
#if HAS_VOLTAGE_DIVIDER
  float voltage_value = get_voltage();
  String voltage_str = String("Voltage:    ") + String(voltage_value, 2);
#else
  String voltage_str = String("Voltage:    ") + String("N/A");
#endif
#if (HAS_SPEED_SENSORS_FRONT or HAS_SPEED_SENSORS_BACK or HAS_SPEED_SENSORS_MIDDLE)
  String left_rpm_str = String("Left RPM:  ") + String(rpm_left, 0);
  String right_rpm_str = String("Right RPM:  ") + String(rpm_right, 0);
#else
  String left_rpm_str = String("Left RPM:  ") + String("N/A");
  String right_rpm_str = String("Right RPM:  ") + String("N/A");
#endif
#if HAS_SONAR
  String distance_str = String("Distance:   ") + String(distance_estimate);
#else
  String distance_str = String("Distance:   ") + String("N/A");
#endif
#if DEBUG
  Serial.println("------------------");
  Serial.println(voltage_str);
  Serial.println(left_rpm_str);
  Serial.println(right_rpm_str);
  Serial.println(distance_str);
  Serial.println("------------------");
#endif
#if HAS_OLED
  // Set display information
  drawString(
    voltage_str,
    left_rpm_str,
    right_rpm_str,
    distance_str);
#endif
}

#endif

#if (HAS_VOLTAGE_DIVIDER)

void send_voltage_reading() {
  sendData("v" + String(get_voltage(), 2));
}

#endif

#if (HAS_SPEED_SENSORS_FRONT or HAS_SPEED_SENSORS_BACK or HAS_SPEED_SENSORS_MIDDLE)

void send_wheel_reading(long duration) {
  float rpm_factor = 60.0 * 1000.0 / duration / TICKS_PER_REV;
  rpm_left = (counter_lf + counter_lb + counter_lm) * rpm_factor;
  rpm_right = (counter_rf + counter_rb + counter_rm) * rpm_factor;
  counter_lf = 0;
  counter_rf = 0;
  counter_lb = 0;
  counter_rb = 0;
  counter_lm = 0;
  counter_rm = 0;
#if (HAS_SPEED_SENSORS_FRONT and HAS_SPEED_SENSORS_BACK and HAS_SPEED_SENSORS_MIDDLE)
  sendData("w" + String(rpm_left / 3) + "," + String(rpm_right / 3));
#elif ((HAS_SPEED_SENSORS_FRONT and HAS_SPEED_SENSORS_BACK) or (HAS_SPEED_SENSORS_FRONT and HAS_SPEED_SENSORS_MIDDLE) or (HAS_SPEED_SENSORS_MIDDLE and HAS_SPEED_SENSORS_BACK))
  sendData("w" + String(rpm_left / 2) + "," + String(rpm_right / 2));
#elif (HAS_SPEED_SENSORS_FRONT or HAS_SPEED_SENSORS_BACK or HAS_SPEED_SENSORS_MIDDLE)
  sendData("w" + String(rpm_left) + "," + String(rpm_right));
#endif
}

#endif

#if (HAS_INDICATORS)

void update_indicator() {
  if (indicator_left > 0) {
#if ((OPENBOT == RTR_TT2 || OPENBOT == RTR_520 || OPENBOT == DIY_ESP32) && PIN_LED_LI == PIN_LED_LB)
    ledcDetachPin(PIN_LED_LB);
#endif
    digitalWrite(PIN_LED_LI, !digitalRead(PIN_LED_LI));
  } else {
#if (HAS_LEDS_BACK)
    digitalWrite(PIN_LED_LI, PIN_LED_LI == PIN_LED_LB ? light_back : LOW);
#else
    digitalWrite(PIN_LED_LI, LOW);
#endif
#if ((OPENBOT == RTR_TT2 || OPENBOT == RTR_520 || OPENBOT == DIY_ESP32) && PIN_LED_LI == PIN_LED_LB)
    ledcAttachPin(PIN_LED_LB, CH_LED_LB);
#endif
  }
  if (indicator_right > 0) {
#if ((OPENBOT == RTR_TT2 || OPENBOT == RTR_520 || OPENBOT == DIY_ESP32) && PIN_LED_RI == PIN_LED_RB)
    ledcDetachPin(PIN_LED_RB);
#endif
    digitalWrite(PIN_LED_RI, !digitalRead(PIN_LED_RI));
  } else {
#if (HAS_LEDS_BACK)
    digitalWrite(PIN_LED_RI, PIN_LED_RI == PIN_LED_RB ? light_back : LOW);
#else
    digitalWrite(PIN_LED_RI, LOW);
#endif
#if ((OPENBOT == RTR_TT2 || OPENBOT == RTR_520 || OPENBOT == DIY_ESP32) && PIN_LED_RI == PIN_LED_RB)
    ledcAttachPin(PIN_LED_RB, CH_LED_RB);
#endif
  }
}

#endif

#if (HAS_LEDS_FRONT || HAS_LEDS_BACK)
void update_light() {
#if (HAS_LEDS_FRONT)
#if (OPENBOT == RTR_TT2 || OPENBOT == RTR_520 || OPENBOT == DIY_ESP32)
  analogWrite(CH_LED_LF, light_front);
  analogWrite(CH_LED_RF, light_front);
#else
  analogWrite(PIN_LED_LF, light_front);
  analogWrite(PIN_LED_RF, light_front);
#endif
#endif

#if (HAS_LEDS_BACK)
#if (OPENBOT == RTR_TT2 || OPENBOT == RTR_520 || OPENBOT == DIY_ESP32)
  analogWrite(CH_LED_LB, light_back);
  analogWrite(CH_LED_RB, light_back);
#else
  analogWrite(PIN_LED_LB, light_back);
  analogWrite(PIN_LED_RB, light_back);
#endif
#endif
}
#endif

int get_median(int a[], int sz) {
  //bubble sort
  for (int i = 0; i < (sz - 1); i++) {
    for (int j = 0; j < (sz - (i + 1)); j++) {
      if (a[j] > a[j + 1]) {
        int t = a[j];
        a[j] = a[j + 1];
        a[j + 1] = t;
      }
    }
  }
  return a[sz / 2];
}

#if HAS_SONAR

void send_sonar_reading() {
  sendData("s" + String(distance_estimate));
}

// Send pulse by toggling trigger pin
void send_ping() {
  echo_time = 0;
  ping_success = false;
  if (PIN_TRIGGER == PIN_ECHO)
    pinMode(PIN_TRIGGER, OUTPUT);
  digitalWrite(PIN_TRIGGER, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIGGER, LOW);
  if (PIN_TRIGGER == PIN_ECHO)
    pinMode(PIN_ECHO, INPUT);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(PIN_ECHO), start_timer, RISING);
}

void update_distance_estimate() {
#if SONAR_MEDIAN
  distance_array[distance_counter % distance_array_sz] = distance;
  distance_counter++;
  distance_estimate = get_median(distance_array, distance_array_sz);
#else
  distance_estimate = distance;
#endif
}

#endif


void sendData(String data) {
Serial.print(data);
Serial.println();
#if (HAS_BLUETOOTH)
  if (deviceConnected) {
    char outData[MAX_MSG_SZ] = "";
    for (int i = 0; i < data.length(); i++) {
      outData[i] = data[i];
    }
    pTxCharacteristic->setValue(outData);
    pTxCharacteristic->notify();
  }
#endif
}
