#include <Arduino.h>
#include <HerkulexServo.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



// Servos
#define SERVO_YAW_ID  4
#define SERVO_LOWER_ID 3

#define YAW_CENTRE 131
#define LOWER_CENTRE 126

#define WIRE_PINS I2C_PINS_18_19
#define WIRE1_PINS I2C_PINS_22_23

// IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

HerkulexServoBus herkulex_bus(Serial3);
HerkulexServo    yaw_servo(herkulex_bus, SERVO_YAW_ID);
HerkulexServo    lower_servo(herkulex_bus, SERVO_LOWER_ID);

unsigned long last_update = 0;
unsigned long now = 0;
bool toggle = false;

float Kp_desiredSpeeds = 10.5;
float Kp_speed = 5;
float Kd_speed = 5;

int state = 2;
bool set_target_flag = false;
int desired_speed_lower;
int desired_speed_yaw;
int position = -1;
bool servosOff = false;

struct delta {
  int lower, yaw;
};
typedef struct delta servoDeltas;

void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.println(mag, DEC);
}


servoDeltas psuedoTarget() {
  float lower_target_deg = -45.0;   // target angle from starting pos
  float yaw_target_deg = -45.0;
  int alpha = lower_target_deg + LOWER_CENTRE * 0.325;
  int theta = yaw_target_deg + YAW_CENTRE * 0.325;
  if (alpha > 360) {
    alpha -= 360;
  }
  if (theta > 360) {
    theta -= 360;
  }
  float lower_target_pos = uint16_t(alpha/0.325);
  float yaw_target_pos = uint16_t(theta/0.325);

  int delta_lower = lower_target_pos - lower_servo.getPosition();
  if ((lower_servo.getPosition() < 638) && (lower_target_pos > 638)) {
    delta_lower = -lower_servo.getPosition() - (1024 - lower_target_pos);
  }

  int delta_yaw = yaw_target_pos - yaw_servo.getPosition();
  if ((yaw_servo.getPosition() < 638) && (yaw_target_pos > 638)) {
    delta_yaw = -yaw_servo.getPosition() - (1024 - yaw_target_pos);
  } 

  // float dHFOV = (delta_lower * 100.0) / 512.0; //scale to -100 to 100
  servoDeltas delta;
  delta.lower = (int)delta_lower;
  delta.yaw = (int)delta_yaw;

  return delta;
}


void setLowerPos(int angle, int time) {
  int alpha = angle + LOWER_CENTRE * 0.325;
  float playtime = time / 11.2;
  if (alpha > 360) {
    alpha -= 360;
  }
  int pos = uint16_t(alpha/0.325);
  // Serial.println(pos);
  lower_servo.setPosition(pos,int(playtime),HerkulexLed::Red);
}

void setYawPos(int angle, int time) {
  int alpha = angle + YAW_CENTRE * 0.325;
  float playtime = time / 11.2;
  if (alpha > 360) {
    alpha -= 360;
  }
  int pos = uint16_t(alpha/0.325);
  // Serial.println(pos);
  yaw_servo.setPosition(pos,int(playtime),HerkulexLed::Red);
}


void setup() {

  Wire.begin();
  Wire.setSDA(18);
  Wire.setSCL(19);

  pinMode(16, OUTPUT);      // IMU address pin
  digitalWrite(16, LOW);
  pinMode(15, OUTPUT);      // IMU reset pin
  digitalWrite(15, HIGH);

  delay(100);
  Serial.begin(9600);
  delay(100);
  Serial3.begin(115200);
  delay(100);

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  while(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
  }

  // yaw_servo.setTorqueOff();
  // lower_servo.setTorqueOff();

  // yaw_servo.enableSpeedControlMode();
  // lower_servo.enableSpeedControlMode();

  // yaw_servo.setTorqueOn();
  // lower_servo.setTorqueOn();
  // // yaw_servo.enablePositionControlMode();
  // // lower_servo.enablePositionControlMode();

  lower_servo.setTorqueOff();
  yaw_servo.setTorqueOff();
  delay(20);
  lower_servo.enableSpeedControlMode();
  yaw_servo.enableSpeedControlMode();
  delay(20);
  lower_servo.setTorqueOn();
  yaw_servo.setTorqueOn();

  delay(100);

  bno.setExtCrystalUse(true);

  bno.setMode(OPERATION_MODE_NDOF);

}



void loop() {

  bno.setMode(OPERATION_MODE_NDOF);
  delay(4);
  sensors_event_t event;
  bno.getEvent(&event);
  imu::Vector<3> omega = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  /* Display the floating point data */
  // Serial.print("x: ");
  // Serial.print(event.orientation.x, 4);
  // Serial.print("\tY: ");
  // Serial.print(event.orientation.y, 4);
  // Serial.print("\tZ: ");
  // Serial.print(event.orientation.z, 4);
  // Serial.println("");


  if (state == 1) {    // manual calibration, move IMU around and place flat on surface in different orientations
    Serial.print("Calibration manual:  ");
    displayCalStatus();
    now = millis();
    if (!servosOff) {
      lower_servo.setTorqueOff();
      yaw_servo.setTorqueOff();
      last_update = now;
      servosOff = true;
    }
    if ((now - last_update) > 15000) {
        Serial.println("done");
        lower_servo.enableSpeedControlMode();
        yaw_servo.enableSpeedControlMode();
        delay(20);
        lower_servo.setTorqueOn();
        yaw_servo.setTorqueOn();
        state = 2;
    }
  }

  if (state == 0) {   // calibration routine auto
    displayCalStatus();
    now = millis();
    if ((now - last_update) > 4000) {
      if (position == -1) {
          lower_servo.setTorqueOff();
          yaw_servo.setTorqueOff();
          delay(20);
          lower_servo.enablePositionControlMode();
          yaw_servo.enablePositionControlMode();
          delay(20);
          lower_servo.setTorqueOn();
          yaw_servo.setTorqueOn();
      }

      position++;
      Serial.println(position);

      if (position == 0) {
        setYawPos(0, 2000);
        setLowerPos(0,2000);
        Serial.println(position);
      }
      if (position == 1) {
        setYawPos(0, 2000);
        setLowerPos(90,2000);
        Serial.println(position);
      }
      if (position == 2) {
        setYawPos(0, 2000);
        setLowerPos(0,2000);
        Serial.println(position);
      }
      if (position == 3) {
        setYawPos(0, 2000);
        setLowerPos(270,2000);
      }
      if (position == 4) {
        setYawPos(0, 2000);
        setLowerPos(0,2000);
      }
      if (position == 5) {
        setYawPos(0, 2000);
        setLowerPos(90,2000);
      }
      if (position == 6) {
        setYawPos(0, 2000);
        setLowerPos(0,2000);
      }
      if (position == 7) {
        state = 2;
        Serial.println("done");
        lower_servo.setTorqueOff();
        yaw_servo.setTorqueOff();
        delay(20);
        lower_servo.enableSpeedControlMode();
        yaw_servo.enableSpeedControlMode();
        delay(20);
        lower_servo.setTorqueOn();
        yaw_servo.setTorqueOn();
      }

      last_update = now;
        
      }
    }


  //***SERVOS***
  herkulex_bus.update();

  // Check lower servo within angular range
  int lower_pos = lower_servo.getPosition();
  int lower_thresh_l = (LOWER_CENTRE + 512) - (45 / 0.325);
  int upper_thresh_l = (LOWER_CENTRE + 512) + (45 / 0.325);
  if ((lower_pos > lower_thresh_l) && (lower_pos < upper_thresh_l)) {
    lower_servo.setTorqueOff();
    lower_servo.enablePositionControlMode();
    lower_servo.setTorqueOn();
    setLowerPos(0, 2000);
    while(1) {
      Serial.println("Exceeded rotational range!");
    }
  }

  // Check yaw servo within angular range
  int yaw_pos = yaw_servo.getPosition();
  int lower_thresh_y = (YAW_CENTRE + 512) - (45 / 0.325);
  int upper_thresh_y = (YAW_CENTRE + 512) + (45 / 0.325);
  if ((yaw_pos > lower_thresh_y) && (yaw_pos < upper_thresh_y)) {
    yaw_servo.setTorqueOff();
    yaw_servo.enablePositionControlMode();
    yaw_servo.setTorqueOn();
    setYawPos(0, 2000);
    while(1) {
      Serial.println("Exceeded rotational range!");
    }
  }


  if (state == 2) {   // tracking target
      // get targets
      servoDeltas delta = psuedoTarget();
      Serial.print("delta: "); Serial.println(delta.yaw);

      // calculate desired speeds
      if (delta.lower > 1 && delta.lower < 1) {
        desired_speed_lower = Kp_desiredSpeeds * delta.lower;
      } else {
        desired_speed_lower = 0;
      }
      if (delta.yaw > 1 && delta.yaw < 1) {
        desired_speed_yaw = Kp_desiredSpeeds * delta.yaw;
      } else {
        desired_speed_yaw = 0;
      }
      
      // set desired speeds
        // get angular speeds and apply PID control
      float lower_current_speed = omega.x() * (512/PI);
      float yaw_current_speed = omega.z() * (512/PI);

      float lower_set_speed = (lower_current_speed - desired_speed_lower) * Kp_desiredSpeeds;
      float yaw_set_speed = (yaw_current_speed - desired_speed_yaw) * Kp_desiredSpeeds;
      
      lower_servo.setSpeed(lower_set_speed);
      yaw_servo.setSpeed(yaw_set_speed);

      Serial.print("X: ");
      Serial.print(omega.x(), 4);
      Serial.print("\tY: ");
      Serial.print(omega.y(), 4);
      Serial.print("\tZ: ");
      Serial.print(omega.z(), 4);
      Serial.println("");

  } 

}  