// Ros Includes
#include <ros.h>
#include <cara_base_msgs/Imu.h>
#include <cara_base_msgs/Encoders.h>
#include <cara_base_msgs/SpeedCommand.h>
#include <cara_base_msgs/SteeringCommand.h>
#include <cara_base_msgs/BaseboardCommand.h>
#include <cara_base_msgs/BaseboardInfo.h>
#include <cara_base_msgs/EscArm.h>
#include <cara_base_msgs/Headlight.h>
// Hardware Includes
#include <Servo.h>
#include <Wire.h>

#define ESC_SIGNAL 10
#define STEERING_SIGNAL 11

Servo steering_;
Servo motor_;
const int MPU_addr=0x68;

ros::NodeHandle  nh;

void BaseboardCommandCallback(const cara_base_msgs::BaseboardCommand& msg) {
  steercom = msg.steeringcommand.steer;
  speedcom = msg.speedcommand.speed;
  if (steercom != 0) {
    // Logic to write steer command.
    steering_.writeMicroseconds(steercom)
  }
  if (speedcom != 0) {
    // logic to write speed command
    motor_.writeMicroseconds(speedcom)
  }
}

cara_base_msgs::BaseboardInfo PackageBaseboardInfo(cara_base_msgs::Imu imu, cara_base_msgs::Encoders enc) {
  msg = cara_base_msgs::BaseboardInfo()
  msg.imu = imu;
  msg.encoders = enc;
  return msg
}

void RetrieveImuData(uint16_t imu_data[]) {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  imu_data[0] = Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  imu_data[1] = Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  imu_data[2] = Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  imu_data[3] = Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  imu_data[4] = Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  imu_data[5] = Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  imu_data[6] = Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void RetrieveEncoderData(uint16_t& encoder_data) {
  encoder_data = 0 //Once encoders are installed, this will change.
}

ros::Subscriber<cara_base_msgs::BaseboardCommand> sub("basecommand", BaseboardCommandCallback);
cara_base_msgs::BaseboardInfo info;
ros::Publisher base_info("baseinfo", info)


char hello[13] = "hello world!";

void setup()
{
  // Ros initializations
  nh.initNode();
  nh.advertise(base_info);
  nh.subscribe(sub);

  // Hardware Initializations
  steering_.attach(STEERING_SIGNAL)
  motor_.attach(ESC_SIGNAL)
  steering_.writeMicroseconds(1500)
  motor_.writeMicroseconds(1500)
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 reg
  Wire.write(0);    // Set to zero and wake up mpu
  Wire.endTransmission(true);

}

void loop()
{
  uint16_t imu_data[7];
  uint16_t encoder_data;
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  // delay(50);
}
