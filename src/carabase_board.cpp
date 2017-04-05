#include <Arduino.h>
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
#define MPU_addr 0x68

Servo steering_;
Servo motor_;

ros::NodeHandle  nh;


void BaseboardCommandCallback(const cara_base_msgs::BaseboardCommand& msg) {
  if (msg.steeringcommand.steer != 0)
    steering_.writeMicroseconds(msg.steeringcommand.steer);
  if (msg.speedcommand.speed != 0)
    motor_.writeMicroseconds(msg.speedcommand.speed);
}

void PackageBaseboardInfo(uint16_t imu_data[],
                          uint16_t& enc,
                          cara_base_msgs::BaseboardInfo& msg) {
  msg.imu.accelerator_x = imu_data[0];
  msg.imu.accelerator_y = imu_data[1];
  msg.imu.accelerator_z = imu_data[2];
  msg.imu.temperature = imu_data[3];
  msg.imu.gyroscope_x = imu_data[4];
  msg.imu.gyroscope_y = imu_data[5];
  msg.imu.gyroscope_z = imu_data[6];
  msg.encoders.motor_speed = enc;
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


ros::Subscriber<cara_base_msgs::BaseboardCommand> sub("basecommand", BaseboardCommandCallback);
cara_base_msgs::BaseboardInfo info;
ros::Publisher base_info("baseinfo", &info);


void setup()
{
  // Ros initializations
  nh.initNode();
  nh.advertise(base_info);
  nh.subscribe(sub);

  // Hardware Initializations
  steering_.attach(STEERING_SIGNAL);
  motor_.attach(ESC_SIGNAL);
  steering_.writeMicroseconds(1500);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B); // PWR_MGMT_1 reg
  Wire.write(0);    // Set to zero and wake up mpu
  Wire.endTransmission(true);

}

void loop()
{
  uint16_t imu_data[7];
  RetrieveImuData(imu_data);
  PackageBaseboardInfo(imu_data, encoder_data, info);
  base_info.publish(&info);

  nh.spinOnce();
  // delay(50);
}


// Override = false
//
// loop
// if not Override
//  pulse in
//  pulse out
//
// else
//  //
//
// is bind pressed? aka Override activated?
//
