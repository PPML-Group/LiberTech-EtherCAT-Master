
//
// Created by qiayuan on 1/24/22.
//

/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include <iostream>
#include <fstream>
#include <legged_hw/LeggedHW.h>

#include <memory.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <cstdio>
// #include "Console.hpp"

#include <std_msgs/Float32MultiArray.h>
#include <math.h>
#include <controller_manager_msgs/SwitchController.h>

#include "ethercat/EthercatDeviceConfigurator.hpp"


namespace legged
{
const std::vector<std::string> CONTACT_SENSOR_NAMES = { "RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT" };

struct freemanMotorData
{
  double pos_, vel_, tau_;
  double pos_des_, vel_des_, kp_, kd_, ff_;
  double tau_des_;
};

struct freemanImuData
{
  double ori[4];
  double ori_cov[9];
  double angular_vel[3];
  double angular_vel_cov[9];
  double linear_acc[3];
  double linear_acc_cov[9];
};

class BridgeHW : public LeggedHW
{
public:
  BridgeHW()
  {
  }

  ~BridgeHW();

  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * Call @ref Gsmp_LEGGED_SDK::UDP::Recv() to get robot's state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref Gsmp_LEGGED_SDK::UDP::Recv(). Publish actuator
   * current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

private:
//******* EtherCAT *******/
  bool abrt = false;
  EthercatDeviceConfigurator::SharedPtr configurator;
  std::string ethercatConfigFile;
  // std::unique_ptr<std::thread> ethercat_thread;
  std::thread ethercat_thread;

  bool setupEtherCAT();

//******* Joint *******/
  freemanMotorData jointData_[10]{};
  freemanMotorData ActuatorData_[10]{};
  int jointOffset_[10];
  bool setupJoints();

//******* Hardware ********//
  int powerLimit_{};

//******* Contact *******/
  int contactThreshold_{};
  bool estimateContact_[4];

  bool setupContactSensor(ros::NodeHandle& nh);

 //****** IMU ******//
  freemanImuData imuData_{};
  sensor_msgs::Imu imuMsg_;

  ros::Subscriber imu_sub_;
  bool setupImu();
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

};

}  // namespace legged
