#include "legged_bridge_hw/BridgeHW.h"
#include "std_msgs/Float64MultiArray.h"
#include <ostream>
#include <vector>
#include <iostream>
#include <cmath>

#include <elmo_ethercat_sdk/Elmo.hpp>

#include <thread>
#include <csignal>


namespace legged
{

  // void setupEtherCAT()
  // {
  //     bool rtSuccess = true;
  //     for(const auto & master: configurator->getMasters())
  //     {
  //         rtSuccess &= master->setRealtimePriority(99);
  //     }
  //     std::cout << "Setting RT Priority: " << (rtSuccess? "successful." : "not successful. Check user privileges.") << std::endl;

  //     // Flag to set the drive state for the elmos on first startup
  // #ifdef _ELMO_FOUND_
  //     bool elmoEnabledAfterStartup = false;
  // #endif

  //     /*
  //     ** The communication update loop.
  //     ** This loop is supposed to be executed at a constant rate.
  //     ** The EthercatMaster::update function incorporates a mechanism
  //     ** to create a constant rate.
  //     */
  //     while(!abrt)
  //     {
  //         /*
  //         ** Update each master.
  //         ** This sends tha last staged commands and reads the latest readings over EtherCAT.
  //         ** The StandaloneEnforceRate update mode is used.
  //         ** This means that average update rate will be close to the target rate (if possible).
  //         */
  //         for(const auto & master: configurator->getMasters() )
  //         {
  //             master->update(ecat_master::UpdateMode::StandaloneEnforceRate); // TODO fix the rate compensation (Elmo reliability problem)!!
  //         }

  //         /*
  //         ** Do things with the attached devices.
  //         ** Your lowlevel control input / measurement logic goes here.
  //         ** Different logic can be implemented for each device.
  //         */
  //         for(const auto & slave:configurator->getSlaves())
  //         {
  //             // Elmo
  //             if(configurator->getInfoForSlave(slave).type == EthercatDeviceConfigurator::EthercatSlaveType::Elmo)
  //             {
  // // #ifdef _ELMO_FOUND_
  //                 std::shared_ptr<elmo::Elmo> elmo_slave_ptr = std::dynamic_pointer_cast<elmo::Elmo>(slave);

  //                 // if(elmo_slave_ptr->getName() == "ElmoTwitter_3")
  //                 {
  //                     if(!elmoEnabledAfterStartup)
  //                     // Set elmos to operation enabled state, do not block the call!
  //                     elmo_slave_ptr->setDriveStateViaPdo(elmo::DriveState::OperationEnabled, false);

  //                     // set commands if we can
  //                     if(elmo_slave_ptr->lastPdoStateChangeSuccessful() && elmo_slave_ptr->getReading().getDriveState() == elmo::DriveState::OperationEnabled)
  //                     {
  //                     //     elmo::Command command;
  //                     //     command.setModeOfOperation(elmo::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                          
  //                     //     command.setTargetTorque(0.0);
  //                     //     elmo_slave_ptr->stageCommand(command);
  //                         std::cout << elmo_slave_ptr->getName() << "': " << "operation state success!!!\n";
  //                     }
  //                     else
  //                     {
  //                         MELO_WARN_STREAM("Elmo '" << elmo_slave_ptr->getName() << "': " << elmo_slave_ptr->getReading().getDriveState());
  //                         elmo_slave_ptr->setDriveStateViaPdo(elmo::DriveState::OperationEnabled, false);
  //                     }
  //                     // auto reading = elmo_slave_ptr->getReading();
  //                     // std::cout << "Elmo '" << elmo_slave_ptr->getName() << "': "
  //                     //                 << "position: " << reading.getActualPosition() << " rad"
  //                     //                 << " Velocity: " << reading.getActualVelocity() << " rad/s"
  //                     //                 << " torque:" << reading.getActualCurrent() << " A\n";
  // // #endif
  //                 }
  //             }
  //         }
  // #ifdef _ELMO_FOUND_
  //         elmoEnabledAfterStartup = true;
  // #endif

  //     }
  // }


bool BridgeHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
{
  if (!LeggedHW::init(root_nh, robot_hw_nh))
    return false;

  root_nh.setParam("gsmp_controller_switch", "null");

  robot_hw_nh.getParam("power_limit", powerLimit_);

  root_nh.getParam("/ethercatConfigFile", ethercatConfigFile);

  if(setupEtherCAT())
  std::cout << "EtherCAT Setup finished!" << std::endl;

  if(setupJoints())
  {
    std::cout << "Joints setup finished!" << std::endl;
  }

  imu_sub_ = root_nh.subscribe<sensor_msgs::Imu>("imu/data", 1, &BridgeHW::imuCallback, this);
  if(setupImu())
  {
    std::cout << "IMU setup finished!" << std::endl;
  }
  return true;
}

BridgeHW::~BridgeHW() 
{
    /*
    ** Pre shutdown procedure.
    ** The devices execute procedures (e.g. state changes) that are necessary for a
    ** proper shutdown and that must be done with PDO communication.
    ** The communication update loop (i.e. PDO loop) continues to run!
    ** You might thus want to implement some logic that stages zero torque / velocity commands
    ** or simliar safety measures at this point using e.g. atomic variables and checking them
    ** in the communication update loop.
     */
    for(const auto & master: configurator->getMasters())
    {
        master->preShutdown();
    }

    // stop the PDO communication at the next update of the communication loop
    abrt = true;
    if(ethercat_thread.joinable())
    {
      ethercat_thread.join();
    }

    /*
    ** Completely halt the EtherCAT communication.
    ** No online communication is possible afterwards, including SDOs.
     */
    for(const auto & master: configurator->getMasters())
    {
        master->shutdown();
    }

    // Exit this executable
    std::cout << "Shutdown" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    exit(0);
}


void BridgeHW::read(const ros::Time& time, const ros::Duration& period)
{
  for (int i = 0; i < 10; ++i)
  {
    jointData_[i].pos_ = ActuatorData_[i].pos_ + jointOffset_[i]*2*M_PI;
    jointData_[i].vel_ = ActuatorData_[i].vel_;
    jointData_[i].ff_ = ActuatorData_[i].ff_;
  }

  // imuData_.ori[0] = imuData_recv.quat_float[2];          
  // imuData_.ori[1] = -imuData_recv.quat_float[1];
  // imuData_.ori[2] = imuData_recv.quat_float[3];
  // imuData_.ori[3] = imuData_recv.quat_float[0];
  // imuData_.angular_vel[0] = imuData_recv.gyro_float[1];  
  // imuData_.angular_vel[1] = -imuData_recv.gyro_float[0];
  // imuData_.angular_vel[2] = imuData_recv.gyro_float[2];
  // imuData_.linear_acc[0] = imuData_recv.accel_float[1];   
  // imuData_.linear_acc[1] = -imuData_recv.accel_float[0];
  // imuData_.linear_acc[2] = imuData_recv.accel_float[2];

  imuData_.ori[0] = imuMsg_.orientation.x;
  imuData_.ori[1] = imuMsg_.orientation.y;
  imuData_.ori[2] = imuMsg_.orientation.z;
  imuData_.ori[3] = imuMsg_.orientation.w;

  imuData_.angular_vel[0] = imuMsg_.angular_velocity.x;  
  imuData_.angular_vel[1] = imuMsg_.angular_velocity.y;
  imuData_.angular_vel[2] = imuMsg_.angular_velocity.z;
  imuData_.linear_acc[0] = imuMsg_.linear_acceleration.x;   
  imuData_.linear_acc[1] = imuMsg_.linear_acceleration.y;
  imuData_.linear_acc[2] = imuMsg_.linear_acceleration.z;

  // std::cout << "imuData: " << imuData_.linear_acc[0] << " " << imuData_.linear_acc[1] << "" << imuData_.linear_acc[2] << "\n";

  std::vector<std::string> names = hybridJointInterface_.getNames();
  for (const auto& name : names)
  {
    HybridJointHandle handle = hybridJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(0.8);
    handle.setKp(0.);
  }
}

void BridgeHW::write(const ros::Time& time, const ros::Duration& period)
{
  for (int i = 0; i < 10; ++i)
  {
    ActuatorData_[i].tau_des_ = jointData_[i].kp_ * (jointData_[i].pos_des_ - jointData_[i].pos_) + 
                                jointData_[i].kd_ * (jointData_[i].vel_des_ - jointData_[i].vel_) +
                                jointData_[i].ff_;
    ActuatorData_[i].kp_ = jointData_[i].kp_;
    ActuatorData_[i].kd_ = jointData_[i].kd_;

  }

}

bool BridgeHW::setupJoints()
{
  for (const auto& joint : urdfModel_->joints_)
  {
    int leg_index, joint_index;
    if (joint.first.find("left_") != std::string::npos)
    {
      leg_index = 0;
    }
    else if (joint.first.find("right_") != std::string::npos)
    {
      leg_index = 1;
    }
    else
      continue;

    if (joint.first.find("HTAA") != std::string::npos)
      joint_index = 0;
    else if (joint.first.find("HAA") != std::string::npos)
      joint_index = 1;
    else if (joint.first.find("HFE") != std::string::npos)
      joint_index = 2;
    else if (joint.first.find("KFE") != std::string::npos)
      joint_index = 3;
    else if (joint.first.find("AFE") != std::string::npos)
      joint_index = 4;
    else
      continue;

    int index = leg_index * 5 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].pos_des_,
                                                           &jointData_[index].vel_des_, &jointData_[index].kp_,
                                                           &jointData_[index].kd_, &jointData_[index].ff_));
  }
  return true;
}

bool BridgeHW::setupEtherCAT()
{
  // a new EthercatDeviceConfigurator object (path to setup.yaml as constructor argument)
  configurator = std::make_shared<EthercatDeviceConfigurator>(ethercatConfigFile);

  /*
  ** Start all masters.
  ** There is exactly one bus per master which is also started.
  ** All online (i.e. SDO) configuration is done during this call.
  ** The EtherCAT interface is active afterwards, all drives are in Operational
  ** EtherCAT state and PDO communication may begin.
   */
  for(auto & master: configurator->getMasters())
  {
      if(!master->startup())
      {
          std::cerr << "Startup not successful." << std::endl;
          return EXIT_FAILURE;
      }
  }

  // Start the PDO loop in a new thread.
  // ethercat_thread = std::thread(&setupEtherCAT);
  ethercat_thread = std::thread([&]() {
      bool rtSuccess = true;

      for(const auto & master: configurator->getMasters())
      {
          rtSuccess &= master->setRealtimePriority(99);
      }
      std::cout << "Setting RT Priority: " << (rtSuccess? "successful." : "not successful. Check user privileges.") << std::endl;

      // Flag to set the drive state for the elmos on first startup
      bool elmoEnabledAfterStartup = false;
      bool ActuatorPosOffset = true;
      while(!abrt)
      {
          /*
          ** Update each master.
          ** This sends tha last staged commands and reads the latest readings over EtherCAT.
          ** The StandaloneEnforceRate update mode is used.
          ** This means that average update rate will be close to the target rate (if possible).
          */
          for(const auto & master: configurator->getMasters() )
          {
              master->update(ecat_master::UpdateMode::StandaloneEnforceRate); // TODO fix the rate compensation (Elmo reliability problem)!!
          }

          //  Do things with the attached devices.  your lowlevel control input / measurement logic goes here. * Different logic can be implemented for each device.
          for(const auto & slave:configurator->getSlaves())
          {
              // Elmo
              if(configurator->getInfoForSlave(slave).type == EthercatDeviceConfigurator::EthercatSlaveType::Elmo)
              {
                  std::shared_ptr<elmo::Elmo> elmo_slave_ptr = std::dynamic_pointer_cast<elmo::Elmo>(slave);
                  {
                      if(!elmoEnabledAfterStartup)
                      // Set elmos to operation enabled state, do not block the call!
                      elmo_slave_ptr->setDriveStateViaPdo(elmo::DriveState::OperationEnabled, false);
                      int leg_index=0, joint_index=0;
                      
                      auto reading = elmo_slave_ptr->getReading();
                        
                        if(elmo_slave_ptr->getName().find("Left") != std::string::npos)
                        {
                          leg_index = 0;
                        }
                        else if(elmo_slave_ptr->getName().find("Right") != std::string::npos)
                        {
                          leg_index = 1;
                        }

                        if (elmo_slave_ptr->getName().find("HipYaw") != std::string::npos)
                          joint_index = 0;
                        else if (elmo_slave_ptr->getName().find("HipRoll") != std::string::npos)
                          joint_index = 1;
                        else if (elmo_slave_ptr->getName().find("HipPitch") != std::string::npos)
                          joint_index = 2;
                        else if (elmo_slave_ptr->getName().find("Knee") != std::string::npos)
                          joint_index = 3;
                        else if (elmo_slave_ptr->getName().find("Ankle") != std::string::npos)
                          joint_index = 4;

                        // std::cout << "Elmo '" << elmo_slave_ptr->getName() << "': "
                        //                 << "position: " << reading.getActualPosition() << " rad"
                        //                 << " Velocity: " << reading.getActualVelocity() << " rad/s"
                        //                 << " torque:" << reading.getActualTorque() << " A\n";
                        ActuatorData_[leg_index * 5 + joint_index].pos_ = reading.getActualPosition();
                        ActuatorData_[leg_index * 5 + joint_index].vel_ = reading.getActualVelocity();
                        ActuatorData_[leg_index * 5 + joint_index].tau_ = reading.getActualTorque();

                        if(ActuatorPosOffset)
                        {
                          if(ActuatorData_[leg_index * 5 + joint_index].pos_ > 1.3*M_PI)
                          {
                            jointOffset_[leg_index * 5 + joint_index] = -1;
                          }
                          else if(ActuatorData_[leg_index * 5 + joint_index].pos_ < -1.3*M_PI)
                          {
                            jointOffset_[leg_index * 5 + joint_index] = 1;
                          }
                          else
                          {
                            jointOffset_[leg_index * 5 + joint_index] = 0;
                          }
                          
                        }

                      // set commands if we can
                      if(elmo_slave_ptr->lastPdoStateChangeSuccessful() && elmo_slave_ptr->getReading().getDriveState() == elmo::DriveState::OperationEnabled)
                      {
                        elmo::Command command;
                        command.setModeOfOperation(elmo::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                        command.setTargetTorque(0.1);
                        // command.setTargetTorque(ActuatorData_[leg_index * 5 + joint_index].tau_des_);
                        //   std::cout << "leg:" << leg_index
                        //             << " Joint " << joint_index                                    
                        //             << " KpDes: " << ActuatorData_[leg_index * 5 + joint_index].kp_ 
                        //             << " KdDes: " << ActuatorData_[leg_index * 5 + joint_index].kd_
                        //             << " PosDes: " << jointData_[leg_index * 5 + joint_index].pos_des_
                        //             << " Pos: " << jointData_[leg_index * 5 + joint_index].pos_ 
                        //             << " TauDes: " << ActuatorData_[leg_index * 5 + joint_index].tau_des_ << "\n"; 

                        elmo_slave_ptr->stageCommand(command);
                      }
                      else
                      {
                          MELO_WARN_STREAM("Elmoxx '" << elmo_slave_ptr->getName() << "': " << elmo_slave_ptr->getReading().getDriveState());
                          std::cout << "stateChange: " << elmo_slave_ptr->lastPdoStateChangeSuccessful() << "\n";
                          // elmo_slave_ptr->setDriveStateViaPdo(elmo::DriveState::OperationEnabled, false);
                      }
                  }
              }
          }
          elmoEnabledAfterStartup = true;
          ActuatorPosOffset = false;
      }
  });

  // Wait for a few PDO cycles to pass.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  for(auto & slave: configurator->getSlaves())
  {
    std::cout << " " << slave->getName() << ": " << slave->getAddress() << std::endl;
  }
  return true;
}

bool BridgeHW::setupImu()
{
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle(
      "imu_link", "imu_link", imuData_.ori, imuData_.ori_cov, imuData_.angular_vel, imuData_.angular_vel_cov,
      imuData_.linear_acc, imuData_.linear_acc_cov));
  imuData_.ori_cov[0] = 0.0012;
  imuData_.ori_cov[4] = 0.0012;
  imuData_.ori_cov[8] = 0.0012;

  imuData_.angular_vel_cov[0] = 0.0004;
  imuData_.angular_vel_cov[4] = 0.0004;
  imuData_.angular_vel_cov[8] = 0.0004;

  return true;
}

void BridgeHW::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  imuMsg_ = *msg;
}

}  // namespace legged
