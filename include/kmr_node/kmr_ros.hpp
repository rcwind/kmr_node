/*
 * Copyright (c) 2012, Yujin Robot.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Yujin Robot nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file /kmr_node/include/kmr_node/kmr_ros.hpp
 *
 * @brief Wraps the kmr driver in a ROS-specific library
 *
 **/

/*****************************************************************************
 ** Ifdefs
 *****************************************************************************/

#ifndef KMR_ROS_HPP_
#define KMR_ROS_HPP_

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <angles/angles.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <ecl/sigslots.hpp>
#include <kmr_msgs/ButtonEvent.h>
#include <kmr_msgs/BumperEvent.h>
#include <kmr_msgs/CliffEvent.h>
#include <kmr_msgs/DigitalOutput.h>
#include <kmr_msgs/DigitalInputEvent.h>
#include <kmr_msgs/ExternalPower.h>
#include <kmr_msgs/Led.h>
#include <kmr_msgs/MotorPower.h>
#include <kmr_msgs/PowerSystemEvent.h>
#include <kmr_msgs/RobotStateEvent.h>
#include <kmr_msgs/SensorState.h>
#include <kmr_msgs/Sound.h>
#include <kmr_msgs/VersionInfo.h>
#include <kmr_msgs/WheelDropEvent.h>
#include <kmr_driver/kmr.hpp>
#include "diagnostics.hpp"
#include "odometry.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kmr
{
class KmrRos
{
public:
  KmrRos(std::string& node_name);
  ~KmrRos();
  bool init(ros::NodeHandle& nh, ros::NodeHandle& nh_pub);
  bool update();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  /*********************
   ** Variables
   **********************/
  ros::NodeHandle *node_handle;
  std::string name; // name of the ROS node
  Kmr kmr;
  sensor_msgs::JointState joint_states;
  Odometry odometry;
  bool cmd_vel_timed_out_; // stops warning spam when cmd_vel flags as timed out more than once in a row
  bool serial_timed_out_; // stops warning spam when serial connection timed out more than once in a row

  /*********************
   ** Ros Comms
   **********************/
  ros::Publisher version_info_publisher;
  ros::Publisher imu_data_publisher, sensor_state_publisher, joint_state_publisher, raw_imu_data_publisher, raw_ultrasonic_data_publisher, ultrasonic_cloud_publisher;
  ros::Publisher button_event_publisher, input_event_publisher, robot_event_publisher;
  ros::Publisher bumper_event_publisher, cliff_event_publisher, wheel_event_publisher, power_event_publisher;
  ros::Publisher raw_data_command_publisher, raw_data_stream_publisher, raw_control_command_publisher;

  ros::Subscriber velocity_command_subscriber, digital_output_command_subscriber, external_power_command_subscriber;
  ros::Subscriber dock_command_subscriber;
  ros::Subscriber mag_tracker_subscriber;
  ros::Subscriber relay_control_subscriber;
  ros::Subscriber led1_command_subscriber, led2_command_subscriber, sound_command_subscriber;
  ros::Subscriber motor_power_subscriber, reset_odometry_subscriber;

  void advertiseTopics(ros::NodeHandle& nh);
  void subscribeTopics(ros::NodeHandle& nh);

  /*********************
  ** Ros Callbacks
  **********************/
  void subscribeVelocityCommand(const geometry_msgs::TwistConstPtr);
  void subscribeLed1Command(const kmr_msgs::LedConstPtr);
  void subscribeLed2Command(const kmr_msgs::LedConstPtr);
  void subscribeDigitalOutputCommand(const kmr_msgs::DigitalOutputConstPtr);
  void subscribeExternalPowerCommand(const kmr_msgs::ExternalPowerConstPtr);
  void subscribeResetOdometry(const std_msgs::EmptyConstPtr);
  void subscribeSoundCommand(const kmr_msgs::SoundConstPtr);
  void subscribeMotorPower(const kmr_msgs::MotorPowerConstPtr msg);
  void subscribeDockCommand(const std_msgs::UInt8 msg);
  void subscribeMagTracker(const std_msgs::Int32 msg);
  void subscribeRelayControl(const std_msgs::Int32 msg);

  /*********************
   ** SigSlots
   **********************/
  ecl::Slot<const VersionInfo&> slot_version_info;
  ecl::Slot<> slot_stream_data;
  ecl::Slot<const ButtonEvent&> slot_button_event;
  ecl::Slot<const BumperEvent&> slot_bumper_event;
  ecl::Slot<const CliffEvent&>  slot_cliff_event;
  ecl::Slot<const WheelEvent&>  slot_wheel_event;
  ecl::Slot<const PowerEvent&>  slot_power_event;
  ecl::Slot<const InputEvent&>  slot_input_event;
  ecl::Slot<const RobotEvent&>  slot_robot_event;
  ecl::Slot<const std::string&> slot_debug, slot_info, slot_warn, slot_error;
  ecl::Slot<const std::vector<std::string>&> slot_named;
  ecl::Slot<Command::Buffer&> slot_raw_data_command;
  ecl::Slot<PacketFinder::BufferType&> slot_raw_data_stream;
  ecl::Slot<const std::vector<short>&> slot_raw_control_command;

  /*********************
   ** Slot Callbacks
   **********************/
  void processStreamData();
  void publishWheelState();
  void publishInertia();
  void publishRawInertia();
  void publishUltrasonic();
  void publishSensorState();
  void publishDockIRData();
  void publishVersionInfo(const VersionInfo &version_info);
  void publishButtonEvent(const ButtonEvent &event);
  void publishBumperEvent(const BumperEvent &event);
  void publishCliffEvent(const CliffEvent &event);
  void publishWheelEvent(const WheelEvent &event);
  void publishPowerEvent(const PowerEvent &event);
  void publishInputEvent(const InputEvent &event);
  void publishRobotEvent(const RobotEvent &event);


  // debugging
  void rosDebug(const std::string &msg) { ROS_DEBUG_STREAM("Kmr : " << msg); }
  void rosInfo(const std::string &msg) { ROS_INFO_STREAM("Kmr : " << msg); }
  void rosWarn(const std::string &msg) { ROS_WARN_STREAM("Kmr : " << msg); }
  void rosError(const std::string &msg) { ROS_ERROR_STREAM("Kmr : " << msg); }
  void rosNamed(const std::vector<std::string> &msgs) {
    if (msgs.size()==0) return;
    if (msgs.size()==1) { ROS_INFO_STREAM("Kmr : " << msgs[0]); }
    if (msgs.size()==2) {
      if      (msgs[0] == "debug") { ROS_DEBUG_STREAM("Kmr : " << msgs[1]); }
      else if (msgs[0] == "info" ) { ROS_INFO_STREAM ("Kmr : " << msgs[1]); }
      else if (msgs[0] == "warn" ) { ROS_WARN_STREAM ("Kmr : " << msgs[1]); }
      else if (msgs[0] == "error") { ROS_ERROR_STREAM("Kmr : " << msgs[1]); }
      else if (msgs[0] == "fatal") { ROS_FATAL_STREAM("Kmr : " << msgs[1]); }
    }
    if (msgs.size()==3) {
      if      (msgs[0] == "debug") { ROS_DEBUG_STREAM_NAMED(msgs[1], "Kmr : " << msgs[2]); }
      else if (msgs[0] == "info" ) { ROS_INFO_STREAM_NAMED (msgs[1], "Kmr : " << msgs[2]); }
      else if (msgs[0] == "warn" ) { ROS_WARN_STREAM_NAMED (msgs[1], "Kmr : " << msgs[2]); }
      else if (msgs[0] == "error") { ROS_ERROR_STREAM_NAMED(msgs[1], "Kmr : " << msgs[2]); }
      else if (msgs[0] == "fatal") { ROS_FATAL_STREAM_NAMED(msgs[1], "Kmr : " << msgs[2]); }
    }
  }

  void publishRawDataCommand(Command::Buffer &buffer);
  void publishRawDataStream(PacketFinder::BufferType &buffer);
  void publishRawControlCommand(const std::vector<short> &velocity_commands);

  /*********************
  ** Diagnostics
  **********************/
  diagnostic_updater::Updater updater;
  BatteryTask     battery_diagnostics;
  WatchdogTask   watchdog_diagnostics;
  CliffSensorTask   cliff_diagnostics;
  WallSensorTask   bumper_diagnostics;
  WheelDropTask     wheel_diagnostics;
  MotorStateTask    state_diagnostics;
  GyroSensorTask     gyro_diagnostics;
  DigitalInputTask dinput_diagnostics;
  AnalogInputTask  ainput_diagnostics;
};

} // namespace kmr

#endif /* KMR_ROS_HPP_ */
