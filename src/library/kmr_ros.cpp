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
 * @file /kmr_node/src/node/kmr_node.cpp
 *
 * @brief Implementation for the ros kmr node wrapper.
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <float.h>
#include <tf/tf.h>
#include <ecl/streams/string_stream.hpp>
#include <kmr_msgs/VersionInfo.h>
#include "kmr_node/kmr_ros.hpp"
#include <sensor_msgs/PointCloud2.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kmr
{

/*****************************************************************************
 ** Implementation [KmrRos]
 *****************************************************************************/

/**
 * @brief Default constructor.
 *
 * Make sure you call the init() method to fully define this node.
 */
KmrRos::KmrRos(std::string& node_name) :
    name(node_name), cmd_vel_timed_out_(false), serial_timed_out_(false),
    slot_version_info(&KmrRos::publishVersionInfo, *this),
    slot_stream_data(&KmrRos::processStreamData, *this),
    slot_button_event(&KmrRos::publishButtonEvent, *this),
    slot_bumper_event(&KmrRos::publishBumperEvent, *this),
    slot_cliff_event(&KmrRos::publishCliffEvent, *this),
    slot_wheel_event(&KmrRos::publishWheelEvent, *this),
    slot_power_event(&KmrRos::publishPowerEvent, *this),
    slot_input_event(&KmrRos::publishInputEvent, *this),
    slot_robot_event(&KmrRos::publishRobotEvent, *this),
    slot_debug(&KmrRos::rosDebug, *this),
    slot_info(&KmrRos::rosInfo, *this),
    slot_warn(&KmrRos::rosWarn, *this),
    slot_error(&KmrRos::rosError, *this),
    slot_named(&KmrRos::rosNamed, *this),
    slot_raw_data_command(&KmrRos::publishRawDataCommand, *this),
    slot_raw_data_stream(&KmrRos::publishRawDataStream, *this),
    slot_raw_control_command(&KmrRos::publishRawControlCommand, *this)
{
  updater.setHardwareID("Kmr");
  updater.add(battery_diagnostics);
  updater.add(watchdog_diagnostics);
  updater.add(bumper_diagnostics);
  updater.add(cliff_diagnostics);
  updater.add(wheel_diagnostics);
  updater.add(state_diagnostics);
  updater.add(gyro_diagnostics);
  updater.add(dinput_diagnostics);
  updater.add(ainput_diagnostics);
}

/**
 * This will wait some time while kmr internally closes its threads and destructs
 * itself.
 */
KmrRos::~KmrRos()
{
  ROS_INFO_STREAM("Kmr : waiting for kmr thread to finish [" << name << "].");
}

bool KmrRos::init(ros::NodeHandle& nh, ros::NodeHandle& nh_pub)
{
  node_handle = &nh;
  /*********************
   ** Communications
   **********************/
  advertiseTopics(nh);
  subscribeTopics(nh);

  /*********************
   ** Slots
   **********************/
  slot_stream_data.connect(name + std::string("/stream_data"));
  slot_version_info.connect(name + std::string("/version_info"));
  slot_button_event.connect(name + std::string("/button_event"));
  slot_bumper_event.connect(name + std::string("/bumper_event"));
  slot_cliff_event.connect(name + std::string("/cliff_event"));
  slot_wheel_event.connect(name + std::string("/wheel_event"));
  slot_power_event.connect(name + std::string("/power_event"));
  slot_input_event.connect(name + std::string("/input_event"));
  slot_robot_event.connect(name + std::string("/robot_event"));
  slot_debug.connect(name + std::string("/ros_debug"));
  slot_info.connect(name + std::string("/ros_info"));
  slot_warn.connect(name + std::string("/ros_warn"));
  slot_error.connect(name + std::string("/ros_error"));
  slot_named.connect(name + std::string("/ros_named"));
  slot_raw_data_command.connect(name + std::string("/raw_data_command"));
  slot_raw_data_stream.connect(name + std::string("/raw_data_stream"));
  slot_raw_control_command.connect(name + std::string("/raw_control_command"));

  /*********************
   ** Driver Parameters
   **********************/
  Parameters parameters;

  nh.param("acceleration_limiter", parameters.enable_acceleration_limiter, false);
  nh.param("battery_capacity", parameters.battery_capacity, Battery::capacity);
  nh.param("battery_low", parameters.battery_low, Battery::low);
  nh.param("battery_dangerous", parameters.battery_dangerous, Battery::dangerous);

  parameters.sigslots_namespace = name; // name is automatically picked up by device_nodelet parent.
  if (!nh.getParam("device_port", parameters.device_port))
  {
    ROS_ERROR_STREAM("Kmr : no device port given on the parameter server (e.g. /dev/ttyUSB0)[" << name << "].");
    return false;
  }

  /*********************
   ** Joint States
   **********************/
  std::string robot_description, wheel_left_joint_name, wheel_right_joint_name;

  nh.param("wheel_left_joint_name", wheel_left_joint_name, std::string("wheel_left_joint"));
  nh.param("wheel_right_joint_name", wheel_right_joint_name, std::string("wheel_right_joint"));

  // minimalistic check: are joint names present on robot description file?
  if (!nh_pub.getParam("robot_description", robot_description))
  {
    ROS_WARN("Kmr : no robot description given on the parameter server");
  }
  else
  {
    if (robot_description.find(wheel_left_joint_name) == std::string::npos) {
      ROS_WARN("Kmr : joint name %s not found on robot description", wheel_left_joint_name.c_str());
    }

    if (robot_description.find(wheel_right_joint_name) == std::string::npos) {
      ROS_WARN("Kmr : joint name %s not found on robot description", wheel_right_joint_name.c_str());
    }
  }
  joint_states.name.push_back(wheel_left_joint_name);
  joint_states.name.push_back(wheel_right_joint_name);
  joint_states.position.resize(2,0.0);
  joint_states.velocity.resize(2,0.0);
  joint_states.effort.resize(2,0.0);

  /*********************
   ** Validation
   **********************/
  if (!parameters.validate())
  {
    ROS_ERROR_STREAM("Kmr : parameter configuration failed [" << name << "].");
    ROS_ERROR_STREAM("Kmr : " << parameters.error_msg << "[" << name << "]");
    return false;
  }
  else
  {
    if (parameters.simulation)
    {
      ROS_INFO("Kmr : driver going into loopback (simulation) mode.");
    }
    else
    {
      ROS_INFO_STREAM("Kmr : configured for connection on device_port "
                      << parameters.device_port << " [" << name << "].");
      ROS_INFO_STREAM("Kmr : driver running in normal (non-simulation) mode" << " [" << name << "].");
    }
  }

  odometry.init(nh, name);

  /*********************
   ** Driver Init
   **********************/
  try
  {
    kmr.init(parameters);
    ros::Duration(0.25).sleep(); // wait for some data to come in.
    if ( !kmr.isAlive() ) {
      ROS_WARN_STREAM("Kmr : no data stream, is kmr turned on?");
      // don't need to return false here - simply turning kmr on while spin()'ing should resurrect the situation.
    }
    kmr.enable();
  }
  catch (const ecl::StandardException &e)
  {
    switch (e.flag())
    {
      case (ecl::OpenError):
      {
        ROS_ERROR_STREAM("Kmr : could not open connection [" << parameters.device_port << "][" << name << "].");
        break;
      }
      default:
      {
        ROS_ERROR_STREAM("Kmr : initialisation failed [" << name << "].");
        ROS_DEBUG_STREAM(e.what());
        break;
      }
    }
    return false;
  }
  // kmr.printSigSlotConnections();
  return true;
}
/**
 * This is a worker function that runs in a background thread initiated by
 * the nodelet. It gathers diagnostics information from the kmr driver,
 * and broadcasts the results to the rest of the ros ecosystem.
 *
 * Note that the actual driver data is collected via the slot callbacks in this class.
 *
 * @return Bool : true/false if successfully updated or not (kmr driver shutdown).
 */
bool KmrRos::update()
{
  if ( kmr.isShutdown() )
  {
    ROS_ERROR_STREAM("Kmr : Driver has been shutdown. Stopping update loop. [" << name << "].");
    return false;
  }

  if ( (kmr.isEnabled() == true) && odometry.commandTimeout())
  {
    if ( !cmd_vel_timed_out_ )
    {
      kmr.setBaseControl(0, 0, 0);
      cmd_vel_timed_out_ = true;
      ROS_WARN("Kmr : Incoming velocity commands not received for more than %.2f seconds -> zero'ing velocity commands", odometry.timeout().toSec());
    }
  }
  else
  {
    cmd_vel_timed_out_ = false;
  }

  bool is_alive = kmr.isAlive();
  if ( watchdog_diagnostics.isAlive() && !is_alive )
  {
    if ( !serial_timed_out_ )
    {
      ROS_ERROR_STREAM("Kmr : Timed out while waiting for serial data stream [" << name << "].");
      serial_timed_out_ = true;
    }
    else
    {
      serial_timed_out_ = false;
    }
  }

  watchdog_diagnostics.update(is_alive);
  battery_diagnostics.update(kmr.batteryStatus());
  cliff_diagnostics.update(kmr.getCoreSensorData().cliff, kmr.getCliffData());
  bumper_diagnostics.update(kmr.getCoreSensorData().bumper);
  wheel_diagnostics.update(kmr.getCoreSensorData().wheel_drop);
  state_diagnostics.update(kmr.isEnabled());
  gyro_diagnostics.update(kmr.getInertiaData().angle);
  dinput_diagnostics.update(kmr.getGpInputData().digital_input);
  ainput_diagnostics.update(kmr.getGpInputData().analog_input);
  updater.update();

  return true;
}

/**
 * Two groups of publishers, one required by turtlebot, the other for
 * kmr esoterics.
 */
void KmrRos::advertiseTopics(ros::NodeHandle& nh)
{
  /*********************
  ** Turtlebot Required
  **********************/
  joint_state_publisher = nh.advertise <sensor_msgs::JointState>("joint_states",100);

  /*********************
  ** Kmr Esoterics
  **********************/
  version_info_publisher = nh.advertise < kmr_msgs::VersionInfo > ("version_info",  100, true); // latched publisher
  button_event_publisher = nh.advertise < kmr_msgs::ButtonEvent > ("events/button", 100);
  bumper_event_publisher = nh.advertise < kmr_msgs::BumperEvent > ("events/bumper", 100);
  cliff_event_publisher  = nh.advertise < kmr_msgs::CliffEvent >  ("events/cliff",  100);
  wheel_event_publisher  = nh.advertise < kmr_msgs::WheelDropEvent > ("events/wheel_drop", 100);
  power_event_publisher  = nh.advertise < kmr_msgs::PowerSystemEvent > ("events/power_system", 100);
  input_event_publisher  = nh.advertise < kmr_msgs::DigitalInputEvent > ("events/digital_input", 100);
  robot_event_publisher  = nh.advertise < kmr_msgs::RobotStateEvent > ("events/robot_state", 100, true); // also latched
  sensor_state_publisher = nh.advertise < kmr_msgs::SensorState > ("sensors/core", 100);
  imu_data_publisher = nh.advertise < sensor_msgs::Imu > ("sensors/imu_data", 100);
  raw_imu_data_publisher = nh.advertise < sensor_msgs::Imu > ("sensors/imu_data_raw", 100);
  raw_ultrasonic_data_publisher = nh.advertise < std_msgs::Float32MultiArray> ("sensors/ultrasonic_data_raw", 100);
  ultrasonic_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2> ("sensors/ultrasonic_pointcloud", 100);
  raw_data_command_publisher = nh.advertise< std_msgs::String > ("debug/raw_data_command", 100);
  raw_data_stream_publisher = nh.advertise< std_msgs::String > ("debug/raw_data_stream", 100);
  raw_control_command_publisher = nh.advertise< std_msgs::Int16MultiArray > ("debug/raw_control_command", 100);
}

/**
 * Two groups of subscribers, one required by turtlebot, the other for
 * kmr esoterics.
 */
void KmrRos::subscribeTopics(ros::NodeHandle& nh)
{
  velocity_command_subscriber = nh.subscribe(std::string("commands/velocity"), 10, &KmrRos::subscribeVelocityCommand, this);
  led1_command_subscriber =  nh.subscribe(std::string("commands/led1"), 10, &KmrRos::subscribeLed1Command, this);
  led2_command_subscriber =  nh.subscribe(std::string("commands/led2"), 10, &KmrRos::subscribeLed2Command, this);
  digital_output_command_subscriber =  nh.subscribe(std::string("commands/digital_output"), 10, &KmrRos::subscribeDigitalOutputCommand, this);
  external_power_command_subscriber =  nh.subscribe(std::string("commands/external_power"), 10, &KmrRos::subscribeExternalPowerCommand, this);
  sound_command_subscriber =  nh.subscribe(std::string("commands/sound"), 10, &KmrRos::subscribeSoundCommand, this);
  reset_odometry_subscriber = nh.subscribe("commands/reset_odometry", 10, &KmrRos::subscribeResetOdometry, this);
  motor_power_subscriber = nh.subscribe("commands/motor_power", 10, &KmrRos::subscribeMotorPower, this);
  dock_command_subscriber =  nh.subscribe(std::string("commands/dock"), 10, &KmrRos::subscribeDockCommand, this);
  mag_tracker_subscriber =  nh.subscribe(std::string("/mag/action"), 10, &KmrRos::subscribeMagTracker, this);
  relay_control_subscriber =  nh.subscribe(std::string("/io/relay1"), 10, &KmrRos::subscribeRelayControl, this);
}


} // namespace kmr

