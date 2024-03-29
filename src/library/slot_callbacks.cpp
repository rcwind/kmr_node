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
 * @file src/node/slot_callbacks.cpp
 *
 * @brief All the slot callbacks for interrupts from the kmr driver.
 *
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "kmr_node/kmr_ros.hpp"
#include <sensor_msgs/PointCloud2.h>

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kmr
{

void KmrRos::processStreamData() {
  publishWheelState();
  publishSensorState();
  publishInertia();
  publishRawInertia();
  publishUltrasonic();
  publishSteering();
}

/*****************************************************************************
** Publish Sensor Stream Workers
*****************************************************************************/

void KmrRos::publishSensorState()
{
  if ( ros::ok() ) {
    CoreSensors::Data data = kmr.getCoreSensorData();
    if((data.vehicle < 1) || (data.vehicle > 7))
      ROS_ERROR_STREAM("unkown vehicle type: " << data.vehicle);

    if (sensor_state_publisher.getNumSubscribers() > 0) {
      kmr_msgs::SensorState state;
      state.header.stamp = ros::Time::now();
      state.time_stamp = data.time_stamp; // firmware time stamp
      state.vehicle = data.vehicle;
      state.bumper = data.bumper;
      state.wheel_drop = data.wheel_drop;
      state.cliff = data.cliff;
      state.left_front_encoder = data.left_front_encoder;
      state.right_front_encoder = data.right_front_encoder;
      state.left_rear_encoder = data.left_rear_encoder;
      state.right_rear_encoder = data.right_rear_encoder;
      state.charger_status = data.charger_status;
      state.charger_current = data.charger_current;
      state.battery = data.battery;

      Cliff::Data cliff_data = kmr.getCliffData();
      state.bottom = cliff_data.bottom;

      GpInput::Data gp_input_data = kmr.getGpInputData();
      state.digital_input = gp_input_data.digital_input;
      for ( unsigned int i = 0; i < gp_input_data.analog_input.size(); ++i ) {
        state.analog_input.push_back(gp_input_data.analog_input[i]);
      }

      sensor_state_publisher.publish(state);
    }
  }
}

void KmrRos::publishWheelState()
{
  // Take latest encoders and gyro data
  ecl::LegacyPose2D<double> pose_update;
  ecl::linear_algebra::Vector3d pose_update_rates;
  kmr.updateOdometry(pose_update, pose_update_rates);
  kmr.getWheelJointStates(
      joint_states.position[0], joint_states.velocity[0],  // left front wheel
      joint_states.position[1], joint_states.velocity[1], // right front wheel
      joint_states.position[2], joint_states.velocity[2], // left rear wheel
      joint_states.position[3], joint_states.velocity[3]); // right rear wheel

  // Update and publish odometry and joint states
  odometry.update(pose_update, pose_update_rates, kmr.getHeading(), kmr.getAngularVelocity());

  if (ros::ok())
  {
    joint_states.header.stamp = ros::Time::now();
    joint_state_publisher.publish(joint_states);
  }
}

void KmrRos::publishInertia()
{
  if (ros::ok())
  {
    if (imu_data_publisher.getNumSubscribers() > 0)
    {
      // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
      sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);

      msg->header.frame_id = "gyro_link";
      msg->header.stamp = ros::Time::now();

      msg->orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, kmr.getHeading());

      // set a non-zero covariance on unused dimensions (pitch and roll); this is a requirement of robot_pose_ekf
      // set yaw covariance as very low, to make it dominate over the odometry heading when combined
      // 1: fill once, as its always the same;  2: using an invented value; cannot we get a realistic estimation?
      msg->orientation_covariance[0] = DBL_MAX;
      msg->orientation_covariance[4] = DBL_MAX;
      msg->orientation_covariance[8] = 0.05;

      // fill angular velocity; we ignore acceleration for now
      msg->angular_velocity.z = kmr.getAngularVelocity();

      // angular velocity covariance; useless by now, but robot_pose_ekf's
      // roadmap claims that it will compute velocities in the future
      msg->angular_velocity_covariance[0] = DBL_MAX;
      msg->angular_velocity_covariance[4] = DBL_MAX;
      msg->angular_velocity_covariance[8] = 0.05;

      imu_data_publisher.publish(msg);
    }
  }
}

void KmrRos::publishRawInertia()
{
  if ( ros::ok() && (raw_imu_data_publisher.getNumSubscribers() > 0) )
  {
    // Publish as shared pointer to leverage the nodelets' zero-copy pub/sub feature
    sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);
    ThreeAxisGyro::Data data = kmr.getRawInertiaData();

    ros::Time now = ros::Time::now();
    ros::Duration interval(0.01); // Time interval between each sensor reading.

    const double digit_to_dps = (2000.0/32768.0); // 0.00875 // digit to deg/s ratio, comes from datasheet of 3d gyro[L3G4200D].
    unsigned int length = data.followed_data_length/3;
    for( unsigned int i=0; i<length; i++) {
      // Each sensor reading has id, that circulate 0 to 255.
      //msg->header.frame_id = std::string("gyro_link_" + boost::lexical_cast<std::string>((unsigned int)data.frame_id+i));
      msg->header.frame_id = "gyro_link";

      // Update rate of 3d gyro sensor is 100 Hz, but robot's update rate is 50 Hz.
      // So, here is some compensation.
      // See also https://github.com/yujinrobot/kmr/issues/216
      msg->header.stamp = now - interval * (length-i-1);

#if 0
      // kobuki
      // Sensing axis of 3d gyro is not match with robot. It is rotated 90 degree counterclockwise about z-axis.
      msg->angular_velocity.x = angles::from_degrees( -digit_to_dps * (short)data.data[i*3+1] );
      msg->angular_velocity.y = angles::from_degrees(  digit_to_dps * (short)data.data[i*3+0] );
      msg->angular_velocity.z = angles::from_degrees(  digit_to_dps * (short)data.data[i*3+2] );
#else
      msg->angular_velocity.x = angles::from_degrees( digit_to_dps * (short)data.data[i*3+0] );
      msg->angular_velocity.y = angles::from_degrees( digit_to_dps * (short)data.data[i*3+1] );
      msg->angular_velocity.z = angles::from_degrees( digit_to_dps * (short)data.data[i*3+2] );
#endif

      raw_imu_data_publisher.publish(msg);
    }
  }
}

void KmrRos::publishSteering()
{
    Steering::Data data = kmr.getSteeringData();
    unsigned int length = data.followed_data_length;
    // raw data
  if ( ros::ok() && (steering_data_publisher.getNumSubscribers() > 0) )
  {
    float steering;
    std_msgs::Float32MultiArrayPtr msg(new std_msgs::Float32MultiArray);
    for (int i = 0; i < length; ++i) 
    {
        steering = data.data[i] / 1000.f; // 0.001deg -> deg
        msg->data.push_back(steering);
    }
    steering_data_publisher.publish(msg);
  }
}
void KmrRos::publishUltrasonic()
{
    Ultrasonic::Data data = kmr.getUltrasonicData();
    unsigned int length = data.followed_data_length;
    // raw data
  if ( ros::ok() && (raw_ultrasonic_data_publisher.getNumSubscribers() > 0) )
  {
    float distance;
    std_msgs::Float32MultiArrayPtr msg(new std_msgs::Float32MultiArray);
    for (int i = 0; i < length; ++i) 
    {
        distance = data.data[i] / 1000.f; // mm -> m
        msg->data.push_back(distance);
    }
    raw_ultrasonic_data_publisher.publish(msg);
  }
  // point cloud
  if (ros::ok() && ultrasonic_cloud_publisher.getNumSubscribers() > 0)
  {
      std::string base_link_frame;
      double pointcloud_height;
      std::vector<double> angle, ultrasonic_position;
      node_handle->param("pointcloud_height", pointcloud_height, 0.04);  // kmr_node base.yaml文件定义
      node_handle->param("pointcloud_angle", angle, std::vector<double>()); 
      node_handle->param("ultrasonic_position", ultrasonic_position, std::vector<double>()); 
      node_handle->param<std::string>("base_frame", base_link_frame, "/base_link");

      for(int i = 0; i < angle.size(); i++)
          angle[i] = angle[i] / 180.f * 3.1415926;

      sensor_msgs::PointCloud2 pointcloud;
      pointcloud.header.stamp = ros::Time::now();
      pointcloud.header.frame_id = base_link_frame;
      pointcloud.width  = length;
      pointcloud.height = 1;
      pointcloud.fields.resize(3);

      // Set x/y/z as the only fields
      pointcloud.fields[0].name = "x";
      pointcloud.fields[1].name = "y";
      pointcloud.fields[2].name = "z";

      int offset = 0;
      // All offsets are *4, as all field data types are float32
      for (size_t d = 0; d < pointcloud.fields.size(); ++d, offset += 4)
      {
          pointcloud.fields[d].count    = 1;
          pointcloud.fields[d].offset   = offset;
          pointcloud.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
      }

      pointcloud.point_step = offset;
      pointcloud.row_step   = pointcloud.point_step * pointcloud.width;

      pointcloud.data.resize(length * pointcloud.point_step);
      pointcloud.is_bigendian = false;
      pointcloud.is_dense     = true;

      for (int i = 0; i < length; ++i) 
      {
          float x, y, distance;
          distance = data.data[i] / 1000.f; // mm -> m
          // 超声波模块的排布顺序和数据顺序都会影响计算xy的值，请根据实际情况来改
          x = sin(angle[i]) * (distance + ultrasonic_position[i]);
          y = cos(angle[i]) * (distance + ultrasonic_position[i]);
          memcpy(&pointcloud.data[i * pointcloud.point_step + pointcloud.fields[0].offset], &x, sizeof(float));//x
          memcpy(&pointcloud.data[i * pointcloud.point_step + pointcloud.fields[1].offset], &y, sizeof(float));//y
          memcpy(&pointcloud.data[i * pointcloud.point_step + pointcloud.fields[2].offset], &pointcloud_height, sizeof(float));//z
      }

      ultrasonic_cloud_publisher.publish(pointcloud);
  }
}

/*****************************************************************************
** Non Default Stream Packets
*****************************************************************************/
/**
 * @brief Publish fw, hw, sw version information.
 *
 * The driver will only gather this data when initialising so it is
 * important that this publisher is latched.
 */
void KmrRos::publishVersionInfo(const VersionInfo &version_info)
{
  if (ros::ok())
  {
    kmr_msgs::VersionInfoPtr msg(new kmr_msgs::VersionInfo);

    msg->firmware = VersionInfo::toString(version_info.firmware);
    msg->hardware = VersionInfo::toString(version_info.hardware);
    msg->software = VersionInfo::getSoftwareVersion();

    msg->udid.resize(3);
    msg->udid[0] = version_info.udid0;
    msg->udid[1] = version_info.udid1;
    msg->udid[2] = version_info.udid2;

    // Set available features mask depending on firmware and driver versions
    if (version_info.firmware > 65536)  // 1.0.0
    {
      msg->features |= kmr_msgs::VersionInfo::SMOOTH_MOVE_START;
      msg->features |= kmr_msgs::VersionInfo::GYROSCOPE_3D_DATA;
    }
    if (version_info.firmware > 65792)  // 1.1.0
    {
      // msg->features |= kmr_msgs::VersionInfo::SOMETHING_JINCHA_FANCY;
    }
    // if (msg->firmware > ...

    version_info_publisher.publish(msg);
  }
}

/*****************************************************************************
** Events
*****************************************************************************/

void KmrRos::publishBumperEvent(const BumperEvent &event)
{
  if (ros::ok())
  {
    kmr_msgs::BumperEventPtr msg(new kmr_msgs::BumperEvent);
    switch(event.state) {
      case(BumperEvent::Pressed)  : { msg->state = kmr_msgs::BumperEvent::PRESSED;  break; }
      case(BumperEvent::Released) : { msg->state = kmr_msgs::BumperEvent::RELEASED; break; }
      default: break;
    }
    switch(event.bumper) {
      case(BumperEvent::Left)   : { msg->bumper = kmr_msgs::BumperEvent::LEFT;   break; }
      case(BumperEvent::Center) : { msg->bumper = kmr_msgs::BumperEvent::CENTER; break; }
      case(BumperEvent::Right)  : { msg->bumper = kmr_msgs::BumperEvent::RIGHT;  break; }
      default: break;
    }
    bumper_event_publisher.publish(msg);
  }
}

void KmrRos::publishCliffEvent(const CliffEvent &event)
{
  if (ros::ok())
  {
    kmr_msgs::CliffEventPtr msg(new kmr_msgs::CliffEvent);
    switch(event.state) {
      case(CliffEvent::Floor) : { msg->state = kmr_msgs::CliffEvent::FLOOR; break; }
      case(CliffEvent::Cliff) : { msg->state = kmr_msgs::CliffEvent::CLIFF; break; }
      default: break;
    }
    switch(event.sensor) {
      case(CliffEvent::Left)   : { msg->sensor = kmr_msgs::CliffEvent::LEFT;   break; }
      case(CliffEvent::Center) : { msg->sensor = kmr_msgs::CliffEvent::CENTER; break; }
      case(CliffEvent::Right)  : { msg->sensor = kmr_msgs::CliffEvent::RIGHT;  break; }
      default: break;
    }
    msg->bottom = event.bottom;
    cliff_event_publisher.publish(msg);
  }
}

void KmrRos::publishWheelEvent(const WheelEvent &event)
{
  if (ros::ok())
  {
    kmr_msgs::WheelDropEventPtr msg(new kmr_msgs::WheelDropEvent);
    switch(event.state) {
      case(WheelEvent::Dropped) : { msg->state = kmr_msgs::WheelDropEvent::DROPPED; break; }
      case(WheelEvent::Raised)  : { msg->state = kmr_msgs::WheelDropEvent::RAISED;  break; }
      default: break;
    }
    switch(event.wheel) {
      case(WheelEvent::Left)  : { msg->wheel = kmr_msgs::WheelDropEvent::LEFT;  break; }
      case(WheelEvent::Right) : { msg->wheel = kmr_msgs::WheelDropEvent::RIGHT; break; }
      default: break;
    }
    wheel_event_publisher.publish(msg);
  }
}

void KmrRos::publishPowerEvent(const PowerEvent &event)
{
  if (ros::ok())
  {
    kmr_msgs::PowerSystemEventPtr msg(new kmr_msgs::PowerSystemEvent);
    switch(event.event) {
      case(PowerEvent::Unplugged) :
        { msg->event = kmr_msgs::PowerSystemEvent::UNPLUGGED; break; }
      case(PowerEvent::PluggedToAdapter) :
        { msg->event = kmr_msgs::PowerSystemEvent::PLUGGED_TO_ADAPTER;  break; }
      case(PowerEvent::PluggedToDockbase) :
        { msg->event = kmr_msgs::PowerSystemEvent::PLUGGED_TO_DOCKBASE; break; }
      case(PowerEvent::ChargeCompleted)  :
        { msg->event = kmr_msgs::PowerSystemEvent::CHARGE_COMPLETED;  break; }
      case(PowerEvent::BatteryLow) :
        { msg->event = kmr_msgs::PowerSystemEvent::BATTERY_LOW; break; }
      case(PowerEvent::BatteryCritical) :
        { msg->event = kmr_msgs::PowerSystemEvent::BATTERY_CRITICAL;  break; }
      default: break;
    }
    power_event_publisher.publish(msg);
  }
}

void KmrRos::publishInputEvent(const InputEvent &event)
{
  if (ros::ok())
  {
    kmr_msgs::DigitalInputEventPtr msg(new kmr_msgs::DigitalInputEvent);
    for (unsigned int i = 0; i < msg->values.size(); i++)
      msg->values[i] = event.values[i];
    input_event_publisher.publish(msg);
  }
}

void KmrRos::publishRobotEvent(const RobotEvent &event)
{
  if (ros::ok())
  {
    kmr_msgs::RobotStateEventPtr msg(new kmr_msgs::RobotStateEvent);
    switch(event.state) {
      case(RobotEvent::Online)  : { msg->state = kmr_msgs::RobotStateEvent::ONLINE;  break; }
      case(RobotEvent::Offline) : { msg->state = kmr_msgs::RobotStateEvent::OFFLINE; break; }
      default: break;
    }

    robot_event_publisher.publish(msg);
  }
}

/**
 * @brief Prints the raw data stream to a publisher.
 *
 * This is a lazy publisher, it only publishes if someone is listening. It publishes the
 * hex byte values of the raw data commands. Useful for debugging command to protocol
 * byte packets to the firmware.
 *
 * The signal which calls this
 * function is sending a copy of the buffer (don't worry about mutexes). Be ideal if we used
 * const PacketFinder::BufferType here, but haven't updated PushPop to work with consts yet.
 *
 * @param buffer
 */
void KmrRos::publishRawDataCommand(Command::Buffer &buffer)
{
  if ( raw_data_command_publisher.getNumSubscribers() > 0 ) { // do not do string processing if there is no-one listening.
    std::ostringstream ostream;
    Command::Buffer::Formatter format;
    ostream << format(buffer); // convert to an easily readable hex string.
    std_msgs::String s;
    s.data = ostream.str();
    if (ros::ok())
    {
      raw_data_command_publisher.publish(s);
    }
  }
}
/**
 * @brief Prints the raw data stream to a publisher.
 *
 * This is a lazy publisher, it only publishes if someone is listening. It publishes the
 * hex byte values of the raw data (incoming) stream. Useful for checking when bytes get
 * mangled.
 *
 * The signal which calls this
 * function is sending a copy of the buffer (don't worry about mutexes). Be ideal if we used
 * const PacketFinder::BufferType here, but haven't updated PushPop to work with consts yet.
 *
 * @param buffer
 */
void KmrRos::publishRawDataStream(PacketFinder::BufferType &buffer)
{
  if ( raw_data_stream_publisher.getNumSubscribers() > 0 ) { // do not do string processing if there is no-one listening.
    /*std::cout << "size: [" << buffer.size() << "], asize: [" << buffer.asize() << "]" << std::endl;
    std::cout << "leader: " << buffer.leader << ", follower: " << buffer.follower  << std::endl;
    {
      std::ostringstream ostream;
      PacketFinder::BufferType::Formatter format;
      ostream << format(buffer); // convert to an easily readable hex string.
      //std::cout << ostream.str() << std::endl;
      std_msgs::String s;
      s.data = ostream.str();
      if (ros::ok())
      {
        raw_data_stream_publisher.publish(s);
      }
    }*/
    {
      std::ostringstream ostream;
      ostream << "{ " ;
      ostream << std::setfill('0') << std::uppercase;
      for (unsigned int i=0; i < buffer.size(); i++)
          ostream << std::hex << std::setw(2) << static_cast<unsigned int>(buffer[i]) << " " << std::dec;
      ostream << "}";
      //std::cout << ostream.str() << std::endl;
      std_msgs::StringPtr msg(new std_msgs::String);
      msg->data = ostream.str();
      if (ros::ok())
      {
        raw_data_stream_publisher.publish(msg);
      }
    }
  }
}

void KmrRos::publishRawControlCommand(const std::vector<short> &velocity_commands)
{
  if ( raw_control_command_publisher.getNumSubscribers() > 0 ) {
    std_msgs::Int16MultiArrayPtr msg(new std_msgs::Int16MultiArray);
    msg->data = velocity_commands;
    if (ros::ok())
    {
      raw_control_command_publisher.publish(msg);
    }
  }
  return;
}

} // namespace kmr
