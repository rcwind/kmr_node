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
 * @file /kmr_node/src/node/subscriber_callbacks.cpp
 *
 * @brief Subscriber callbacks for kmr node.
 *
 **/

/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include "../../include/kmr_node/kmr_ros.hpp"

/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace kmr
{

/*****************************************************************************
 ** Implementation
 *****************************************************************************/

void KmrRos::subscribeVelocityCommand(const geometry_msgs::TwistConstPtr msg)
{
  if (kmr.isEnabled())
  {
    // For now assuming this is in the robot frame, but probably this
    // should be global frame and require a transform
    //double vx = msg->linear.x;        // in (m/s)
    //double wz = msg->angular.z;       // in (rad/s)
    ROS_DEBUG_STREAM("Kmr : velocity command received [" << msg->linear.x << "],[" << msg->angular.z << "]");
    double v = std::sqrt(pow(msg->linear.x, 2) + pow(msg->linear.y, 2));
    double yaw = 0.0;

    if((msg->linear.x == 0) && (msg->linear.y == 0))
        yaw = 0.0;
    else if((msg->linear.x == 0) && (msg->linear.y > 0))
        yaw = M_PI / 2;
    else if((msg->linear.x == 0) && (msg->linear.y < 0))
        yaw = -M_PI / 2;
    else
        yaw = std::atan2(msg->linear.y, msg->linear.x);

    if(abs(yaw) > M_PI / 2)
        v = -v;

    kmr.setBaseControl(v, msg->angular.z, yaw);
    odometry.resetTimeout();
  }
  return;
}


void KmrRos::subscribeLed1Command(const kobuki_msgs::LedConstPtr msg)
{
  switch( msg->value ) {
  case kobuki_msgs::Led::GREEN:  kmr.setLed(Led1, Green ); break;
  case kobuki_msgs::Led::ORANGE: kmr.setLed(Led1, Orange ); break; 
  case kobuki_msgs::Led::RED:    kmr.setLed(Led1, Red ); break;
  case kobuki_msgs::Led::BLACK:  kmr.setLed(Led1, Black ); break;
  default: ROS_WARN_STREAM("Kmr : led 1 command value invalid."); break;
  }
  return;
}

void KmrRos::subscribeLed2Command(const kobuki_msgs::LedConstPtr msg)
{
  switch( msg->value ) {
  case kobuki_msgs::Led::GREEN:  kmr.setLed(Led2, Green ); break;
  case kobuki_msgs::Led::ORANGE: kmr.setLed(Led2, Orange ); break;
  case kobuki_msgs::Led::RED:    kmr.setLed(Led2, Red ); break;
  case kobuki_msgs::Led::BLACK:  kmr.setLed(Led2, Black ); break;
  default: ROS_WARN_STREAM("Kmr : led 2 command value invalid."); break;
  }
  return;
}

void KmrRos::subscribeDigitalOutputCommand(const kobuki_msgs::DigitalOutputConstPtr msg)
{
  DigitalOutput digital_output;
  for ( unsigned int i = 0; i < 4; ++i ) {
    digital_output.values[i] = msg->values[i];
    digital_output.mask[i] = msg->mask[i];
  }
  kmr.setDigitalOutput(digital_output);
  return;
}

void KmrRos::subscribeExternalPowerCommand(const kobuki_msgs::ExternalPowerConstPtr msg)
{
  // Validate message
  if (!((msg->source == kobuki_msgs::ExternalPower::PWR_3_3V1A) ||
        (msg->source == kobuki_msgs::ExternalPower::PWR_5V1A) ||
        (msg->source == kobuki_msgs::ExternalPower::PWR_12V5A) ||
        (msg->source == kobuki_msgs::ExternalPower::PWR_12V1_5A)))
  {
    ROS_ERROR_STREAM("Kmr : Power source " << (unsigned int)msg->source << " does not exist! [" << name << "].");
    return;
  }
  if (!((msg->state == kobuki_msgs::ExternalPower::OFF) ||
      (msg->state == kobuki_msgs::ExternalPower::ON)))
  {
    ROS_ERROR_STREAM("Kmr : Power source state "
        << (unsigned int)msg->state << " does not exist! [" << name << "].");
    return;
  }

  DigitalOutput digital_output;
  for ( unsigned int i = 0; i < 4; ++i )
  {
    if (i == msg->source)
    {
      if (msg->state)
      {
        digital_output.values[i] = true; // turn source on
        ROS_INFO_STREAM("Kmr : Turning on external power source "
            << (unsigned int)msg->source << ". [" << name << "].");
      }
      else
      {
        digital_output.values[i] = false; // turn source off
        ROS_INFO_STREAM("Kmr : Turning off external power source "
            << (unsigned int)msg->source << ". [" << name << "].");
      }
      digital_output.mask[i] = true; // change source state
    }
    else
    {
      digital_output.values[i] = false; // values doesn't matter here, since mask is set false, what means ignoring
      digital_output.mask[i] = false;
    }
  }
  kmr.setExternalPower(digital_output);
  return;
}

/**
 * @brief Play a predefined sound (single sound or sound sequence)
 */
void KmrRos::subscribeSoundCommand(const kobuki_msgs::SoundConstPtr msg)
{
  if ( msg->value == kobuki_msgs::Sound::ON )
  {
    kmr.playSoundSequence(On);
  }
  else if ( msg->value == kobuki_msgs::Sound::OFF )
  {
    kmr.playSoundSequence(Off);
  }
  else if ( msg->value == kobuki_msgs::Sound::RECHARGE )
  {
    kmr.playSoundSequence(Recharge);
  }
  else if ( msg->value == kobuki_msgs::Sound::BUTTON )
  {
    kmr.playSoundSequence(Button);
  }
  else if ( msg->value == kobuki_msgs::Sound::ERROR )
  {
    kmr.playSoundSequence(Error);
  }
  else if ( msg->value == kobuki_msgs::Sound::CLEANINGSTART )
  {
    kmr.playSoundSequence(CleaningStart);
  }
  else if ( msg->value == kobuki_msgs::Sound::CLEANINGEND )
  {
    kmr.playSoundSequence(CleaningEnd);
  }
  else
  {
    ROS_WARN_STREAM("Kmr : Invalid sound command! There is no sound stored for value '" << msg->value << "'.");
  }
  return;
}

/**
 * @brief Reset the odometry variables.
 */
void KmrRos::subscribeResetOdometry(const std_msgs::EmptyConstPtr /* msg */)
{
  ROS_INFO_STREAM("Kmr : Resetting the odometry. [" << name << "].");
  joint_states.position[0] = 0.0; // wheel_left
  joint_states.velocity[0] = 0.0;
  joint_states.position[1] = 0.0; // wheel_right
  joint_states.velocity[1] = 0.0;
  odometry.resetOdometry();
  kmr.resetOdometry();
  return;
}

void KmrRos::subscribeMotorPower(const kobuki_msgs::MotorPowerConstPtr msg)
{
  if (msg->state == kobuki_msgs::MotorPower::ON)
  {
    ROS_INFO_STREAM("Kmr : Firing up the motors. [" << name << "]");
    kmr.enable();
    odometry.resetTimeout();
  }
  else if (msg->state == kobuki_msgs::MotorPower::OFF)
  {
    kmr.disable();
    ROS_INFO_STREAM("Kmr : Shutting down the motors. [" << name << "]");
    odometry.resetTimeout();
  }
  else
  {
    ROS_ERROR_STREAM("Kmr : Motor power command specifies unknown state '" << (unsigned int)msg->state
                     << "'. [" << name << "]");
  }
}

void KmrRos::subscribeControllerInfoCommand(const kobuki_msgs::ControllerInfoConstPtr msg)
{
  if( msg->p_gain < 0.0f ||  msg->i_gain < 0.0f ||  msg->d_gain < 0.0f) {
    ROS_ERROR_STREAM("Kmr : All controller gains should be positive. [" << name << "]");
    return;
  }
  kmr.setControllerGain(msg->type,
                           static_cast<unsigned int>(msg->p_gain*1000.0f),
                           static_cast<unsigned int>(msg->i_gain*1000.0f),
                           static_cast<unsigned int>(msg->d_gain*1000.0f));
  return;
}

} // namespace kmr
