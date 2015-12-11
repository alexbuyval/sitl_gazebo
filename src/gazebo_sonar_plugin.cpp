//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include "gazebo_sonar_plugin.h"
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <limits>

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdio.h>

#include <boost/bind.hpp>


namespace gazebo {

GazeboSonarPlugin::GazeboSonarPlugin()
    : node_handle_(0) {
      this->seed = 0;
    }

GazeboSonarPlugin::~GazeboSonarPlugin() {
  event::Events::DisconnectWorldUpdateBegin(updateConnection);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboSonarPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {

  // Get then name of the parent sensor
  sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
  if (!sensor_)
  {
    gzthrow("GazeboSonarPlugin requires a Ray Sensor as its parent");
    return;
  }
  // Get the world name.
  std::string worldName = sensor_->GetWorldName();
  world = physics::get_world(worldName);

  // default parameters
  namespace_.clear();
  topic_ = "sonar";
  frame_id_ = "/sonar_link";

  // load parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->Get<std::string>();

  if (_sdf->HasElement("topicName"))
    topic_ = _sdf->GetElement("topicName")->Get<std::string>();
  
  if (_sdf->HasElement("gaussianNoise"))
  {
    gaussian_noise_ = _sdf->Get<double>("gaussianNoise");
  }
  else
  {
    gaussian_noise_ = 0;
  }
    

  //range_.header.frame_id = frame_id_;
  //range_.radiation_type = sensor_msgs::Range::ULTRASOUND;
  //range_.field_of_view = std::min(fabs((sensor_->GetAngleMax() - sensor_->GetAngleMin()).Radian()), fabs((sensor_->GetVerticalAngleMax() - sensor_->GetVerticalAngleMin()).Radian()));
  range_message_.set_range_max(sensor_->GetRangeMax());
  range_message_.set_range_min(sensor_->GetRangeMin());
  range_message_.set_radius(sensor_->GetRangeMax());
  range_message_.set_frame("base_link");

  gazebo::msgs::Pose* pose = new gazebo::msgs::Pose();
  
  gazebo::msgs::Vector3d* position = new gazebo::msgs::Vector3d();
  position->set_x(0.0);
  position->set_y(0.0);
  position->set_z(0.0);
  
  pose->set_allocated_position(position);
  
  gazebo::msgs::Quaternion* orientation = new gazebo::msgs::Quaternion();
  
  orientation->set_x(0.0);
  orientation->set_y(0.0);
  orientation->set_z(0.0);
  orientation->set_w(1.0);
  
  pose->set_allocated_orientation(orientation);
  
  range_message_.set_allocated_world_pose(pose);
 
  node_handle_ = transport::NodePtr(new transport::Node());
  node_handle_->Init(namespace_);

  publisher_ = node_handle_->Advertise<gazebo::msgs::Sonar>(topic_, 1);


  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboSonarPlugin::OnUpdate, this, _1));

  // activate RaySensor
  sensor_->SetActive(true);
}



////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboSonarPlugin::OnUpdate(const common::UpdateInfo& _info) 
{
  common::Time current_time  = world->GetSimTime();
  double dt = (current_time - last_time_).Double();
  last_time_ = current_time;

  // activate RaySensor if it is not yet active
  if (!sensor_->IsActive()) sensor_->SetActive(true);

  //range_.header.stamp.sec  = (world->GetSimTime()).sec;
  //range_.header.stamp.nsec = (world->GetSimTime()).nsec;

  // find ray with minimal range
  range_message_.set_range(sensor_->GetRangeMax());
  int num_ranges = sensor_->GetLaserShape()->GetSampleCount() * sensor_->GetLaserShape()->GetVerticalSampleCount();
  for(int i = 0; i < num_ranges; ++i) {
    double ray = sensor_->GetLaserShape()->GetRange(i);
    if (ray < range_message_.range()) range_message_.set_range(ray);
  }

  // add Gaussian noise (and limit to min/max range)
  if (range_message_.range() < range_message_.range_max()) {
    range_message_.set_range(range_message_.range()+this->GaussianKernel(0,gaussian_noise_));
    if (range_message_.range() < range_message_.range_min()) range_message_.set_range(range_message_.range_min());
    if (range_message_.range() > range_message_.range_max()) range_message_.set_range(range_message_.range_max());
  }

  publisher_->Publish(range_message_);
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboSonarPlugin::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  // normalized uniform random variable
  double U = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  // normalized uniform random variable
  double V = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);

  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboSonarPlugin)

} // namespace gazebo
