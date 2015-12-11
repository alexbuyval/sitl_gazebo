
#include <sensor_msgs/Range.h>
#include <random>

#include <Eigen/Core>
//#include "Sonar.pb.h"
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"

#include "common.h"

namespace gazebo
{

class GazeboSonarPlugin : public SensorPlugin
{
public:
  GazeboSonarPlugin();
  ~GazeboSonarPlugin();
  
  void InitializeParams();
  void Publish();

protected:
  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

  void OnUpdate(const common::UpdateInfo&);

private:
  /// \brief The parent World
  physics::WorldPtr world;

  // Pointer to the link
  physics::LinkPtr link_;
  
  common::Time last_time_;
  
  sensors::RaySensorPtr sensor_;

  transport::NodePtr node_handle_;
  transport::PublisherPtr publisher_;

  //sensor_msgs::Range range_;

  gazebo::msgs::Sonar range_message_;
  
  std::string namespace_;
  std::string topic_;
  std::string frame_id_;
  
  unsigned int seed;
  double gaussian_noise_;

  event::ConnectionPtr updateConnection;
  
  double GaussianKernel(double mu, double sigma);
};

} // namespace gazebo

