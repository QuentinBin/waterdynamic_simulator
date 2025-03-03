/*
 * @Description: None
 * @Author: Bin Peng
 * @Email: pb20020816@163.com
 * @Date: 2025-02-24 15:34:22
 * @LastEditTime: 2025-02-25 10:18:32
 */
#ifndef __WDSIM_FIN_PLUGIN_HH__
#define __WDSIM_FIN_PLUGIN_HH__

#include <gazebo/gazebo.hh>
#include <waterdynamic_simulator/WaterDynamic.hh>

#include <boost/scoped_ptr.hpp>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

namespace gazebo
{

typedef boost::shared_ptr<UnderWaterObject_c> UnderWaterObject_Ptr;

class WaterDynamicFinPlugin : public gazebo::ModelPlugin
{
    public: WaterDynamicFinPlugin();
    public: virtual ~WaterDynamicFinPlugin();

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();
    public: virtual void OnUpdate(const gazebo::common::UpdateInfo &_info);
    // Connect the update callback
    public: virtual void Connect();
    public: virtual void Reset();
    

    protected: void UpdateInput(ConstVector3dPtr &_msg);
    protected: void UpdateFlowVelocity(ConstVector3dPtr &_msg);
    protected: void RosUpdateInput(const geometry_msgs::Vector3::ConstPtr& msg);

    public: ignition::math::Vector3d flow_velocity_;
    protected: gazebo::transport::SubscriberPtr flowSubscriber_;
    protected: gazebo::transport::SubscriberPtr inputSubscriber_;
    protected: gazebo::event::ConnectionPtr update_connection_;
    private: gazebo::event::ConnectionPtr ros_update_connection_;
    protected: gazebo::physics::WorldPtr world_;
    protected: gazebo::physics::ModelPtr model_;
    protected: gazebo::transport::NodePtr node_;
    protected: physics::JointPtr joint_;
    protected: UnderWaterObject_Ptr underWater_objects_;

    private: boost::scoped_ptr<ros::NodeHandle> rosNode_;
    private: ros::Subscriber ros_inputSubscriber_;
    private: ros::Publisher ros_outputPublisher_;
    private: gazebo::common::Time ros_publish_period_;
    private: gazebo::common::Time last_ros_publish_time_;

    protected: double angle;
    protected: double velocity_by_rotation;
    protected: double acceleration_by_rotation;
    protected: gazebo::common::Time angleStamp;
    protected: bool received_input_;
    
};

inline std::vector<double> Str2Vector(std::string _input)
{
  std::vector<double> output;
  std::string buf;
  std::stringstream ss(_input);
  while (ss >> buf)
    output.push_back(std::stod(buf));
  return output;
};

} // namespace gazebo


#endif