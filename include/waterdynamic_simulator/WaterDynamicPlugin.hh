/*
 * @Description: None
 * @Author: Bin Peng
 * @Email: pb20020816@163.com
 * @Date: 2025-02-19 06:15:38
 * @LastEditTime: 2025-02-19 16:36:14
 */

#ifndef WDSIM_WATERDYNAMIC_PLUGIN_HH_
#define WDSIM_WATERDYNAMIC_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <vector>
#include <string>
#include <map>
#include <waterdynamic_simulator/WaterDynamic.hh>


namespace gazebo{

/// \brief Pointer to model
typedef boost::shared_ptr<UnderWaterObject_c> UnderWaterObject_Ptr;

class WaterDynamicPlugin : public gazebo::ModelPlugin
{
    public: WaterDynamicPlugin();
    public: virtual ~WaterDynamicPlugin();

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: virtual void Init();
    public: virtual void Update(const gazebo::common::UpdateInfo &_info);
    // Connect the update callback
    public: virtual void Connect();

    protected: void UpdateFlowVelocity(ConstVector3dPtr &_msg);
    
    protected: ignition::math::Vector3d flow_velocity_;
    protected: gazebo::transport::SubscriberPtr flowSubscriber_;
    protected: gazebo::event::ConnectionPtr update_connection_;
    protected: gazebo::physics::WorldPtr world_;
    protected: gazebo::physics::ModelPtr model_;
    protected: gazebo::transport::NodePtr node_;
    protected: std::string base_link_name_;

    protected: std::map<gazebo::physics::LinkPtr, UnderWaterObject_Ptr> underWater_objects_;
    
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
#endif // WDSIM_WATERDYNAMIC_PLUGIN_HH_