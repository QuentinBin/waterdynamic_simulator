/*
 * @Description: None
 * @Author: Bin Peng
 * @Email: pb20020816@163.com
 * @Date: 2025-02-14 12:16:41
 * @LastEditTime: 2025-02-18 14:25:29
 */
#include <cmath>
#include "waterdynamic_simulator/WaterDynamic.hh"

namespace gazebo
{
/////////////////////////////////////////////////
UnderWaterObject_c::UnderWaterObject_c(sdf::ElementPtr _sdf, physics::LinkPtr _link)
{
    GZ_ASSERT(_link != NULL, "Link is NULL");
    this->link_ = _link;
    // initialize the link
    this->volume_ = 0.0;
    this->fluid_density_ = 1000.0;
    this->g_ = 9.8;
    this->center_of_buoyancy_ = ignition::math::Vector3d(0, 0, 0);
};

/////////////////////////////////////////////////
UnderWaterObject_c::~UnderWaterObject_c(){};

/////////////////////////////////////////////////
void UnderWaterObject_c::GetBuoyancyForce(
    ignition::math::Vector3d &_buoyancy_force,
    ignition::math::Vector3d &_buoyancy_torque)
{
    ignition::math::Pose3d _pose = this->link_->WorldPose();
    ignition::math::Vector3d CoB_in_world_frame = _pose.Rot().RotateVector(this->center_of_buoyancy_);
    _buoyancy_force = this->fluid_density_ * this->g_ * this->volume_ * ignition::math::Vector3d(0, 0, 1);
    _buoyancy_torque = CoB_in_world_frame.Cross(_buoyancy_force);
}

/////////////////////////////////////////////////
void UnderWaterObject_c::ApplyBuoyancyForce()
{
    ignition::math::Vector3d buoyancy_force;
    ignition::math::Vector3d buoyancy_torque;
    this->GetBuoyancyForce(buoyancy_force, buoyancy_torque);
    this->link_->AddForce(buoyancy_force);
    this->link_->AddTorque(buoyancy_torque);
}

/////////////////////////////////////////////////
void UnderWaterObject_c::GetLiftDragForce(
    const ignition::math::Vector3d &_flow_linear_vel,
    const double _coef_lift,
    const double _coef_drag_x,
    const double _coef_drag_y,
    const double _coef_drag_z,
    ignition::math::Vector3d &_lift_force,
    ignition::math::Vector3d &_drag_force,
    ignition::math::Vector3d &_lift_torque,
    ignition::math::Vector3d &_drag_torque)
{
    ignition::math::Vector3d _relative_vel_in_world_frame = this->link_->WorldLinearVel() - _flow_linear_vel; //relative velocity to the flow
    ignition::math::Vector3d _relative_vel_in_body_frame = this->link_->WorldPose().Rot().RotateVectorReverse(_relative_vel_in_world_frame);
    ignition::math::Vector3d _lift_dir_xoy, _lift_dir_xoz, _drag_dir;
    _lift_dir_xoy = ignition::math::Vector3d(0, 0, 1).Cross(_relative_vel_in_body_frame);
    if (_lift_dir_xoy.X() * _relative_vel_in_body_frame.X() < 0)
    {
        _lift_dir_xoy = -_lift_dir_xoy;
    }
    _drag_dir = -_relative_vel_in_body_frame;
    double attack_angle = std::acos(_relative_vel_in_body_frame.Z() / _relative_vel_in_body_frame.Length());
}

/////////////////////////////////////////////////
void UnderWaterObject_c::SetVolume(double _volume)
{
    GZ_ASSERT(_volume >= 0, "Volume should be greater than or equal to 0");
    this->volume_ = _volume;
}

/////////////////////////////////////////////////
double UnderWaterObject_c::GetVolume()
{
    return this->volume_;
}

/////////////////////////////////////////////////
void UnderWaterObject_c::SetFluidDensity(double _fluid_density)
{
    GZ_ASSERT(_fluid_density >= 0, "Fluid density should be greater than or equal to 0");
    this->fluid_density_ = _fluid_density;
}

/////////////////////////////////////////////////
double UnderWaterObject_c::GetFluidDensity()
{
    return this->fluid_density_;
}

/////////////////////////////////////////////////
void UnderWaterObject_c::SetGravityCoef(double _g)
{
    GZ_ASSERT(_g >= 0, "Gravity should be greater than or equal to 0");
    this->g_ = _g;
}

/////////////////////////////////////////////////
double UnderWaterObject_c::GetGravityCoef()
{
    return this->g_;
}

} // namespace gazebo

