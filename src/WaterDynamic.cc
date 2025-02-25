/*
 * @Description: None
 * @Author: Bin Peng
 * @Email: pb20020816@163.com
 * @Date: 2025-02-14 12:16:41
 * @LastEditTime: 2025-02-25 16:05:56
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
    // // initialize the link
    // this->volume_ = 0.0;
    // this->fluid_density_ = 1000.0;
    // this->g_ = 9.8;
    // this->center_of_buoyancy_ = ignition::math::Vector3d(0, 0, 0);
};

/////////////////////////////////////////////////
UnderWaterObject_c::~UnderWaterObject_c(){};

/// @brief Apply the hydrodynamic forces on link: Lift, Drag, Buoyancy, Reaction
/// @param _flow_linear_vel 
void UnderWaterObject_c::ApplyHydroDynamics(ignition::math::Vector3d &_flow_linear_vel, ignition::math::Vector3d &_velocity_in_body_frame, ignition::math::Vector3d &_acceleration_in_body_frame)
{
    this->ApplyBuoyancyForce();
    this->ApplyLiftDragForce(_flow_linear_vel, _velocity_in_body_frame);
    this->ApplyReactionForce(_acceleration_in_body_frame);

    // std::cout << "total force: " << this->link_->RelativeForce() << std::endl;
    // std::cout << "total torque: " << this->link_->RelativeTorque() << std::endl;
    // std::cout << "acceleration: " << this->link_->RelativeLinearAccel() << std::endl;
}



/// @brief Compute the buoyancy force in world frame
/// @param _buoyancy_force 
/// @param _buoyancy_torque 
void UnderWaterObject_c::GetBuoyancyForce(
    ignition::math::Vector3d &_buoyancy_force,
    ignition::math::Vector3d &_buoyancy_torque)
{
    ignition::math::Pose3d _pose = this->link_->WorldPose();
    ignition::math::Vector3d CoB_in_world_frame = _pose.Rot().RotateVector(this->center_of_buoyancy_);
    _buoyancy_force = this->fluid_density_ * this->g_ * this->volume_ * ignition::math::Vector3d(0, 0, 1);
    // std::cout << "Buoyancy Force: " << _buoyancy_force << std::endl;
    _buoyancy_torque = CoB_in_world_frame.Cross(_buoyancy_force);
}

/// @brief Apply the buoyancy force on link
void UnderWaterObject_c::ApplyBuoyancyForce()
{
    ignition::math::Vector3d buoyancy_force;
    ignition::math::Vector3d buoyancy_torque;
    this->GetBuoyancyForce(buoyancy_force, buoyancy_torque);
    this->link_->AddForce(buoyancy_force);
    this->link_->AddTorque(buoyancy_torque);
}

/// @brief Compute the lift and drag force in body frame
/// @param _flow_linear_vel 
/// @param _lift_force 
/// @param _drag_force 
void UnderWaterObject_c::GetLiftDragForce(
    const ignition::math::Vector3d &_flow_linear_vel,
    ignition::math::Vector3d &_lift_force,
    ignition::math::Vector3d &_drag_force,
    ignition::math::Vector3d &_velocity_in_body_frame)
{
    ignition::math::Vector3d flow_linear_vel = _flow_linear_vel;
    ignition::math::Vector3d _relative_vel_in_body_frame;
    _relative_vel_in_body_frame = _velocity_in_body_frame - this->link_->WorldPose().Rot().RotateVectorReverse(flow_linear_vel);

    ignition::math::Vector3d _lift_dir_xoy, _lift_dir_xoz, _drag_dir_x, _drag_dir_y, _drag_dir_z;
    _lift_dir_xoy = ignition::math::Vector3d(0, 0, 1).Cross(_relative_vel_in_body_frame);
    _lift_dir_xoz = ignition::math::Vector3d(0, 1, 0).Cross(_relative_vel_in_body_frame);
    if (_lift_dir_xoy.X() * _relative_vel_in_body_frame.X() < 0)
    {
        _lift_dir_xoy = -_lift_dir_xoy;
    }
    _lift_dir_xoy = _lift_dir_xoy.Normalize();
    if (_lift_dir_xoz.X() * _relative_vel_in_body_frame.X() < 0)
    {
        _lift_dir_xoz = -_lift_dir_xoz;
    }
    _lift_dir_xoz = _lift_dir_xoz.Normalize();
    double uu = _relative_vel_in_body_frame.X() * _relative_vel_in_body_frame.X();
    double vv = _relative_vel_in_body_frame.Y() * _relative_vel_in_body_frame.Y();
    double ww = _relative_vel_in_body_frame.Z() * _relative_vel_in_body_frame.Z();
    double attack_angle_xoy = atan2(_relative_vel_in_body_frame.Y(), _relative_vel_in_body_frame.X());
    double attack_angle_xoz = atan2(_relative_vel_in_body_frame.Z(), _relative_vel_in_body_frame.X());
    double lift_xoy = 0.5 * this->fluid_density_ * this->coef_lift_xoy_ * fabs(uu + vv);
    double lift_xoz = 0.5 * this->fluid_density_ * this->coef_lift_xoz_ * fabs(uu + ww);
    _lift_force = lift_xoy * _lift_dir_xoy + lift_xoz * _lift_dir_xoz;
    
    _drag_dir_x = -ignition::math::Vector3d(_relative_vel_in_body_frame.X()+1e-4, 0, 0);
    _drag_dir_y = -ignition::math::Vector3d(0, _relative_vel_in_body_frame.Y()+1e-4, 0);
    _drag_dir_z = -ignition::math::Vector3d(0, 0, _relative_vel_in_body_frame.Z()+1e-5);
    _drag_dir_x = _drag_dir_x.Normalize();
    _drag_dir_y = _drag_dir_y.Normalize();
    _drag_dir_z = _drag_dir_z.Normalize();
    
    double drag_x= 0.5 * this->fluid_density_ * this->coef_drag_x_ * fabs(uu);
    double drag_y = 0.5 * this->fluid_density_ * this->coef_drag_y_ * fabs(vv);
    double drag_z = 0.5 * this->fluid_density_ * this->coef_drag_z_ * fabs(ww);

    _drag_force = drag_x * _drag_dir_x + drag_y * _drag_dir_y + drag_z * _drag_dir_z;
}   

/// @brief Apply the lift and drag force on link
/// @param _flow_linear_vel 
void UnderWaterObject_c::ApplyLiftDragForce(ignition::math::Vector3d &_flow_linear_vel, ignition::math::Vector3d &_velocity_in_body_frame)
{
    ignition::math::Vector3d lift_force;
    ignition::math::Vector3d drag_force;
    // std::cout << "flow vel: " << _flow_linear_vel << std::endl;
    this->GetLiftDragForce(_flow_linear_vel, lift_force, drag_force, _velocity_in_body_frame);
    this->link_->AddRelativeForce(lift_force);
    this->link_->AddForce(drag_force);
}


/// @brief Compute the reaction force in body frame
/// @param _reaction_force 
void UnderWaterObject_c::GetReactionForce(
    ignition::math::Vector3d &_reaction_force,
    ignition::math::Vector3d &_acceleration_in_body_frame)
{
    ignition::math::Vector3d acceleration_rel_in_body_frame = _acceleration_in_body_frame;
    // std::cout << "Acceleration: " << acceleration_rel_in_body_frame << std::endl;
    double added_force_x = this->coef_added_mass_x * acceleration_rel_in_body_frame.X();
    double added_force_y = this->coef_added_mass_y * acceleration_rel_in_body_frame.Y();
    double added_force_z = this->coef_added_mass_z * acceleration_rel_in_body_frame.Z();
    _reaction_force = - ignition::math::Vector3d(added_force_x, added_force_y, added_force_z);
    // std::cout << "Reaction Force in World Frame: " << this->link_->WorldPose().Rot().RotateVector(_reaction_force) << std::endl;  
}

/// @brief Apply the reaction force on link  
void UnderWaterObject_c::ApplyReactionForce(ignition::math::Vector3d &_acceleration_in_body_frame)
{
    ignition::math::Vector3d reaction_force;
    this->GetReactionForce(reaction_force, _acceleration_in_body_frame);
    this->link_->AddRelativeForce(reaction_force);
    std::cout<< "Reaction Force: " << reaction_force << std::endl;
    std::cout << "Reaction Force in World Frame: " << this->link_->WorldPose().Rot().RotateVector(reaction_force) << std::endl;
}

// /////////////////////////////////////////////////
// void UnderWaterObject_c::ComputeAcceleration(Eigen::Matrix<double, 6, 1> _velocity_rel, double _time)
// {
//     double dt = _time - this->last_time_;
//     if(dt <= 0.0)
//     {
//         return ;
//     }

//     Eigen::Matrix<double, 6, 1> _acceleration = (_velocity_rel - this->last_velocity_rel_) / dt;
//     this->last_velocity_rel_ = _velocity_rel;
//     this->last_time_ = _time;
//     this->acceleration_rel_ = _acceleration;
// }

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

/////////////////////////////////////////////////
void UnderWaterObject_c::SetLiftDragCoef(
    double _coef_lift_xoy,
    double _coef_lift_xoz,
    double _coef_drag_x,
    double _coef_drag_y,
    double _coef_drag_z
)
{
    GZ_ASSERT(_coef_lift_xoy >= 0, "Lift Coef in xoy should be greater than or equal to 0");
    GZ_ASSERT(_coef_lift_xoz >= 0, "Lift Coef in xoz should be greater than or equal to 0");
    GZ_ASSERT(_coef_drag_x >= 0, "Drag Coef in x should be greater than or equal to 0");
    GZ_ASSERT(_coef_drag_y >= 0, "Drag Coef in y should be greater than or equal to 0");
    GZ_ASSERT(_coef_drag_z >= 0, "Drag Coef in z should be greater than or equal to 0");
    this->coef_lift_xoy_ = _coef_lift_xoy;
    this->coef_lift_xoz_ = _coef_lift_xoz;
    this->coef_drag_x_ = _coef_drag_x;
    this->coef_drag_y_ = _coef_drag_y;
    this->coef_drag_z_ = _coef_drag_z;
}

/////////////////////////////////////////////////
void UnderWaterObject_c::GetLiftDragCoef(
    double &_coef_lift_xoy,
    double &_coef_lift_xoz,
    double &_coef_drag_x,
    double &_coef_drag_y,
    double &_coef_drag_z)
{
    _coef_lift_xoy = this->coef_lift_xoy_;
    _coef_lift_xoz = this->coef_lift_xoz_;
    _coef_drag_x = this->coef_drag_x_;
    _coef_drag_y = this->coef_drag_y_;
    _coef_drag_z = this->coef_drag_z_;
}


/////////////////////////////////////////////////
void UnderWaterObject_c::SetAddedMassCoef(
    double _coef_added_mass_x,
    double _coef_added_mass_y,
    double _coef_added_mass_z)
{
    GZ_ASSERT(_coef_added_mass_x >= 0, "Added Mass Coef in x should be greater than or equal to 0");
    GZ_ASSERT(_coef_added_mass_y >= 0, "Added Mass Coef in y should be greater than or equal to 0");
    GZ_ASSERT(_coef_added_mass_z >= 0, "Added Mass Coef in z should be greater than or equal to 0");
    this->coef_added_mass_x = _coef_added_mass_x;
    this->coef_added_mass_y = _coef_added_mass_y;
    this->coef_added_mass_z = _coef_added_mass_z;
}

/////////////////////////////////////////////////
void UnderWaterObject_c::GetAddedMassCoef(
    double &_coef_added_mass_x,
    double &_coef_added_mass_y,
    double &_coef_added_mass_z)
{
    _coef_added_mass_x = this->coef_added_mass_x;
    _coef_added_mass_y = this->coef_added_mass_y;
    _coef_added_mass_z = this->coef_added_mass_z;
}

/////////////////////////////////////////////////
void UnderWaterObject_c::SetCenterOfBuoyancy(ignition::math::Vector3d &_center_of_buoyancy)
{
    this->center_of_buoyancy_ = _center_of_buoyancy;
}


/////////////////////////////////////////////////
ignition::math::Vector3d UnderWaterObject_c::GetCenterOfBuoyancy()
{
    return this->center_of_buoyancy_;
}




} // namespace gazebo

