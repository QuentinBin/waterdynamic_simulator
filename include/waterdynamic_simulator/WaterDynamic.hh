/*
 * @Description: None
 * @Author: Bin Peng
 * @Email: pb20020816@163.com
 * @Date: 2025-02-14 12:16:22
 * @LastEditTime: 2025-02-18 14:16:58
 */
#ifndef WDSIM_WATERDYNAMIC_HH_
#define WDSIM_WATERDYNAMIC_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Collision.hh>

namespace gazebo
{

class UnderWaterObject_c
{
    public: UnderWaterObject_c(sdf::ElementPtr _sdf, physics::LinkPtr _link);
    public: ~UnderWaterObject_c();
    ///////////Hydrodynamic Functions////////////
    // Get the buoyancy force in World Frame
    public: void GetBuoyancyForce(
        ignition::math::Vector3d &_boyancy_force,
        ignition::math::Vector3d &_buoyancy_torque
    );
    // Apply the buoyancy force on link
    public: void ApplyBuoyancyForce();
    // Get the lift and drag force in World Frame
    public: void GetLiftDragForce(
        const ignition::math::Vector3d &_flow_linear_vel,
        const double _coef_lift,
        const double _coef_drag_x,
        const double _coef_drag_y,
        const double _coef_drag_z,
        ignition::math::Vector3d &_lift_force,
        ignition::math::Vector3d &_drag_force,
        ignition::math::Vector3d &_lift_torque,
        ignition::math::Vector3d &_drag_torque
    );
    // Apply the lift and drag force on link
    public: void ApplyLiftDragForce();

    
    // Set the volume of the object
    public: void SetVolume(double _volume = -1);
    // Get the volume of the object
    public: double GetVolume();
    // Sey the fluid density
    public: void SetFluidDensity(double _fluid_density);
    // Get the fluid density
    public: double GetFluidDensity();
    // Set the gravity
    public: void SetGravityCoef(double _g);
    // Get the gravity
    public: double GetGravityCoef();
    // Set th Center of Buoyancy
    public: void SetCenterOfBuoyancy(ignition::math::Vector3d &_center_of_buoyancy);
    // Get the Center of Buoyancy
    public: ignition::math::Vector3d GetCenterOfBuoyancy();

    protected: double volume_;
    protected: double fluid_density_;
    protected: double g_;
    protected: ignition::math::Vector3d center_of_buoyancy_;

    protected: physics::LinkPtr link_;

};

}

#endif