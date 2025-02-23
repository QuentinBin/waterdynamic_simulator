/*
 * @Description: None
 * @Author: Bin Peng
 * @Email: pb20020816@163.com
 * @Date: 2025-02-14 12:16:22
 * @LastEditTime: 2025-02-23 15:42:53
 */
#ifndef WDSIM_WATERDYNAMIC_HH_
#define WDSIM_WATERDYNAMIC_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Collision.hh>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gazebo
{



class UnderWaterObject_c
{
    public: UnderWaterObject_c(sdf::ElementPtr _sdf, physics::LinkPtr _link);
    public: ~UnderWaterObject_c();

    
    ///////////Hydrodynamic Functions////////////
    public: void ApplyHydroDynamics(ignition::math::Vector3d &_flow_linear_vel);
    // Get the buoyancy force in World Frame
    public: void GetBuoyancyForce(
        ignition::math::Vector3d &_boyancy_force,
        ignition::math::Vector3d &_buoyancy_torque
    );
    // Apply the buoyancy force on link
    public: void ApplyBuoyancyForce();
    // Get the lift and drag force in Body Frame
    public: void GetLiftDragForce(
        const ignition::math::Vector3d &_flow_linear_vel,
        ignition::math::Vector3d &_lift_force,
        ignition::math::Vector3d &_drag_force
    );
    // Apply the lift and drag force on link
    public: void ApplyLiftDragForce(ignition::math::Vector3d &_flow_linear_vel);
    // Get the reaction force in Body Frame
    public: void GetReactionForce(ignition::math::Vector3d &_reaction_force);
    // Apply the reaction force on link
    public: void ApplyReactionForce();
    ////////////Hydrodynamic Functions////////////
    

    // // Get the acceleration of the object
    // public: void ComputeAcceleration(Eigen::Matrix<double, 6, 1> _velocity_rel, double _time);
    
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
    // Set the Coefficients of Lift and Drag
    public: void SetLiftDragCoef(
        double _coef_lift_xoy,
        double _coef_lift_xoz,
        double _coef_drag_x,
        double _coef_drag_y,
        double _coef_drag_z
    );
    // Get the Coefficients of Lift and Drag
    public: void GetLiftDragCoef(
        double &_coef_lift_xoy,
        double &_coef_lift_xoz,
        double &_coef_drag_x,
        double &_coef_drag_y,
        double &_coef_drag_z
    );
    // Set the Coefficients of Added Mass
    public: void SetAddedMassCoef(
        double _coef_added_mass_x,
        double _coef_added_mass_y,
        double _coef_added_mass_z
    );
    // Get the Coefficients of Added Mass
    public: void GetAddedMassCoef(
        double &_coef_added_mass_x,
        double &_coef_added_mass_y,
        double &_coef_added_mass_z
    );

    public: void PrintParameters(std::string _paramName);

    protected: double volume_;
    protected: double fluid_density_;
    protected: double g_;
    protected: double coef_lift_xoy_;
    protected: double coef_lift_xoz_;
    protected: double coef_drag_x_;
    protected: double coef_drag_y_;
    protected: double coef_drag_z_;
    protected: double coef_added_mass_x;
    protected: double coef_added_mass_y;
    protected: double coef_added_mass_z;
    protected: ignition::math::Vector3d center_of_buoyancy_;
    protected: physics::LinkPtr link_;
    // protected: double last_time_;
    // protected: Eigen::Matrix<double, 6, 1> velocity_;
    // protected: Eigen::Matrix<double, 6, 1> last_velocity_rel_;
    // protected: Eigen::Matrix<double, 6, 1> acceleration_rel_;
};


// class UnderWaterObject_Factory
// {
//     public: UnderWaterObject_c* create(sdf::ElementPtr _sdf, physics::LinkPtr _link);
//     public: static UnderWaterObject_Factory& GetInstance();
//     public: bool RegisterCreator(const std::string& _identifier,
//         UnderWaterObjectCreator _creator);
//         private: UnderWaterObject_Factory() {}
//     private: std::map<std::string, UnderWaterObjectCreator> creators_;
// };

}


#endif