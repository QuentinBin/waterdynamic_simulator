#include <waterdynamic_simulator/FinPlugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/Shape.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(WaterDynamicFinPlugin)

/////////////////////////////////////////////
WaterDynamicFinPlugin::WaterDynamicFinPlugin() 
{   
    this->angle = 0.0;
    this->velocity_by_rotation = 0.0;
    this->acceleration_by_rotation = 0.0;
    this->angleStamp = gazebo::common::Time(0.0);
}

/////////////////////////////////////////////
WaterDynamicFinPlugin::~WaterDynamicFinPlugin()
{
    this->update_connection_.reset();
}

/////////////////////////////////////////////
void WaterDynamicFinPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model != NULL, "Model is NULL");
    GZ_ASSERT(_sdf != NULL, "SDF is NULL");
    std::cout << "Loading WaterDynamicFinPlugin" << std::endl;
    this->model_ = _model;
    this->world_ = this->model_->GetWorld();
    std::string world_name = this->world_->Name();

    // Initialize the transport node
    this->node_ = transport::NodePtr(new transport::Node());
    this->node_->Init(world_name);
    
    // If fluid topic is available, subscribe to it
    if (_sdf->HasElement("flow_velocity_topic"))
    {
    std::string flowTopic = _sdf->Get<std::string>("flow_velocity_topic");
    GZ_ASSERT(!flowTopic.empty(),
              "Fluid velocity topic tag cannot be empty");

    gzmsg << "Subscribing to current velocity topic: " << flowTopic
        << std::endl;
    this->flowSubscriber_ = this->node_->Subscribe(flowTopic,
      &WaterDynamicFinPlugin::UpdateFlowVelocity, this);
    }

    if (_sdf->HasElement("input_topic"))
    {
        std::string inputTopic = _sdf->Get<std::string>("input_topic");
        GZ_ASSERT(!inputTopic.empty(),
                  "Input topic tag cannot be empty");

        gzmsg << "Subscribing to input topic: " << inputTopic
              << std::endl;
        this->inputSubscriber_ = this->node_->Subscribe(inputTopic,
          &WaterDynamicFinPlugin::UpdateInput, this);
    }

    // read sdf parameters
    double fluid_density = 1000.0;
    if (_sdf->HasElement("fluid_density"))
    {   
        fluid_density = _sdf->Get<double>("fluid_density");
        std::cout << "fluid density: " << _sdf->Get<double>("fluid_density") << std::endl;
    }

    if (_sdf->HasElement("link"))
    {
        sdf::ElementPtr linkElem = _sdf->GetElement("link");
        physics::LinkPtr link;
        std::string linkName = "";
        if (linkElem->HasAttribute("name"))
        {   
            std::cout << "link name: " << linkElem->Get<std::string>("name") << std::endl;
            linkName = linkElem->Get<std::string>("name");
           
            link = this->model_->GetLink(linkName);
        }
        else
        {
            gzwarn << "Attribute name missing from link [" << linkName
                   << "]" << std::endl;
        }
        this->underWater_objects_.reset(new UnderWaterObject_c(linkElem, link));
        this->underWater_objects_->SetFluidDensity(fluid_density);

        std::vector<double> cob = {0, 0, 0};
        if (linkElem->HasElement("center_of_buoyancy"))
        {
            cob = Str2Vector(linkElem->Get<std::string>("center_of_buoyancy"));
            ignition::math::Vector3d cob_vec;
            cob_vec.Set(cob[0], cob[1], cob[2]);
            this->underWater_objects_->SetCenterOfBuoyancy(cob_vec);
            std::cout << "Center of Buoyancy: " << cob_vec << std::endl;
        }

        double g = std::abs(this->world_->Gravity().Z());
        this->underWater_objects_->SetGravityCoef(g);

        double volume = -1;
        if (linkElem->HasElement("volume"))
        {
            volume = linkElem->Get<double>("volume");
        }
        this->underWater_objects_->SetVolume(volume);

        double coef_lift_xoy = 0.0;
        double coef_lift_xoz = 0.0;
        double coef_drag_x = 0.0;
        double coef_drag_y = 0.0;
        double coef_drag_z = 0.0;
        std::vector<double> coef_lift_drag = {0, 0, 0, 0, 0};
        if (linkElem->HasElement("coef_lift_drag"))
        {
            coef_lift_drag = Str2Vector(linkElem->Get<std::string>("coef_lift_drag"));
            this->underWater_objects_->SetLiftDragCoef(coef_lift_drag[0], coef_lift_drag[1], coef_lift_drag[2], coef_lift_drag[3], coef_lift_drag[4]);
        }

        double coef_added_mass_x = 0.0;
        double coef_added_mass_y = 0.0;
        double coef_added_mass_z = 0.0;
        std::vector<double> coef_added_mass = {0, 0, 0};
        if (linkElem->HasElement("coef_added_mass"))
        {
            coef_added_mass = Str2Vector(linkElem->Get<std::string>("coef_added_mass"));
            this->underWater_objects_->SetAddedMassCoef(coef_added_mass[0], coef_added_mass[1], coef_added_mass[2]);
        }
        

    }

    this->Connect();
}


/////////////////////////////////////////////
void WaterDynamicFinPlugin::Connect()
{
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&WaterDynamicFinPlugin::Update, this, _1));
}

void WaterDynamicFinPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
    GZ_ASSERT(!std::isnan(this->angle), "Angle is NaN");

    double upper_limit = 0.0, lower_limit = 0.0;
    upper_limit = this->joint_->UpperLimit(0);
    lower_limit = this->joint_->LowerLimit(0);
    this->angle = std::max(lower_limit, std::min(upper_limit, this->angle));

    ignition::math::Vector3d velocity_by_rotation = ignition::math::Vector3d(this->velocity_by_rotation, 0, 0);
    ignition::math::Vector3d acceleration_by_rotation = ignition::math::Vector3d(this->acceleration_by_rotation, 0, 0);

    ignition::math::Vector3d velocity_in_body_frame = this->underWater_objects_->link_->RelativeLinearVel() + velocity_by_rotation;
    ignition::math::Vector3d acceleration_in_body_frame = this->underWater_objects_->link_->RelativeLinearAccel() + acceleration_by_rotation;

    this->underWater_objects_->ApplyHydroDynamics(this->flow_velocity_, velocity_in_body_frame, acceleration_in_body_frame);
    
    this->joint_->SetPosition(0, this->angle); //Do last since this will set the joint velocity to 0

    this->angleStamp = _info.simTime;
}


void WaterDynamicFinPlugin::UpdateFlowVelocity(ConstVector3dPtr &_msg)
{
    this->flow_velocity_ = ignition::math::Vector3d(_msg->x(), _msg->y(), _msg->z());
}

void WaterDynamicFinPlugin::UpdateInput(const double (&_msg)[3])
{
    this->angle = _msg[0];
    this->velocity_by_rotation = _msg[1];
    this->acceleration_by_rotation = _msg[2];
}

} // namespace gazebo