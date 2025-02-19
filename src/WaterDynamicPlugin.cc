#include <gazebo/gazebo.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/Shape.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/transport.hh>

#include <waterdynamic_simulator/WaterDynamicPlugin.hh>

namespace gazebo
{

GZ_REGISTER_MODEL_PLUGIN(WaterDynamicPlugin)

/////////////////////////////////////////////////
WaterDynamicPlugin::WaterDynamicPlugin(){}
WaterDynamicPlugin::~WaterDynamicPlugin()
{
    this->update_connection_.reset();
}


/////////////////////////////////////////////////
void WaterDynamicPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model != NULL, "Model is NULL");
    GZ_ASSERT(_sdf != NULL, "SDF is NULL");

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
      &WaterDynamicPlugin::UpdateFlowVelocity, this);
    }

    // read sdf parameters
    double fluid_density = 1000.0;
    if (_sdf->HasElement("fluid_density"))
    {
        fluid_density = _sdf->Get<double>("fluid_density");
    }
    
    this->base_link_name_ = std::string();
    if (_sdf->HasElement("link"))
    {
        for (sdf::ElementPtr linkElem = _sdf->GetElement("link"); linkElem;
            linkElem = linkElem->GetNextElement("link"))
        {
            physics::LinkPtr link;
            std::string linkName = "";

            if (linkElem->HasAttribute("name"))
            {
                linkName = linkElem->Get<std::string>("name");
                std::size_t found = linkName.find("base_link");
                if (found != std::string::npos)
                {
                this->base_link_name_ = linkName;
                gzmsg << "Name of the BASE_LINK: " << this->base_link_name_ << std::endl;
                }

                link = this->model_->GetLink(linkName);
                if (!link)
                {
                gzwarn << "Specified link [" << linkName << "] not found."
                        << std::endl;
                continue;
                }
            }
            else
            {
                gzwarn << "Attribute name missing from link [" << linkName
                       << "]" << std::endl;
                continue;
            }

            UnderWaterObject_Ptr underwaterobject;
            underwaterobject.reset(new UnderWaterObject_c(linkElem, link));
            underwaterobject->SetFluidDensity(fluid_density);

            std::vector<double> cob = {0, 0, 0};
            if (linkElem->HasAttribute("center_of_buoyancy"))
            {
                cob = Str2Vector(linkElem->Get<std::string>("center_of_buoyancy"));
                underwaterobject->SetCenterOfBuoyancy(ignition::math::Vector3d(cob[0], cob[1], cob[2]));
            }

            double g = std::abs(this->world_->Gravity().Z());
            underwaterobject->SetGravityCoef(g);

            double volume = -1;
            if (linkElem->HasAttribute("volume"))
            {
                volume = linkElem->Get<double>("volume");
            }
            underwaterobject->SetVolume(volume);

            double coef_lift_xoy = 0.0;
            double coef_lift_xoz = 0.0;
            double coef_drag_xoy = 0.0;
            double coef_drag_xoz = 0.0;
            if (linkElem->HasAttribute("coef_lift_drag"))
            {
                coef_lift_drag = Str2Vector(linkElem->Get<std::string>("coef_lift_drag"));
                underwaterobject->SetLiftDragCoef(coef_lift_drag[0], coef_lift_drag[1], coef_lift_drag[2], coef_lift_drag[3]);
            }

            double coef_added_mass_x = 0.0;
            double coef_added_mass_y = 0.0;
            double coef_added_mass_z = 0.0;
            if (linkElem->HasAttribute("coef_added_mass"))
            {
                coef_added_mass = Str2Vector(linkElem->Get<std::string>("coef_added_mass"));
                underwaterobject->SetAddedMassCoef(coef_added_mass[0], coef_added_mass[1], coef_added_mass[2]);
            }
            
            this->underWater_objects_[link] = underwaterobject;
            // this->underWater_objects_[link]->PrintParameters('all')
        }

    }
    this->Connect();
}

void WaterDynamicPlugin::Connect()
{
  // Connect the update event
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&UnderwaterObjectPlugin::Update,
                    this, _1));
}

void WaterDynamicPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
    double time = _info.simTime.Double();

    for (std::map<gazebo::physics::LinkPtr,
        HydrodynamicModelPtr>::iterator it = models.begin();
        it != models.end(); ++it)
    {
        physics::LinkPtr link = it->first;
        UnderWaterObject_Ptr underwaterobject = it->second;

        double linearAccel, angularAccel;
        linearAccel = link->RelativeLinearAccel().Length();
        angularAccel = link->RelativeAngularAccel().Length();
        GZ_ASSERT(!std::isnan(linearAccel) && !std::isnan(angularAccel),
            "Linear or angular accelerations are invalid.");
        underwaterobject->ApplyHydroDynamics(this->flow_velocity_);
    }
}

void WaterDynamicPlugin::UpdateFlowVelocity(ConstVector3dPtr &_msg)
{
    this->flow_velocity_ = ignition::math::Vector3d(_msg->x(), _msg->y(), _msg->z());
}

void WaterDynamicPlugin::Init(){}




} // namespace gazebo