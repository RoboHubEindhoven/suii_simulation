#ifndef _ROTATING_TABLE_PLUGIN_HH_
#define _ROTATING_TABLE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  class RotatingTablePlugin : public ModelPlugin
  {
    public: RotatingTablePlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
    if (_model->GetJointCount() == 0)
    {
      std::cerr << "Invalid joint count, delta_robot plugin not loaded\n";
      return;
    }

    this->model = _model;
    this->joint = _model->GetJoint("rotation");


    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client",
          ros::init_options::NoSigintHandler);
    }

    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<std_msgs::Float32>(
          "/" + this->model->GetName() + "/speed",
          1,
          boost::bind(&RotatingTablePlugin::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);

    this->rosSub = this->rosNode->subscribe(so);

    this->rosQueueThread =
        std::thread(std::bind(&RotatingTablePlugin::QueueThread, this));
    }

    public: void OnRosMsg(const std_msgs::Float32ConstPtr _msg)
    {
      this->joint->SetVelocity(0, _msg->data);
    }

    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    private: physics::ModelPtr model;
    private: physics::JointPtr joint;
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;
    };

  GZ_REGISTER_MODEL_PLUGIN(RotatingTablePlugin)
}
#endif