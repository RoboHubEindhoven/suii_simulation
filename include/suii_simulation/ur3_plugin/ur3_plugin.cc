#ifndef _UR3_PLUGIN_HH_
#define _UR3_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "sensor_msgs/JointState.h"

namespace gazebo
{
  class UR3Plugin : public ModelPlugin
  {
    public: UR3Plugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
    if (_model->GetJointCount() == 0)
    {
      std::cerr << "Invalid joint count, delta_robot plugin not loaded\n";
      return;
    }

    this->model = _model;
    this->joint1 = _model->GetJoint("ur3/shoulder_pan_joint");
    this->joint2 = _model->GetJoint("ur3/shoulder_lift_joint");
    this->joint3 = _model->GetJoint("ur3/elbow_joint");
    this->joint4 = _model->GetJoint("ur3/wrist_1_joint");
    this->joint5 = _model->GetJoint("ur3/wrist_2_joint");
    this->joint6 = _model->GetJoint("ur3/wrist_3_joint");

    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client",
          ros::init_options::NoSigintHandler);
    }

    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<sensor_msgs::JointState>(
          "/joint_states",
          1,
          boost::bind(&UR3Plugin::OnRosMsg, this, _1),
          ros::VoidPtr(), &this->rosQueue);

    this->rosSub = this->rosNode->subscribe(so);

    this->rosQueueThread =
        std::thread(std::bind(&UR3Plugin::QueueThread, this));
    }

    public: void OnRosMsg(const sensor_msgs::JointStateConstPtr _msg)
    {
      this->joint1->SetPosition(0, _msg->position[0]);
      this->joint2->SetPosition(0, _msg->position[1]);
      this->joint3->SetPosition(0, _msg->position[2]);
      this->joint4->SetPosition(0, _msg->position[3]);
      this->joint5->SetPosition(0, _msg->position[4]);
      this->joint6->SetPosition(0, _msg->position[5]);
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
    private: physics::JointPtr joint1;
    private: physics::JointPtr joint2;
    private: physics::JointPtr joint3;
    private: physics::JointPtr joint4;
    private: physics::JointPtr joint5;
    private: physics::JointPtr joint6;
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber rosSub;
    private: ros::CallbackQueue rosQueue;
    private: std::thread rosQueueThread;
    };

  GZ_REGISTER_MODEL_PLUGIN(UR3Plugin)
}
#endif