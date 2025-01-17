#ifndef _ROBOT_CONTROL_PLUGIN_HH_
#define _ROBOT_CONTROL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "geometry_msgs/Twist.h"

namespace gazebo
{
	class RobotControlPlugin : public ModelPlugin
	{
		private: physics::ModelPtr model;
		private: physics::JointPtr joint_1;
		private: physics::JointPtr joint_2;
		private: physics::JointPtr joint_3;
		private: physics::JointPtr joint_4;
		private: physics::JointPtr joint_5;
		private: physics::JointPtr joint_6;
		private: physics::JointPtr joint_7;
		private: physics::JointPtr joint_8;
		private: common::PID pid;
		private: std::unique_ptr<ros::NodeHandle> rosNode;
		private: ros::Subscriber rosSub;
		private: ros::CallbackQueue rosQueue;
		private: std::thread rosQueueThread;

		public: RobotControlPlugin() {}
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			this->model = _model;
			this->joint_1 = _model->GetJoints()[0];
			this->joint_2 = _model->GetJoints()[1];
			this->joint_3 = _model->GetJoints()[2];
			this->joint_4 = _model->GetJoints()[3];
			this->joint_5 = _model->GetJoints()[4];
			this->joint_6 = _model->GetJoints()[5];
			this->joint_7 = _model->GetJoints()[6];
			this->joint_8 = _model->GetJoints()[7];

			this->pid = common::PID(5, 2, 0);

			this->model->GetJointController()->SetVelocityPID(this->joint_1->GetScopedName(), this->pid);
			this->model->GetJointController()->SetVelocityPID(this->joint_2->GetScopedName(), this->pid);
			this->model->GetJointController()->SetVelocityPID(this->joint_3->GetScopedName(), this->pid);
			this->model->GetJointController()->SetVelocityPID(this->joint_4->GetScopedName(), this->pid);
			this->model->GetJointController()->SetVelocityPID(this->joint_5->GetScopedName(), this->pid);
			this->model->GetJointController()->SetVelocityPID(this->joint_6->GetScopedName(), this->pid);
			this->model->GetJointController()->SetVelocityPID(this->joint_7->GetScopedName(), this->pid);
			this->model->GetJointController()->SetVelocityPID(this->joint_8->GetScopedName(), this->pid);

			if (!ros::isInitialized())
			{
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "robot_controller", ros::init_options::NoSigintHandler);
			}

			this->rosNode.reset(new ros::NodeHandle("robot_controller"));
			ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
				"/cmd_vel",
				1,
				boost::bind(&RobotControlPlugin::OnRosMsg, this, _1),
				ros::VoidPtr(),
				&this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);
			this->rosQueueThread = std::thread(std::bind(&RobotControlPlugin::QueueThread, this));
		}

		public: void SetVelocity(const geometry_msgs::Twist::ConstPtr& velocity)
		{
		    float dimension = 0.73;
		    float right_velocity;
		    float left_velocity;

		    if (velocity->angular.z != 0)
		    {
		        right_velocity = -velocity->angular.z * dimension / 2;
		        left_velocity = velocity->angular.z * dimension / 2;
		    }
		    else
		    {
		        right_velocity = velocity->linear.x;
		        left_velocity = velocity->linear.x;
		    }

			this->model->GetJointController()->SetVelocityTarget(this->joint_1->GetScopedName(), left_velocity);
			this->model->GetJointController()->SetVelocityTarget(this->joint_2->GetScopedName(), left_velocity);
			this->model->GetJointController()->SetVelocityTarget(this->joint_3->GetScopedName(), right_velocity);
			this->model->GetJointController()->SetVelocityTarget(this->joint_4->GetScopedName(), right_velocity);
			this->model->GetJointController()->SetVelocityTarget(this->joint_5->GetScopedName(), left_velocity);
			this->model->GetJointController()->SetVelocityTarget(this->joint_6->GetScopedName(), left_velocity);
			this->model->GetJointController()->SetVelocityTarget(this->joint_7->GetScopedName(), -right_velocity);
			this->model->GetJointController()->SetVelocityTarget(this->joint_8->GetScopedName(), -right_velocity);
		}

		public: void OnRosMsg(const geometry_msgs::Twist::ConstPtr& msg)
		{
			this->SetVelocity(msg);
		}

		private: void QueueThread()
		{
			static const double timeout = 0.01;
			while (this->rosNode->ok())
			{
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
			}
		}
	};
	GZ_REGISTER_MODEL_PLUGIN(RobotControlPlugin)
}
#endif