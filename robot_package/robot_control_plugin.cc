#ifndef _ROBOT_CONTROL_PLUGIN_HH_
#define _ROBOT_CONTROL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <array>

#include <thread>
#include <cmath>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <ignition/math/Vector3.hh>
#include "geometry_msgs/Twist.h"
#include <gazebo_msgs/ModelStates.h>
 #include <tf2/LinearMath/Quaternion.h>

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
		private: ros::Subscriber rosOdom;
		private: ros::CallbackQueue rosQueue;
		private: ros::CallbackQueue rosOdomQueue;
		private: std::thread rosQueueThread;
		private: std::thread rosOdomQueueThread;
		private: gazebo_msgs::ModelStates current_state;
		private: tf2::Quaternion current_quaternion;
		private:
			double qx{}, qy{}, qz{}, qw{}, yaw{}, linX{}, linY{};
			double px{}, py{}, pz{};
		// private: math::Quaternion quaternion;

		public: RobotControlPlugin() {}
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			// Safety check
			if (_model->GetJointCount() == 0)
			{
				std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
				return;
			}
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

			ros::SubscribeOptions  od = ros::SubscribeOptions::create<gazebo_msgs::ModelStates>(
				"/gazebo/model_states",
				1,
				boost::bind(&RobotControlPlugin::OnRosOdomMsg, this, _1),
				ros::VoidPtr(),
				&this->rosOdomQueue);
			this->rosOdom = this->rosNode->subscribe(od);

			this->rosQueueThread = std::thread(std::bind(&RobotControlPlugin::QueueThread, this));
		}


		public: void SetVelocity(const geometry_msgs::Twist::ConstPtr& velocity)
		{
		    float width{0.73/2}, length{1.46/2}, wheel_radius{0.2};
			const double pi = std::acos(-1);

			// For Rail Wheels
		    double right_velocity{(1/wheel_radius)*velocity->linear.x}, left_velocity{(1/wheel_radius)*velocity->linear.x}; 

			// For Mecanum Wheels
			double front_right_velocity{}, rear_right_velocity{}, rear_left_velocity{}, front_left_velocity{};
			// for lateral movement
			if (velocity->linear.x == 0 and velocity->linear.y != 0 and velocity->angular.z == 0){
				double speed {std::abs(velocity->linear.y)};
				front_right_velocity = (1/wheel_radius)*speed*((velocity->linear.y < 0)?-1:1);
				rear_right_velocity = (1/wheel_radius)*speed*((velocity->linear.y > 0)?-1:1);
				rear_left_velocity = (1/wheel_radius)*speed*((velocity->linear.y < 0)?-1:1);
				front_left_velocity = (1/wheel_radius)*speed*((velocity->linear.y > 0)?-1:1);
			}
			// for longitudinal movement
			else if(velocity->linear.x != 0 and velocity->linear.y == 0 and velocity->angular.z == 0){
				front_right_velocity = (1/wheel_radius)*velocity->linear.x;
				rear_right_velocity = (1/wheel_radius)*velocity->linear.x;
				rear_left_velocity = (1/wheel_radius)*velocity->linear.x;
				front_left_velocity = (1/wheel_radius)*velocity->linear.x;
			}
			else{
				double theta{(velocity->linear.x == 0 and velocity->linear.y != 0) ? pi/4 : 0};
				front_right_velocity = (1/wheel_radius) * (velocity->linear.x + velocity->linear.y + (width + length)*std::sin(velocity->angular.z + theta));
				rear_left_velocity = (1/wheel_radius) * (velocity->linear.x + velocity->linear.y - (width + length)*std::sin(velocity->angular.z - theta));
				rear_right_velocity = (1/wheel_radius) * (velocity->linear.x - velocity->linear.y + (width + length)*std::sin(velocity->angular.z + theta));
				front_left_velocity = (1/wheel_radius) * (velocity->linear.x - velocity->linear.y - (width + length)*std::sin(velocity->angular.z - theta));
			}


			this->yaw = std::atan2(2*(qw*qz+qx*qy),  1-2*(qy*qy+qz*qz));
			this->linX = std::cos(this->yaw) * velocity->linear.x - std::sin(this->yaw) * velocity->linear.y;
    		this->linY = std::cos(this->yaw) * velocity->linear.y + std::sin(this->yaw) * velocity->linear.x;
			this->model->SetLinearVel(ignition::math::Vector3d(-linY, linX, 0.0));
			this->model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0,velocity->angular.z));

			this->model->GetJointController()->SetVelocityTarget(this->joint_1->GetScopedName(), front_right_velocity);
			this->model->GetJointController()->SetVelocityTarget(this->joint_2->GetScopedName(), rear_right_velocity);
			this->model->GetJointController()->SetVelocityTarget(this->joint_3->GetScopedName(), rear_left_velocity);
			this->model->GetJointController()->SetVelocityTarget(this->joint_4->GetScopedName(), front_left_velocity);

			this->model->GetJointController()->SetVelocityTarget(this->joint_5->GetScopedName(), left_velocity);
			this->model->GetJointController()->SetVelocityTarget(this->joint_6->GetScopedName(), left_velocity);
			this->model->GetJointController()->SetVelocityTarget(this->joint_7->GetScopedName(), -right_velocity);
			this->model->GetJointController()->SetVelocityTarget(this->joint_8->GetScopedName(), -right_velocity);
		}

		public: void OnRosMsg(const geometry_msgs::Twist::ConstPtr& msg)
		{
			this->SetVelocity(msg);
		}

		public: void OnRosOdomMsg(const gazebo_msgs::ModelStates::ConstPtr& odom)
		{
			short index{};
			size_t len{odom->name.size()};
			for (short i{}; i < len; i++)
				if (odom->name[i] == "robot"){
					index = i;
					break;
				}
			// gazebo_msgs::ModelState test_model{};
			// this->current_state.name = odom->name[index];
			// this->current_state.pose = odom->pose[index];
			// this->current_state.twist = odom->twist[index];
			// this->qx = odom->pose[index].orientation.x;
			// std::cout << odom->pose[index].orientation.x << std::endl;
			this -> qx = odom->pose[index].orientation.x;
			this -> qy = odom->pose[index].orientation.y;
			this -> qz = odom->pose[index].orientation.z;
			this -> qw = odom->pose[index].orientation.w;

			this -> px = odom->pose[index].position.x;
			this -> py = odom->pose[index].position.y;
			this -> pz = odom->pose[index].position.z;
		}

		private: void QueueThread()
		{
			static const double timeout = 0.01;
			while (this->rosNode->ok())
			{
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
				this->rosOdomQueue.callAvailable(ros::WallDuration(timeout));
			}
		}
	};
	GZ_REGISTER_MODEL_PLUGIN(RobotControlPlugin)
}
#endif
