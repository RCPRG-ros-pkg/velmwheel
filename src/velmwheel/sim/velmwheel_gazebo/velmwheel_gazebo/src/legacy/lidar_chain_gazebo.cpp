#include <functional>
#include <string>
#include <iostream>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <ros/console.h>
#include <tf2_ros/transform_broadcaster.h>

#define MODEL_NAME std::string("omnivelma")
#define CLIENT_NAME "gazebo_ros"
#define MONOKL_R_NAME "monokl_r"
#define MONOKL_L_NAME "monokl_l"
#define MONOKL_HEART_NAME "heart"

namespace gazebo
{
	//Klasa nadająca aktualną pozycję czujników laserowych do tf2
	class MonoklChain : public ModelPlugin
	{
	public:
		MonoklChain()
		{
			counter = 0;
		}
		
		
		///Uruchamiane na inicjalizację
		void Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
		{
			model = parent;
			std::string linkPrefix = std::string(model -> GetName()).append("::").append(MODEL_NAME).append("::");
			linkL = model -> GetLink(linkPrefix + std::string(MONOKL_L_NAME).append("::").append(MONOKL_HEART_NAME));
			linkR = model -> GetLink(linkPrefix + std::string(MONOKL_R_NAME).append("::").append(MONOKL_HEART_NAME));
			
			if(!linkL || !linkR)
			{
				ROS_FATAL_STREAM("Nie udało się znaleźć modeli czujników laserowych");
			}
			
			updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&MonoklChain::OnUpdate, this));
			
			//inicjalizacja ROSa
			if (!ros::isInitialized())
			{
				int argc = 0;
				char **argv = nullptr;
				ros::init(argc, argv, CLIENT_NAME, ros::init_options::NoSigintHandler);
			}
			
			//powiadom o gotowości
			ROS_DEBUG_STREAM("MonoklChain zainicjalizowany");
		}
		
	private:
		void OnUpdate()
		{
			//wyślij ramkę prawego
			ignition::math::Pose3<double> pose = linkR -> WorldPose();
			geometry_msgs::TransformStamped transRMsg;
			transRMsg.header.stamp = ros::Time::now();
			transRMsg.header.frame_id = "map";
			transRMsg.child_frame_id = "monokl_r_heart";
			transRMsg.transform.translation.x = pose.Pos().X();
			transRMsg.transform.translation.y = pose.Pos().Y();
			transRMsg.transform.translation.z = pose.Pos().Z();
			transRMsg.transform.rotation.x = pose.Rot().X();
			transRMsg.transform.rotation.y = pose.Rot().Y();
			transRMsg.transform.rotation.z = pose.Rot().Z();
			transRMsg.transform.rotation.w = pose.Rot().W();
			framePublisher.sendTransform(transRMsg);
			
			//wyślij ramkę lewego
			pose = linkL -> WorldPose();
			geometry_msgs::TransformStamped transLMsg;
			transLMsg.header.stamp = ros::Time::now();
			transLMsg.header.frame_id = "map";
			transLMsg.child_frame_id = "monokl_l_heart";
			transLMsg.transform.translation.x = pose.Pos().X();
			transLMsg.transform.translation.y = pose.Pos().Y();
			transLMsg.transform.translation.z = pose.Pos().Z();
			transLMsg.transform.rotation.x = pose.Rot().X();
			transLMsg.transform.rotation.y = pose.Rot().Y();
			transLMsg.transform.rotation.z = pose.Rot().Z();
			transLMsg.transform.rotation.w = pose.Rot().W();
			framePublisher.sendTransform(transLMsg);
			
			counter++;
		}
		
		///Wskaźnik na model
		physics::ModelPtr model;
		
		///Prawy monokl
		physics::LinkPtr linkR;
		
		///Lewy monokl
		physics::LinkPtr linkL;
		
		//Nadajnik ramki
		tf2_ros::TransformBroadcaster framePublisher;
		
		///Licznik kroków
		unsigned int counter;
		
		///Wskaźnik na zdarzenie aktualizacji
		event::ConnectionPtr updateConnection;
		
		///Node dla ROSa
		std::unique_ptr<ros::NodeHandle> rosNode;
	};
	
	//zarejestruj wtyczkę
	GZ_REGISTER_MODEL_PLUGIN(MonoklChain)
	
}


