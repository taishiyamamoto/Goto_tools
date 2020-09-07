#include<ros/ros.h>
#include<tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <jsk_rviz_plugins/OverlayMenu.h>

#include <string>
#include <random>

class RandamKidnap {
	public:
		RandamKidnap(){
			tf_sub = nh.subscribe("/tf",1,&RandamKidnap::TfCallback,this);
			pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose",1);
			menu_pub_ = nh.advertise<jsk_rviz_plugins::OverlayMenu>("kidnap_menu", 1);
			gnss_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("gps/position",1,&RandamKidnap::GnssCallback,this);
			gnss_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("gps/position_kidnap",1);
			nh.param("robto_frame", robot_frame, std::string("/base_link"));
			nh.param("world_frame", world_frame, std::string("/map"));
			//乱数の初期化

			create_noise_flag_ = false;
			kidnap_flag_ = false;

    		menu.action = jsk_rviz_plugins::OverlayMenu::ACTION_SELECT;
			menu.current_index = 0;
    		menu.menus.resize(1);
    		menu.title = "Pseudo Kidnap Counter";
			kidnap_counter_ = 0;
		}
		void ClickedCallback(const geometry_msgs::PointStampedConstPtr &point);
		void TfCallback(const tf2_msgs::TFMessage &tf);

	private:
		ros::NodeHandle nh;
		std::string world_frame_;
		ros::Subscriber tf_sub;
		ros::Publisher pose_pub;
		ros::Subscriber gnss_sub_;
		ros::Publisher gnss_pub_;
		ros::Publisher menu_pub_;
		jsk_rviz_plugins::OverlayMenu menu;
		geometry_msgs::PoseWithCovarianceStamped pose_t;

		tf::TransformListener tf_listener;
		std::string world_frame;
		std::string robot_frame;
		static ros::Time now_time;
		bool create_noise_flag_, kidnap_flag_;

		int kidnap_counter_;

		std::random_device seed_gen;
		double CreateRanramNum(double min, double max);
		void GnssCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& gnss_msg);
};

void
RandamKidnap::GnssCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& gnss_msg){
	static ros::Time saved_time;
	geometry_msgs::PoseWithCovarianceStamped gnss;
	gnss = *gnss_msg;

	double kidnap_dis_x,kidnap_dis_y,dis;

	do{
		//ループは範囲の都合上行っている
		kidnap_dis_x = CreateRanramNum(-15.0, 15.0);
		kidnap_dis_y = CreateRanramNum(-15.0, 15.0);
		dis = sqrt(kidnap_dis_x * kidnap_dis_x + kidnap_dis_y + kidnap_dis_y);
		//ROS_INFO("%lf,%lf",kidnap_dis_x, kidnap_dis_y);
		//ROS_INFO("%lf",dis);
	}while(dis <= 10.0);

	gnss.pose.pose.position.x += kidnap_dis_x;
	gnss.pose.pose.position.y += kidnap_dis_y;

	/*
	if((ros::Time::now() - saved_time).toSec() > 10 && (ros::Time::now() - saved_time).toSec() < 20){
		
		if(!create_noise_flag_){
		double kidnap_dis_x = CreateRanramNum(-10.0, 10.0);
		double kidnap_dis_y = CreateRanramNum(-10.0, 10.0);
			create_noise_flag_ = true;
		}
		gnss.pose.pose.position.x += kidnap_dis_x;
		gnss.pose.pose.position.y += kidnap_dis_y;

		kidnap_flag_ = true;
	
	}
	else if(kidnap_flag_){
		saved_time = ros::Time::now();
		create_noise_flag_ = false;
		kidnap_flag_ = false;
	}

	ROS_INFO("%lf",(ros::Time::now() - saved_time).toSec());
	*/

	gnss_pub_.publish(gnss);
	
}


void RandamKidnap::TfCallback(const tf2_msgs::TFMessage &tf){
	tf::StampedTransform robot_gl;
	static ros::Time saved_time;
	try{
		tf_listener.lookupTransform(world_frame, robot_frame, ros::Time(0.0), robot_gl);

		pose_t.pose.pose.position.x = robot_gl.getOrigin().x();
		pose_t.pose.pose.position.y = robot_gl.getOrigin().y();
		pose_t.pose.pose.position.z = 0;
		//2D Pose Estimate cocariance
		pose_t.pose.covariance[0] = pose_t.pose.covariance[7] = 0.25;
		pose_t.pose.covariance[35] = 0.7;
		//now_time = ros::Time::now();
		if((ros::Time::now() - saved_time).toSec() > 30){
			//ROS_INFO("nya");
			saved_time = ros::Time::now();

			//距離の誘拐
			//誘拐距離は3〜60ｍ
			double kidnap_dis_x,kidnap_dis_y,dis;
			//int kidnap_dis_x,kidnap_dis_y;
			//int dis;

			/*
			do{
				//ループは範囲の都合上行っている
				kidnap_dis_x = CreateRanramNum(-10.0, 10.0);
				kidnap_dis_y = CreateRanramNum(-10.0, 10.0);
				dis = sqrt(kidnap_dis_x * kidnap_dis_x + kidnap_dis_y + kidnap_dis_y);
				ROS_INFO("%lf,%lf",kidnap_dis_x, kidnap_dis_y);
				ROS_INFO("%lf",dis);
			}while(dis <= 3.0);
			*/
			kidnap_dis_x = CreateRanramNum(-50.0, 50.0);
			kidnap_dis_y = CreateRanramNum(-50.0, 50.0);
			dis = sqrt(kidnap_dis_x * kidnap_dis_x + kidnap_dis_y * kidnap_dis_y);
			ROS_INFO("%lf,%lf",kidnap_dis_x, kidnap_dis_y);
			ROS_INFO("%lf",dis);
			pose_t.pose.pose.position.x += kidnap_dis_x;
			pose_t.pose.pose.position.y += kidnap_dis_y;

			//角度の誘拐
			//範囲は[0,2π)でヨー軸のみ
			double kidnap_yaw = CreateRanramNum(0, 6.28);
			tf::Quaternion tf_quat=tf::createQuaternionFromRPY(0,0,kidnap_yaw);
			geometry_msgs::Quaternion geo_quat;
			quaternionTFToMsg(tf_quat,geo_quat);
			pose_t.pose.pose.orientation = geo_quat;

			kidnap_counter_++;
			std::stringstream kidnap_ss; 
      		kidnap_ss << kidnap_counter_;

      		menu.menus[0] = kidnap_ss.str();
			menu_pub_.publish(menu);

			pose_pub.publish(pose_t);
		}

	}catch(tf::TransformException &e){
		ROS_WARN_STREAM(e.what());
	}
}

double RandamKidnap::CreateRanramNum(double min, double max){
	std::default_random_engine engine(seed_gen());
	std::uniform_real_distribution<> randam_num(min, max);
	//std::uniform_int_distribution<> randam_num(min, max);
	return randam_num(engine);
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "RandamKidnap");
	RandamKidnap randamKidnap;
	while(ros::ok()){
		ros::spin();
	}
	return 0;
}
