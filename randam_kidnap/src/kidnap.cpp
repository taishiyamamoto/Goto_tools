#include<ros/ros.h>
#include<tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PointStamped.h>

#include <string>

class RandamKidnap {
	public:
		RandamKidnap(){
			clicled_point = nh.subscribe("/clicked_point",1,&RandamKidnap::ClickedCallback,this);
			tf_sub = nh.subscribe("/tf",1,&RandamKidnap::TfCallback,this);
			nh.param("robto_frame", robot_frame, std::string("/base_link"));
			nh.param("world_frame", world_frame, std::string("/map"));
		}
		void ClickedCallback(const geometry_msgs::PointStampedConstPtr &point);
		void TfCallback(const tf2_msgs::TFMessage &tf);

	private:
		ros::NodeHandle nh;
		std::string world_frame_;
		ros::Subscriber clicled_point;
		ros::Subscriber tf_sub;

		geometry_msgs::PointStamped position_t;

		tf::TransformListener tf_listener;
		std::string world_frame;
		std::string robot_frame;
		static ros::Time now_time;

};

void RandamKidnap::ClickedCallback(const geometry_msgs::PointStampedConstPtr &point){
	double x= point->point.x;
	ROS_INFO("%lf",x);

}

void RandamKidnap::TfCallback(const tf2_msgs::TFMessage &tf){
	tf::StampedTransform robot_gl;
	static ros::Time saved_time;
	try{
		tf_listener.lookupTransform(world_frame, robot_frame, ros::Time(0.0), robot_gl);

		position_t.point.x = robot_gl.getOrigin().x();
		position_t.point.y = robot_gl.getOrigin().y();
		position_t.point.z = 0;

		//now_time = ros::Time::now();
		if((ros::Time::now() - saved_time).toSec() > 30){
			ROS_INFO("nya");
			saved_time = ros::Time::now();
		}
		//誘拐させる座標の確認
		//geometry_msgs/PoseWithCovarianceStamped
	}catch(tf::TransformException &e){
		ROS_WARN_STREAM(e.what());
	}
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "RandamKidnap");
	RandamKidnap randamKidnap;
	while(ros::ok()){
		ros::spin();
	}
	return 0;
}
