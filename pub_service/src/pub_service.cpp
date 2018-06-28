#include<ros/ros.h>
#include<std_srvs/Trigger.h>
#include<std_srvs/Empty.h>

#include <string>
#include <random>

class PublishService{
public:
    PublishService(){
        reset_flag = false;
        before_flag = false;
    }
    bool reqStartWpNav(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool reqResumeNav(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool reqKidnapped(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

private:
    ros::NodeHandle nh;
    ros::ServiceClient pub_kidnapped = nh.serviceClient<std_srvs::Empty>("kidnapped");
    ros::ServiceServer start_wp_nav = nh.advertiseService("start_wp_nav", &PublishService::reqStartWpNav,this);
    ros::ServiceServer resume_nav = nh.advertiseService("resume_nav", &PublishService::reqResumeNav,this);
    ros::ServiceServer request_kidnapped = nh.advertiseService("kidnapped_req", &PublishService::reqKidnapped,this);
    bool reset_flag;
    bool before_flag;
};

bool PublishService::reqStartWpNav(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    reset_flag = true;
    ROS_INFO("start navigaion");
    return true;
}

bool PublishService::reqResumeNav(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    before_flag = false;
    ROS_INFO("resume navigaion");
    return true;
}

bool PublishService::reqKidnapped(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    if(reset_flag != before_flag){
        ROS_INFO("Call kidnapped");
    }

    before_flag = true;
    return true;
}

int main(int argc, char *argv[]){
	ros::init(argc, argv, "PublishService");
	PublishService publish_service;
	while(ros::ok()){
		ros::spin();
	}
	return 0;
}
