#include "ros_driver.hpp"


int main(int argc, char *argv[]) { 
    ros::init(argc, argv, "ss_lidar");
    ros::NodeHandle nh; 

    dephan_ros::Driver driver(nh, "point_cloud2_data");  
    ROS_INFO("Started"); 

    while(ros::ok()) {
        driver.poll();
        ros::spinOnce();
    }

    return 0; 
}