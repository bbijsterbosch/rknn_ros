#include <ros/ros.h>
#include <rknn_ros.h>
#include <stdio.h>
int main(int argc, char** argv) {

    ros::init(argc, argv, "rknn_ros_node");
    ros::NodeHandle nodeHandle;
    const char *model_path = "../model/yolov5.rknn";
    Rknn rknn(model_path, nodeHandle);

    ros::spin();
    return 0;
}