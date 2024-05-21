#ifndef __RKNN_ROS_H__
#define __RKNN_ROS_H__

#include <rknn_ros_msgs/BoundingBox.h>
#include <rknn_ros_msgs/BoundingBoxes.h>
#include <rknn_ros_msgs/CheckForObjectsAction.h>
#include <rknn_ros_msgs/ObjectCount.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <ros/ros.h>
#include "rknn_api.h"
#include "image_utils.h"
#include "yolov5.h"
#include "rknn_api.h"
#include "common.h"
class Rknn{

private:
    //Declare objects for message handling and node
    ros::NodeHandle nh;
    image_transport::Subscriber sub_img_;
    image_transport::ImageTransport it;
    image_transport::Publisher pub_img_;
    ros::Publisher pub_detections;
    

public:
    int ret;
    rknn_app_context_t rknn_app_ctx;
    //Declare constructor/destructor and public function/variables
    Rknn(const char* model_path, ros::NodeHandle &nh_);
    ~Rknn();
    void callback(const sensor_msgs::ImageConstPtr& msg);
    object_detect_result_list od_results;
};

#endif
