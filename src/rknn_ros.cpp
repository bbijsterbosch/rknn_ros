#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rknn_ros_msgs/BoundingBox.h>
#include <rknn_ros_msgs/BoundingBoxes.h>
#include <rknn_ros_msgs/CheckForObjectsAction.h>
#include <rknn_ros_msgs/ObjectCount.h>
#include <sensor_msgs/Image.h>
#include "yolov5.h"
#include "image_utils.h"
#include "image_drawing.h"
#include "rknn_ros.h"
#include "postprocess.h"
#include "common.h"



Rknn::Rknn(const char* model_path, ros::NodeHandle &nh_) : nh(nh_), it(nh_){

    memset(&rknn_app_ctx, 0, sizeof(rknn_app_context_t));
    init_post_process();
    ret = init_yolov5_model(model_path, &rknn_app_ctx);
    if (ret != 0)
    {
        printf("init_yolov5_model fail! ret=%d model_path=%s\n", ret, model_path);
        goto out;
    }

    sub_img_ = it.subscribe("/mirte/camera/image_raw", 1, &Rknn::callback, this);
    pub_img_ = it.advertise("/rknn_node/image",1);
    pub_detections = nh.advertise<rknn_ros_msgs::BoundingBoxes>("/rknn_ros_node/detections", 1);

    out:
    deinit_post_process();

    ret = release_yolov5_model(&rknn_app_ctx);
    if (ret != 0)
    {
        printf("release_yolov5_model fail! ret=%d\n", ret);
    }
}

Rknn::~Rknn(){
    deinit_post_process();
    ret = release_yolov5_model(&rknn_app_ctx);
    }


void Rknn::callback(const sensor_msgs::ImageConstPtr& src_img){
    char text[256];
    rknn_ros_msgs::BoundingBoxes boundingBoxesResults_;
    cv_bridge::CvImagePtr cv_ptr;
    sensor_msgs::Image bbox_img;
    bbox_img.is_bigendian = src_img->is_bigendian;
    bbox_img.step = src_img->step;
    bbox_img.header.seq = src_img->header.seq;
    bbox_img.header.frame_id = src_img->header.frame_id;
    //Convert ROS message to OpenCV object
    try{
        cv_ptr = cv_bridge::toCvCopy(src_img, sensor_msgs::image_encodings::RGB8);
    }

    //check if image is converted
    catch(cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", src_img->encoding.c_str());
        return;
    }
        
    //Check if image contains data, if not, skip proccessing
    if (!cv_ptr->image.data) {
        ROS_WARN("Empty image. Skipping processing.");
        return; 
    }
   

    //Person Detection algorithm
    else {
        image_buffer_t rknn_image;
        image_buffer_t* rknn_ptr = &rknn_image;
        memset(rknn_ptr, 0, sizeof(image_buffer_t));
        object_detect_result_list od_results;
        cv::Mat img = cv_ptr->image;
        size_t data_size = img.total() * img.elemSize();
        unsigned char *data = (unsigned char *)malloc(data_size);
        if (data == NULL) {
            printf("Memory allocation failed\n");
        }

        // Copy the image data to the buffer
        

        // Set the image buffer fields
        rknn_ptr->virt_addr = data;
        rknn_ptr->size = data_size;
        
        if (ret != 0){
            ROS_ERROR("Something went wrong with image conversion");
            
        }
        ret = inference_yolov5_model(&rknn_app_ctx, rknn_ptr, &od_results);
        if (ret != 0)
        {
            printf("init_yolov5_model fail! ret=%d\n", ret);
        }
        for (int i = 0; i < od_results.count; i++)
        {
            rknn_ros_msgs::BoundingBox boundingBox;
            object_detect_result *det_result = &(od_results.results[i]);
            printf("%s @ (%d %d %d %d) %.3f\n", coco_cls_to_name(det_result->cls_id),
                det_result->box.left, det_result->box.top,
                det_result->box.right, det_result->box.bottom,
                det_result->prop);
            int x1 = det_result->box.left;
            int y1 = det_result->box.top;
            int x2 = det_result->box.right;
            int y2 = det_result->box.bottom;

            draw_rectangle(rknn_ptr, x1, y1, x2 - x1, y2 - y1, COLOR_BLUE, 3);

            sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), det_result->prop * 100);
            draw_text(rknn_ptr, text, x1, y1 - 20, COLOR_RED, 10);
            boundingBox.Class = coco_cls_to_name(det_result->cls_id);
            boundingBox.id = i;
            boundingBox.probability = det_result->prop;
            boundingBox.xmin = x1;
            boundingBox.ymin = y2;
            boundingBox.xmax = x2;
            boundingBox.ymax = y1;
            boundingBoxesResults_.bounding_boxes.push_back(boundingBox);
        }
    
        bbox_img.header.stamp = ros::Time::now();
        bbox_img.height = rknn_ptr->height;
        bbox_img.width = rknn_ptr->width;
        bbox_img.encoding = "rgb8";
        //bbox_img.data = rknn_ptr->virt_addr;


        pub_detections.publish(boundingBoxesResults_);
        pub_img_.publish(bbox_img);
    }
}

