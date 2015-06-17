#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "utility.h"
#include "Camera.h"
#include "Stereosystem.h"
//#include <functional> //std::bind

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <bluefox_cam_node/bluefox_cam_nodeConfig.h>

void callback(bluefox_cam_node::bluefox_cam_nodeConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request : %i",
           config.image_type);
}

void publish_distorted(ros::NodeHandle const& nh, Stereosystem& stereo, ros::Rate& loop_rate, image_transport::Publisher const& pubLeft, image_transport::Publisher const& pubRight){

    Stereopair imagePair;
    sensor_msgs::ImagePtr msgLeft;
    sensor_msgs::ImagePtr msgRight;

    while (nh.ok()) {
        //get the different types of images from the stereo system.
        stereo.getImagepair(imagePair);
        cv::waitKey(30);
        msgRight = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mLeft).toImageMsg();
        msgLeft = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mRight).toImageMsg();
        pubLeft.publish(msgLeft);
        pubRight.publish(msgRight);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void publish_undistorted(ros::NodeHandle const& nh, Stereosystem& stereo, ros::Rate& loop_rate, image_transport::Publisher const& pubLeft, image_transport::Publisher const& pubRight){

    Stereopair imagePair;
    sensor_msgs::ImagePtr msgLeft;
    sensor_msgs::ImagePtr msgRight;

    while (nh.ok()) {
        //get the different types of images from the stereo system.
        stereo.getUndistortedImagepair(imagePair);
        cv::waitKey(30);
        msgRight = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mLeft).toImageMsg();
        msgLeft = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mRight).toImageMsg();
        pubLeft.publish(msgLeft);
        pubRight.publish(msgRight);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void publish_rectified(ros::NodeHandle const& nh, Stereosystem& stereo, ros::Rate& loop_rate, image_transport::Publisher const& pubLeft, image_transport::Publisher const& pubRight){

    Stereopair imagePair;
    sensor_msgs::ImagePtr msgLeft;
    sensor_msgs::ImagePtr msgRight;

    while (nh.ok()) {

        //get the different types of images from the stereo system.
        stereo.getRectifiedImagepair(imagePair);
        cv::waitKey(30);
        msgRight = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mLeft).toImageMsg();
        msgLeft = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mRight).toImageMsg();
        pubLeft.publish(msgLeft);
        pubRight.publish(msgRight);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bluefox_cam_node");
    ros::NodeHandle nh;

    dynamic_reconfigure::Server<bluefox_cam_node::bluefox_cam_nodeConfig> srv;
    dynamic_reconfigure::Server<bluefox_cam_node::bluefox_cam_nodeConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    srv.setCallback(f);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pubLeft = it.advertise("stereo/left/image", 1);
    image_transport::Publisher pubRight = it.advertise("stereo/right/image", 1);
    // image_transport::Publisher pub = it.advertise("stereo/right/image", 1);

    //create a device manager for the matrix vision cameras
    mvIMPACT::acquire::DeviceManager devMgr;

    //create cameras
    Camera *left;
    Camera *right;

    //all the driver stuff from matrix vision fo rthe cameras is done here
    if(!Utility::initCameras(devMgr,left,right))
      return 0;


    //init the stereo system AFTER init the cameras!
    Stereosystem stereo(left,right);

    //collect some configuration parameter written in /configs/default.yml to set "something"
    std::vector<std::string> nodes;
    //"something" is here the extrinsic and intrinsic parameters
    nodes.push_back("inputParameter");

    std::string packagePath = ros::package::getPath("bluefox_cam_node");
    std::cout << packagePath << std::endl;
    std::string config = ros::package::getPath("bluefox_cam_node") + "/src/mvStereoVision/configs/default.yml";

    cv::FileStorage fs;

    //check if the config has all the collected configuration parameters
    if(!Utility::checkConfig(config,nodes,fs))
    {
      return 0;
    }

    //put the collected paramters to some variable
    std::string inputParameter;
    fs["inputParameter"] >> inputParameter;

    //load the camera matrices, dist coeffs, R, T, ....
    if(!stereo.loadIntrinsic(inputParameter+"/intrinsic.yml"))
      return 0;
    if(!stereo.loadExtrinisic(inputParameter +"/extrinsic.yml"))
      return 0;

    //set the exposure time for left and right camera
    left->setExposure(24000);
    right->setExposure(24000);


    ros::Rate loop_rate(60);
    //Stereopair is a struct holding a left and right image

    int img_type = 0;

    switch(img_type){
        case 0:{
            publish_distorted(nh, stereo, loop_rate, pubLeft, pubRight);
            break;
        }
        case 1:{
            publish_undistorted(nh, stereo, loop_rate, pubLeft, pubRight);
            break;
        }
        case 2:{
            publish_rectified(nh, stereo, loop_rate, pubLeft, pubRight);
            break;
        }
        default:{
            std::cout << "image types: distorted, unddistorted, rectified" << std::endl;
        }
    }




  return 0;
}