#include "ros/ros.h"
#include "bluefox_cam_node.hpp"
/*#include <ros/package.h>
#include "std_msgs/String.h"
#include "utility.h"
#include "Camera.h"
#include "Stereosystem.h"
#include "bluefox_node.hpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <bluefox_cam_node/bluefox_nodeConfig.h>*/

bluefox_node::bluefox_node(): image_type{0}, left{}, right{}, devMgr{},
                                        nodes{}, stereo(left,right),
                                        imagePair{}, msgLeft{}, msgRight{},
                                        config{ros::package::getPath("bluefox_node") + "/src/mvStereoVision/configs/default.yml"}

    {
        Utility::initCameras(devMgr,left,right);
        nodes.push_back("inputParameter");
        // stereo{left,right};
        Utility::checkConfig(config,nodes,fs);
        //put the collected paramters to some variable
        std::string inputParameter;
        fs["inputParameter"] >> inputParameter;

        //load the camera matrices, dist coeffs, R, T, ....
        stereo.loadIntrinsic(inputParameter+"/intrinsic.yml");
        stereo.loadExtrinisic(inputParameter +"/extrinsic.yml");

        //set the exposure time for left and right camera
        left->setExposure(24000);
        right->setExposure(24000);
    }

void bluefox_node::callback(bluefox_cam_node::bluefox_cam_nodeConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure request : %i",
           config.image_type);
    image_type = config.image_type;
}

void bluefox_node::publish_distorted(image_transport::Publisher const& pubLeft, image_transport::Publisher const& pubRight){

        stereo.getImagepair(imagePair);
        msgRight = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mLeft).toImageMsg();
        msgLeft = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mRight).toImageMsg();
        pubLeft.publish(msgLeft);
        pubRight.publish(msgRight);
}

void bluefox_node::publish_undistorted(image_transport::Publisher const& pubLeft, image_transport::Publisher const& pubRight){

        stereo.getImagepair(imagePair);
        msgRight = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mLeft).toImageMsg();
        msgLeft = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mRight).toImageMsg();
        pubLeft.publish(msgLeft);
        pubRight.publish(msgRight);
}

void bluefox_node::publish_rectified(image_transport::Publisher const& pubLeft, image_transport::Publisher const& pubRight){

        stereo.getImagepair(imagePair);
        msgRight = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mLeft).toImageMsg();
        msgLeft = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mRight).toImageMsg();
        pubLeft.publish(msgLeft);
        pubRight.publish(msgRight);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "bluefox_node");
    ros::NodeHandle nh;

    auto cam_node = new bluefox_node{};

    dynamic_reconfigure::Server<bluefox_cam_node::bluefox_cam_nodeConfig> srv;
    dynamic_reconfigure::Server<bluefox_cam_node::bluefox_cam_nodeConfig>::CallbackType f;
    f = boost::bind(&bluefox_node::callback, cam_node, _1, _2);
    srv.setCallback(f);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pubLeft = it.advertise("stereo/left/image", 1);
    image_transport::Publisher pubRight = it.advertise("stereo/right/image", 1);


    ros::Rate loop_rate(60);
    //Stereopair is a struct holding a left and right image

    while (nh.ok()) {
        //get the different types of images from the stereo system.
        cv::waitKey(30);

        switch(0){
            case 0:{
                cam_node->publish_distorted(pubLeft, pubRight);
                break;
            }
            case 1:{
                cam_node->publish_undistorted(pubLeft, pubRight);
                break;
            }
            case 2:{
                cam_node->publish_rectified(pubLeft, pubRight);
                break;
            }
            default:{
                std::cout << "image types: distorted, unddistorted, rectified" << std::endl;
            }
        }
        ros::spinOnce();
        loop_rate.sleep();
    }



  return 0;
}