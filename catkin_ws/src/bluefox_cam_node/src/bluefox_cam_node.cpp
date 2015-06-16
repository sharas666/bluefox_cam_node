#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "utility.h"
#include "Camera.h"
#include "Stereosystem.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bluefox_cam_node");
    ros::NodeHandle nh;
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


    //create windows
/*    cv::namedWindow("Left Distorted" ,1);
    cv::namedWindow("Right Distorted" ,1);

    cv::namedWindow("Left Undistorted" ,1);
    cv::namedWindow("Right Undistorted" ,1);

    cv::namedWindow("Left Rectified" ,1);
    cv::namedWindow("Right Rectified" ,1);*/

    sensor_msgs::ImagePtr msgLeft;
    sensor_msgs::ImagePtr msgRight;
    ros::Rate loop_rate(60);
    //Stereopair is a struct holding a left and right image
    Stereopair distorted;
/*    Stereopair undistorted;
    Stereopair rectified;*/


    while (nh.ok()) {
        //get the different types of images from the stereo system...
        stereo.getImagepair(distorted);
        //stereo.getUndistortedImagepair(undistorted);
        //stereo.getRectifiedImagepair(rectified);

        //... and show them all (with the stereopair.mLeft/mRight you have acces to the images and can do whatever you want with them)
/*        cv::imshow("Left Distorted", distorted.mLeft);
        cv::imshow("Right Distorted", distorted.mRight);

        cv::imshow("Left Undistorted", undistorted.mLeft);
        cv::imshow("Right Undistorted", undistorted.mRight);

        cv::imshow("Left Rectified", rectified.mLeft);
        cv::imshow("Right Rectified", rectified.mRight);*/


        cv::waitKey(30);
        msgRight = cv_bridge::CvImage(std_msgs::Header(), "mono8", distorted.mLeft).toImageMsg();
        msgLeft = cv_bridge::CvImage(std_msgs::Header(), "mono8", distorted.mRight).toImageMsg();
/*        msg->height = distorted.mLeft.rows;
        msg->width = distorted.mLeft.cols;
*/        //msg->step = distorted.mLeft.step[0];
        //std::cout << msg->height << " " << msg->width << " " << msg->step << std::endl;
        /*std::cout << msg->step << std::endl;*/
        pubLeft.publish(msgLeft);
        pubRight.publish(msgRight);
        ros::spinOnce();
        loop_rate.sleep();
  }
  return 0;
}