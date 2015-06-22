#include "bluefox_cam_node.hpp"
// ros includes
#include "ros/ros.h" // ros main
#include <ros/package.h> // package::getPath
#include <cv_bridge/cv_bridge.h> // convert images to image messages
#include <dynamic_reconfigure/server.h> // dynamically change parameters
// framework includes
#include "utility.h"
#include "Camera.h"
#include "Stereosystem.h"

// gets everything ready to run
bluefox_node::bluefox_node(): image_type{0}, left{}, right{}, devMgr{},
                                        nodes{}, stereo{left,right},
                                        imagePair{}, msgLeft{}, msgRight{},
                                        config{ros::package::getPath("bluefox_cam_node") + "/src/mvStereoVision/configs/default.yml"}
    {
        // initialize cameras
        Utility::initCameras(devMgr,left,right);
        stereo={left,right}; // because cameras must be initialized first
        // get input files and configure cameras
        std::string inputParameter;
        nodes.push_back("inputParameter");
        Utility::checkConfig(config,nodes,fs);
        fs["inputParameter"] >> inputParameter;
        stereo.loadIntrinsic(inputParameter+"/intrinsic.yml");
        stereo.loadExtrinisic(inputParameter +"/extrinsic.yml");
        set_exposure(24000);
        imagePair = Stereopair{cv::Mat{left->getImageHeight(),
            left->getImageWidth(),CV_8UC1, cv::Scalar::all(0)},
            cv::Mat{right->getImageHeight(),right->getImageWidth(),
            CV_8UC1, cv::Scalar::all(0)}};
    }

// clean up camera space
bluefox_node::~bluefox_node(){
    delete left;
    delete right;
}

// for switch case
int bluefox_node::get_image_type()const{
    return image_type;
}

// set exposure time of both cameras
void bluefox_node::set_exposure(int exposure){
        left->setExposure(exposure);
        right->setExposure(exposure);
}

// callback function for dynamic reconfigure
void bluefox_node::callback(bluefox_cam_node::bluefox_cam_nodeConfig &config, uint32_t level)
{
    image_type = config.image_type; // distorted / undistorted / rectified
    set_exposure(config.exposure); // exposure time
}

// publishes distorted images in stereo/left/camera and stereo/right/camera
void bluefox_node::publish_distorted(Publisher pubLeft, Publisher pubRight){

        stereo.getImagepair(imagePair);
        msgRight = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mLeft).toImageMsg();
        msgLeft = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mRight).toImageMsg();
        pubLeft.publish(msgLeft);
        pubRight.publish(msgRight);
}

// publishes undistorted images in stereo/left/camera and stereo/right/camera
void bluefox_node::publish_undistorted(Publisher pubLeft, Publisher pubRight){

        stereo.getUndistortedImagepair(imagePair);
        msgRight = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mLeft).toImageMsg();
        msgLeft = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mRight).toImageMsg();
        pubLeft.publish(msgLeft);
        pubRight.publish(msgRight);
}

// publishes rectified images in stereo/left/camera and stereo/right/camera
void bluefox_node::publish_rectified(Publisher pubLeft, Publisher pubRight){
        stereo.getRectifiedImagepair(imagePair);
        msgRight = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mLeft).toImageMsg();
        msgLeft = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mRight).toImageMsg();
        pubLeft.publish(msgLeft);
        pubRight.publish(msgRight);
}

// fps output on console
void bluefox_node::view_fps()const{
    std::cout << "left: " << left->getFramerate() << " right: " << right->getFramerate() << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "bluefox_node"); // initialize ros::node
    ros::NodeHandle nh; // process will run while Nodehandle is okay/exists

    auto cam_node = new bluefox_node{}; // construct node object

    // initialize dynamic reconfigure stuff and bind the callback function
    dynamic_reconfigure::Server<bluefox_cam_node::bluefox_cam_nodeConfig> srv;
    dynamic_reconfigure::Server<bluefox_cam_node::bluefox_cam_nodeConfig>::CallbackType f;
    f = boost::bind(&bluefox_node::callback, cam_node, _1, _2);
    srv.setCallback(f);

    // initialize image_transport publishers and set publishing channels
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pubLeft = it.advertise("stereo/left/image", 1);
    image_transport::Publisher pubRight = it.advertise("stereo/right/image", 1);

    // set ros loop rate
    ros::Rate loop_rate(200);

    while (nh.ok()) {

        cam_node->view_fps();
        //switch case to change image type dynamically
        switch(cam_node->get_image_type()){
            case 0:{ // distorted
                cam_node->publish_distorted(pubLeft, pubRight);
                break;
            }
            case 1:{ // undistorted
                cam_node->publish_undistorted(pubLeft, pubRight);
                break;
            }
            case 2:{ // rectified
                cam_node->publish_rectified(pubLeft, pubRight);
                break;
            }
            default:{
                std::cout << "image types: distorted, unddistorted, rectified" << std::endl;
            }
        }
        ros::spinOnce(); // call all callbacks in the ros callback queue
        loop_rate.sleep(); // sleep untill next loop cycle begins
    }
  return 0;
}