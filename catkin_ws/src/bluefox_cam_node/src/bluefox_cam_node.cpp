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
                                        infoLeft{}, infoRight{},
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
        init_msgs();
    }

// clean up camera space
bluefox_node::~bluefox_node(){
    delete left;
    delete right;
}

void bluefox_node::init_msgs(){
        infoLeft.height=left->getImageHeight();
        infoLeft.width=left->getImageWidth();
        infoRight.height=left->getImageHeight();
        infoRight.width=left->getImageWidth();
        auto intrinsic = left->getIntrinsic();
        std::copy(intrinsic.datastart,intrinsic.dataend,infoLeft.K.begin());
        intrinsic = right->getIntrinsic();
        std::copy(intrinsic.datastart,intrinsic.dataend,infoRight.K.begin());
}

bool bluefox_node::get_distorted(){
    return stereo.getImagepair(imagePair);
}
bool bluefox_node::get_undistorted(){
    return stereo.getUndistortedImagepair(imagePair);
}
bool bluefox_node::get_rectified(){
    return stereo.getRectifiedImagepair(imagePair);
}

// for switch case
int bluefox_node::get_image_type()const{
    return image_type;
}

void bluefox_node::reset_image(){
    left->set_size();
    right->set_size();
    imagePair = Stereopair{cv::Mat{left->getImageHeight(),
        left->getImageWidth(),CV_8UC1, cv::Scalar::all(0)},
        cv::Mat{right->getImageHeight(),right->getImageWidth(),
        CV_8UC1, cv::Scalar::all(0)}};
}

void bluefox_node::set_binning(bool b){
    if (b==true){
        left->setBinning(BINNING_HV);
        right->setBinning(BINNING_HV);
        reset_image();
        init_msgs();
    }
    else{
        left->setBinning(BINNING_OFF);
        right->setBinning(BINNING_OFF);
        reset_image();
        init_msgs();
    }
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
    set_binning(config.binning_mode);
}

// publishes the Imagepair
void bluefox_node::publish_image(imgPub pubLeft, imgPub pubRight, ros::Publisher& info_l, ros::Publisher& info_r){

        msgRight = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mLeft).toImageMsg();
        msgLeft = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mRight).toImageMsg();
        auto t=ros::Time::now();
        infoLeft.header.stamp=t;
        infoRight.header.stamp=t;
        msgLeft->header.stamp=t;
        msgRight->header.stamp=t;
        info_l.publish(infoLeft);
        info_r.publish(infoRight);
        pubLeft.publish(msgLeft);
        pubRight.publish(msgRight);
}



// fps output on console
void bluefox_node::view_fps()const{
    std::cout << "left: " << left->getFramerate() << " right: " << right->getFramerate() << std::endl;
}

/*void bluefox_node::left_image_loop(ros::NodeHandle const& nh, ros::Rate& loop_rate){
    while(nh.ok()){
        left->getImage(imagePair.mLeft);
        ros::spinOnce(); // call all callbacks in the ros callback queue
        loop_rate.sleep(); // sleep untill next loop cycle begins
    }
}

void bluefox_node::right_image_loop(ros::NodeHandle const& nh, ros::Rate& loop_rate){
    while(nh.ok()){
        left->getImage(imagePair.mRight);
        std::cout << "a"<< std::endl;
        //view_fps();
        ros::spinOnce(); // call all callbacks in the ros callback queue
        loop_rate.sleep(); // sleep untill next loop cycle begins
    }
}*/

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
    image_transport::Publisher pubLeft = it.advertise("stereo/left/image_raw", 1);
    image_transport::Publisher pubRight = it.advertise("stereo/right/image_raw", 1);
    ros::Publisher pub_info_left = nh.advertise<sensor_msgs::CameraInfo>("stereo/left/camera_info", 1);
    ros::Publisher pub_info_right = nh.advertise<sensor_msgs::CameraInfo>("stereo/right/camera_info", 1);

    // set ros loop rate
    ros::Rate loop_rate(90);

    while (nh.ok()) {
        cam_node->view_fps();

        //switch case to change image type dynamically
        switch(cam_node->get_image_type()){
            case 0:{ // distorted
                cam_node->get_distorted();
                cam_node->publish_image(pubLeft, pubRight, pub_info_left, pub_info_right);
                break;
            }
            case 1:{ // undistorted
                cam_node->get_undistorted();
                cam_node->publish_image(pubLeft, pubRight, pub_info_left, pub_info_right);
                break;
            }
            case 2:{ // rectified
                cam_node->get_rectified();
                cam_node->publish_image(pubLeft, pubRight, pub_info_left, pub_info_right);
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