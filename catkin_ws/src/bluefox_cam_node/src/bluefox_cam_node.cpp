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
                                        config{ros::package::getPath("bluefox_cam_node") + "/src/mvStereoVision/configs/default.yml"},
                                        infoLeft{}, infoRight{}, fs{}, binning{0}
    {
        // initialize cameras
        Utility::initCameras(devMgr,left,right);
        stereo={left,right}; // because cameras must be initialized
        // get input files and configure cameras
        std::string inputParameter;
        nodes.push_back("inputParameter");
        Utility::checkConfig(config,nodes,fs);
        fs["inputParameter"] >> inputParameter;
        stereo.loadIntrinsic(inputParameter+"/intrinsic.yml");
        stereo.loadExtrinisic(inputParameter +"/extrinsic.yml");
        set_exposure(24000);
        // the cv::mat have to be initialized with the image size and 0 values
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

// this does not work!
void bluefox_node::init_msgs(){
        infoLeft.height=left->getImageHeight();
        infoLeft.width=left->getImageWidth();
        infoRight.height=left->getImageHeight();
        infoRight.width=left->getImageWidth();

        // std::cout << infoLeft.K[0] << std::endl;

        // auto intrinsic = left->getIntrinsic();
        // std::copy(intrinsic.datastart,intrinsic.dataend,infoLeft.K.begin());
        // intrinsic = right->getIntrinsic();
        // std::copy(intrinsic.datastart,intrinsic.dataend,infoRight.K.begin());
        auto distCoeffsL = left->getDistCoeffs();
        std::vector<double> tempL{distCoeffsL.row(0)};
        std::copy(tempL.begin(), tempL.end(), std::back_inserter(infoLeft.D));
        infoLeft.D.resize(5);
        auto distCoeffsR = right->getDistCoeffs();
        std::vector<double> tempR{distCoeffsR.row(0)};
        std::copy(tempR.begin(), tempR.end(), std::back_inserter(infoRight.D));
        infoRight.D.resize(5);
        infoLeft.distortion_model = "plumb_bob";
        infoRight.distortion_model = infoLeft.distortion_model;

}

// lazy implementation because of enums. Look up matrix vision documentation for a full enum list.
void bluefox_node::set_high_pixelclock(bool b){
    if (b==true){
            left->setHighPixelClock();
            right->setHighPixelClock();
    }
    if (b==false){
        left->setNormalPixelClock();
        right->setNormalPixelClock();
    }
}

bool bluefox_node::get_distorted(){
    return stereo.getImagepair(imagePair);
}
bool bluefox_node::get_undistorted(){
    stereo.getImagepair(imagePair);
    return stereo.undistort_images(imagePair);
}
bool bluefox_node::get_rectified(){
    stereo.getImagepair(imagePair);
    stereo.undistort_images(imagePair);
    return stereo.rectify_images(imagePair);
}

// for switch case
int bluefox_node::get_image_type()const{
    return image_type;
}

// get the current image size from camera and reinitiate imagePair
void bluefox_node::reset_image(){
    left->set_size();
    right->set_size();
    imagePair = Stereopair{cv::Mat{left->getImageHeight(),
        left->getImageWidth(),CV_8UC1, cv::Scalar::all(0)},
        cv::Mat{right->getImageHeight(),right->getImageWidth(),
        CV_8UC1, cv::Scalar::all(0)}};
}

// set binning and adjust image size
void bluefox_node::set_binning(bool b){
    if (b==true){
        left->setBinning(BINNING_HV);
        right->setBinning(BINNING_HV);
        reset_image();
        init_msgs();
        // camera_info msg
        infoLeft.binning_x = 2;
        infoLeft.binning_y = 2;
        infoRight.binning_x = 2;
        infoRight.binning_y = 2;
    }
    else{
        left->setBinning(BINNING_OFF);
        right->setBinning(BINNING_OFF);
        reset_image();
        init_msgs();
        // camera_info msg
        infoLeft.binning_x = 1;
        infoLeft.binning_y = 1;
        infoRight.binning_x = 1;
        infoRight.binning_y = 1;
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
    set_binning(config.binning_mode); // binning on / off
    set_high_pixelclock(config.clock_mode); // normal / high
}

// publishes the Imagepair
void bluefox_node::publish_image(imgPub pubLeft, imgPub pubRight){
        // write ImageMsg
        msgRight = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mLeft).toImageMsg();
        msgLeft = cv_bridge::CvImage(std_msgs::Header(),
                     "mono8", imagePair.mRight).toImageMsg();
        // timestamp and publish
        static auto t=ros::Time::now();
        pubLeft.publish(*msgLeft, infoLeft, t);
        pubRight.publish(*msgRight, infoRight, t);
}

// fps output on console
void bluefox_node::view_fps() const{
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
    image_transport::CameraPublisher pubLeft = it.advertiseCamera("stereo/left/image_raw", 1);
    image_transport::CameraPublisher pubRight = it.advertiseCamera("stereo/right/image_raw", 1);

    // set ros loop rate
    ros::Rate loop_rate(90);

    while (nh.ok()) {
        cam_node->view_fps();

        //switch case to change image type dynamically
        switch(cam_node->get_image_type()){
            case 0:{ // distorted
                cam_node->get_distorted();
                cam_node->publish_image(pubLeft, pubRight);
                break;
            }
            case 1:{ // undistorted
                cam_node->get_undistorted();
                cam_node->publish_image(pubLeft, pubRight);
                break;
            }
            case 2:{ // rectified
                cam_node->get_rectified();
                cam_node->publish_image(pubLeft, pubRight);
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