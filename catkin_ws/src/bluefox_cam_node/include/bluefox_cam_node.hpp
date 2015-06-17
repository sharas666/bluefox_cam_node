#ifndef BLUEFOX_CAM_NODE
#define BLUEFOX_CAM_NODE

#include <ros/package.h>
#include "std_msgs/String.h"
#include "utility.h"
#include "Camera.h"
#include "Stereosystem.h"
#include "bluefox_cam_node.hpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <bluefox_cam_node/bluefox_cam_nodeConfig.h>


class bluefox_node{
public:
    bluefox_node();
    void callback(bluefox_cam_node::bluefox_cam_nodeConfig &config, uint32_t level);
    void publish_distorted(image_transport::Publisher const& pubLeft, image_transport::Publisher const& pubRight);
    void publish_undistorted(image_transport::Publisher const& pubLeft, image_transport::Publisher const& pubRight);
    void publish_rectified(image_transport::Publisher const& pubLeft, image_transport::Publisher const& pubRight);
private:
    int image_type;
    Camera* left;
    Camera* right;
    mvIMPACT::acquire::DeviceManager devMgr;
    std::vector<std::string> nodes;
    Stereosystem stereo;
    Stereopair imagePair;
    sensor_msgs::ImagePtr msgLeft;
    sensor_msgs::ImagePtr msgRight;
    std::string config;
    cv::FileStorage fs;
};

#endif //BLUEFOX_CAM_NODE