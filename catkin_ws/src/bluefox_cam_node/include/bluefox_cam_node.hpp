#ifndef BLUEFOX_CAM_NODE
#define BLUEFOX_CAM_NODE

#include "Stereosystem.h"
#include <image_transport/image_transport.h>
#include <bluefox_cam_node/bluefox_cam_nodeConfig.h>


class bluefox_node{
    using imgPub = image_transport::CameraPublisher const&;
public:
    bluefox_node();
    ~bluefox_node();
    void init_msgs();
    void callback(bluefox_cam_node::bluefox_cam_nodeConfig &config, uint32_t level);
    void publish_image(imgPub pubLeft, imgPub pubRight);
    void view_fps()const;
    void set_exposure(int exposure);
    void set_binning(bool b);
    void reset_image();
    void set_high_pixelclock(bool b);
    int get_image_type()const;
    inline bool get_distorted();
    inline bool get_undistorted();
    inline bool get_rectified();
    // void left_image_loop(ros::NodeHandle const& nh, ros::Rate& loop_rate);
    // void right_image_loop(ros::NodeHandle const& nh, ros::Rate& loop_rate);


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
    sensor_msgs::CameraInfo infoLeft;
    sensor_msgs::CameraInfo infoRight;
    cv::FileStorage fs;
    bool binning;
};

#endif //BLUEFOX_CAM_NODE