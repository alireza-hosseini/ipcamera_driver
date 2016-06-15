#ifndef MRL_IPCAMERA
#define MRL_IPCAMERA

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>

class MrlIpCamera
{
public:
    MrlIpCamera(ros::NodeHandle *nodeHandle);
    ~MrlIpCamera();
    bool publish();
    bool refreshSrvCallback(std_srvs::Empty::Request &req,
                        std_srvs::Empty::Response &res);
private:
    ros::NodeHandle *nh_;
    image_transport::ImageTransport imagetransport_;
    image_transport::CameraPublisher camera_pub_;
    std::string video_url_;
    std::string frame_id_;
    ros::ServiceServer refresh_serviceServer_;
    cv::VideoCapture cap_;
};

#endif
