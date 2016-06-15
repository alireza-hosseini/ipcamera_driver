#include "mrl_ipcamera/mrl_ipcamera.hpp"
#include <opencv2/video/video.hpp>
#include <sensor_msgs/image_encodings.h>

MrlIpCamera::MrlIpCamera(ros::NodeHandle *nodeHandle):nh_(nodeHandle),
    imagetransport_(*nodeHandle)
{
    camera_pub_= imagetransport_.advertiseCamera("/camera/image",10);
    nh_->param<std::string>("video_url",video_url_,"rtsp://admin:A123456789@192.168.1.64/live.sdp?:network-cache=300");
    ROS_INFO_STREAM("video_url  "<<video_url_);
    nh_->param<std::string>("frame_id",frame_id_,"manipulator_cam_link");
    refresh_serviceServer_ = nh_->advertiseService("refresh",&MrlIpCamera::refreshSrvCallback, this);
    cap_.open(video_url_);
}

MrlIpCamera::~MrlIpCamera()
{

}

bool MrlIpCamera::publish()
{
    cv::Mat frame;
    ros::Rate loop(33);
    while(ros::ok())
    {
        if(cap_.isOpened())
        {
            ROS_INFO_ONCE("connection established");
            if( camera_pub_.getNumSubscribers() > 0 )
            {
                cap_ >> frame;
                if( frame.empty() ) continue;
                cv_bridge::CvImage out_msg;
                out_msg.header.frame_id = frame_id_;
                out_msg.header.stamp = ros::Time::now();
                out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                out_msg.image = frame;

                sensor_msgs::CameraInfo camera_info;
                camera_info.header.frame_id = frame_id_;
                camera_info.header.stamp = ros::Time::now();

                sensor_msgs::Image rosimg;
                out_msg.toImageMsg(rosimg);
                camera_pub_.publish(rosimg,camera_info,ros::Time::now());
            }
        }else
        {
            cap_.release();
            ROS_INFO_STREAM("\n\n\n\n********************************* IP Camera Not OPENED *********************************\n\n\n\n");
            cap_.open(video_url_);
        }

        ros::spinOnce();
        loop.sleep();
    }
    return true;
}

bool MrlIpCamera::refreshSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("refreshing");
    cap_.release();
    if(!cap_.open(video_url_))
    {
        ROS_ERROR_STREAM("Connecting to "<<video_url_<<" failed.");
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"MrlIpCamera");
    ros::NodeHandle nh("~");
    MrlIpCamera ipCamera(&nh);
    ipCamera.publish();
    return 0;
}
