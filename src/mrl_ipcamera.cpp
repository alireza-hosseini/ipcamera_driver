/*
 * Copyright (c) 2018, Alireza Hosseini.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Alireza Hosseini */

#include "mrl_ipcamera/mrl_ipcamera.hpp"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/video/video.hpp>

MrlIpCamera::MrlIpCamera(ros::NodeHandle *nodeHandle) : nh_(nodeHandle), image_transport_(*nodeHandle)
{
  camera_pub_ = image_transport_.advertiseCamera("/camera/image", 10);

  nh_->param<std::string>("video_url", video_url_, "rtsp://admin:A123456789@192.168.1.64/live.sdp?:network-cache=300");
  nh_->param<std::string>("frame_id", frame_id_, "cam_link");

  refresh_service_server_ = nh_->advertiseService("refresh", &MrlIpCamera::refreshSrvCallback, this);

  ROS_INFO_STREAM("Trying to connect to  " << video_url_);
  cap_.open(video_url_);
}

bool MrlIpCamera::publish()
{
  cv::Mat frame;
  ros::Rate loop(33);
  while (ros::ok())
  {
    if (cap_.isOpened())
    {
      ROS_INFO_ONCE("connection established");
      if (camera_pub_.getNumSubscribers() > 0)
      {
        cap_ >> frame;
        if (frame.empty())
          continue;
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
        camera_pub_.publish(rosimg, camera_info, ros::Time::now());
      }
    }
    else
    {
      cap_.release();
      ROS_WARN_STREAM_DELAYED_THROTTLE(10, "Video stream is not available, retrying...");
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
  if (!cap_.open(video_url_))
  {
    ROS_ERROR_STREAM("Connecting to " << video_url_ << " failed.");
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "MrlIpCamera");
  ros::NodeHandle nh("~");
  MrlIpCamera ipCamera(&nh);
  ipCamera.publish();
  return 0;
}
