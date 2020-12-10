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

#include "ipcamera_driver/ipcamera_driver.hpp"
#include <sensor_msgs/image_encodings.h>
#include <opencv2/video/video.hpp>

IpCameraDriver::IpCameraDriver() : pnh_("~"), image_transport_(pnh_), camera_info_manager_(pnh_)
{
  camera_pub_ = image_transport_.advertiseCamera("/camera/image", 10);

  pnh_.param<std::string>("video_url", video_url_, "rtsp://admin:A123456789@192.168.1.64/live.sdp?:network-cache=300");
  pnh_.getParam("camera_info_url", camera_info_url_);
  pnh_.param<std::string>("frame_id", frame_id_, "cam_link");

  refresh_service_server_ = pnh_.advertiseService("refresh", &IpCameraDriver::refreshSrvCallback, this);

  camera_info_manager_.setCameraName("camera");

  if (camera_info_manager_.validateURL(camera_info_url_))
  {
    if (camera_info_manager_.loadCameraInfo(camera_info_url_))
    {
      ROS_INFO_STREAM("Loaded camera calibration from " << camera_info_url_);
    }
    else
    {
      ROS_WARN_STREAM("Could not load camera info, using an uncalibrated config.");
    }
  }
  else
  {
    ROS_WARN_STREAM("Given camera info url: " << camera_info_url_ << " is not valid, using an uncalibrated config.");
  }

  ROS_INFO_STREAM("Trying to connect to  " << video_url_);
  cap_.open(video_url_);
}

bool IpCameraDriver::publish()
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
        if (frame.empty()) {
          ros::spinOnce();
          loop.sleep();
          continue;
        }
        cv_bridge::CvImage out_msg;
        out_msg.header.frame_id = frame_id_;
        out_msg.header.stamp = ros::Time::now();
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = frame;

        sensor_msgs::CameraInfo camera_info;
        camera_info = camera_info_manager_.getCameraInfo();
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

bool IpCameraDriver::refreshSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
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
  ros::init(argc, argv, "mrl_ip_camera");
  IpCameraDriver ipCamera;
  ipCamera.publish();
  return 0;
}
