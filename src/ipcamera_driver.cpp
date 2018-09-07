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
#include <opencv2/video/video.hpp>

using namespace std::chrono_literals;

IpCameraDriver::IpCameraDriver() : pnh_("~"), image_transport_(pnh_), camera_info_manager_(pnh_)
{
  camera_pub_ = image_transport_.advertiseCamera("/camera/image", 10);

  pnh_.param<std::string>("video_url", video_url_, "rtsp://admin:A123456789@192.168.1.64/live.sdp?:network-cache=300");
  pnh_.getParam("camera_info_url", camera_info_url_);
  pnh_.param<std::string>("frame_id", frame_id_, "cam_link");
  pnh_.param<int>("frame_rate", frame_rate_, 30);

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

void IpCameraDriver::capture()
{
  while (ros::ok())
  {
    ROS_INFO_ONCE("Starting capture thread...");
    if (keep_running)
    {
      mutex_.lock();
      if (cap_.isOpened())
      {
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty())
        {
          mutex_.unlock();
          continue;
        }
        frames_buffer_.push_back(frame);
        mutex_.unlock();
      }
    }
  }
}

bool IpCameraDriver::publish()
{
  cv::Mat frame;
  ros::Rate loop(frame_rate_);
  last_time_frame_received_ = ros::Time::now();
  std::thread capture_thread = std::thread(&IpCameraDriver::capture, this);
  std::this_thread::sleep_for(2s);  // Give sometime to thread
  while (ros::ok())
  {
    if (cap_.isOpened() && (ros::Time::now() - last_time_frame_received_) <= NO_FRAME_TIME_TOLERANCE)
    {
      ROS_INFO_ONCE("connection established");
      if (frames_buffer_.empty())
      {
        ROS_WARN_STREAM("There is no new frame!");
        ros::spinOnce();
        loop.sleep();
        continue;
      }
      last_time_frame_received_ = ros::Time::now();
      cv_bridge::CvImage out_msg;
      out_msg.header.frame_id = frame_id_;
      out_msg.header.stamp = ros::Time::now();
      out_msg.encoding = sensor_msgs::image_encodings::BGR8;
      out_msg.image = frames_buffer_.front();

      if (camera_pub_.getNumSubscribers() > 0)
      {
        sensor_msgs::CameraInfo camera_info;
        camera_info = camera_info_manager_.getCameraInfo();
        camera_info.header.frame_id = frame_id_;
        camera_info.header.stamp = ros::Time::now();

        sensor_msgs::Image rosimg;
        out_msg.toImageMsg(rosimg);
        camera_pub_.publish(rosimg, camera_info, ros::Time::now());
      }
      frames_buffer_.pop_front();
    }
    else
    {
      ROS_ERROR("Camera connection is corrupted, reinitializing the connection...");
      if (refresh())
      {
        last_time_frame_received_ = ros::Time::now();
      }
    }
    ros::spinOnce();
    loop.sleep();
  }

  capture_thread.join();
  return true;
}

bool IpCameraDriver::refresh()
{
  keep_running = false;
  mutex_.lock();
  cap_.release();
  if (!cap_.open(video_url_))
  {
    ROS_ERROR_STREAM("Connecting to " << video_url_ << " failed.");
    mutex_.unlock();
    return false;
  }
  mutex_.unlock();
  keep_running = true;
  return true;
}

bool IpCameraDriver::refreshSrvCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  ROS_INFO("Received a request to refresh the video stream...");
  refresh();
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mrl_ip_camera");
  IpCameraDriver ipCamera;
  ipCamera.publish();
  return 0;
}
