# ipcamera_driver
Simple node to publish regular IP camera video streams to a ros topic.

## Nodes

### ipcamera_driver_node

#### Publishers

- /camera/image ([sensor_msgs/Image](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/Image.html)):
  Base topic, used for "raw" transport.
- /camera/image/compressed (transport-specific type):
  Topic for "compressed" transport.
- /camera/image/theora (transport-specific type):
  Topic for "theora" transport.
- /camera/camera_info([sensor_msgs/CameraInfo](http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html)]):
  Camera info topic.
   
#### Parameters

- video_url (string)
- camera_info_url (string)
- frame_id (string, "cam_link")
