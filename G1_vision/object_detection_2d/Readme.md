
## intrinsic calibration

### pointgrey camera
`rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.027 
camera:=/pointgrey/camera  image:=/pointgrey/camera/image_color`

### launch the camera with namespace
`ROS_NAMESPACE=pointgrey  roslaunch pointgrey_camera_driver camera.launch`


## extrinsic calibration
### pointgrey camera (default)
`rosrun object_detection_2d top_down_calib`
`rosrun object_detection_2d top_down_calib _camera:=/camera/camera_info  _image:=/camera/image_color`

### kinect camera
`rosrun object_detection_2d top_down_calib _camera:=/camera/rgb/camera_info  _image:=/camera/rgb/image_rect_color`

### run extrinsic calibration with know center location
`rosrun object_detection_2d top_down_calib _center_x:=0.0 _center_y:=0.0`
`rosrun object_detection_2d top_down_calib _center_x:=0.72 _center_y:=0.0 _center_z:=-0.19`

`rosrun object_detection_2d top_down_calib _camera:=/camera/camera_info  _image:=/camera/image_color  _center_x:=0.747 _center_y:=0.003 _center_z:=-0.190`

### run static tf publisher
`rosrun tf static_transform_publisher  0.13148212 0.0023827571 1.9917148  -0.682058 0.689909 -0.175989 0.166885  /base  /top_down_camera_link 30`



