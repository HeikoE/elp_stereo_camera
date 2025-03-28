# elp_stereo_camera

ROS2 camera driver for the ELP stereo camera, tested with ROS Jazzy and the ELP-USB3D1080P120-H120 model.

The supported formats of the camera can be found in `/config/supported_formats.txt`.  
To create disparity images and/or point clouds, you need to calibrate the camera and adjust the camera parameter files in `/cam_params`.

---

## USAGE

### Start the Camera
```bash
ros2 launch elp_stereo_camera stereo_camera_launch.py
```


### Camera Calibration (example)
```bash
ros2 run camera_calibration cameracalibrator --approximate 0.1 --size 10x7 --square 0.025 right:=/stereo/right/image_raw left:=/stereo/left/image_raw right_camera:=/stereo/right left_camera:=/stereo/left
```

### Stereo Image Processing (rectification, disparity image and point cloud):
```bash
ros2 launch stereo_image_proc stereo_image_proc.launch.py namespace:=stereo
```