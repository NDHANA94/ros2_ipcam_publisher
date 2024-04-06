# ros2_ipcam_publisher
This pkg is developed to publish IP camera video stream over ROS-2 topic

### Install & Build
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/NDHANA94/ros2_ipcam_publisher.git

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.sh
```

### Run
```
ros2 run ros2_ipcam_publisher ipcam_publisher --ros-args -p camera_ip:="<CAMERA IP ADDRESS>" 
```

### Launch
- Use `config/params.yaml` file to configure camera IP address and other parameters.
- Do `colocn build` again
- Then run launch file
```
ros2 launch ros2_ipcamera_publisher ipcam.launch.py
```

### Parameters:

| Parameter | Data Type | Default Value |
| --- | --- |---|
| camera_ip | string | "" |
| image_size | vector<int64_t>> | [1280, 720] |
| frame_rate | int64_t | 100 | 


### App for Smart Phones to stream video:
- Android: IP Webcam

