##  vision
第一次启动：
```bash
sudo chmod 777 /dev/ttyUSB0 # 或者1
```

先source,然后

```bash
ros2 launch orbbec_camera femto_bolt.launch.py enable_colored_point_cloud:=true depth_registration:=true
ros2 run pan_tilt ctrl --ros-args -p device:="/dev/ttyUSB0" # 或者1
ros2 run vision_track person_track_server
```

## manipulation (for URDF)
先source,然后
```bash
ros2 launch mobile_bringup arm_bringup_fake.launch.py
```


## navigation
先source,然后
```bash
ros2 launch navigation_bringup driver.launch.py
ros2 launch navigation_bringup bringup_dwb_launch.py
ros2 run following tracker_action_server
```

## audio
```bash
source ~/anaconda3/etc/profile.d/conda.sh && conda activate audiotts_new && export PYTHONNOUSERSITE=1 && source ~/tk25_ws/install/setup.zsh && python ~/tk25_ws/src/tk_24_audio/src/audio_pakage/audio_pakage/tts.py
```

## behavior tree
先source,然后
```bash
ros2 run behavior_tree hri-follow
```