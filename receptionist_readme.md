# 标点
确保`constants.json`里的内容都正确且是当前地图的。

**如果不在新主控上，将`receptionist.py`第32行`file = open("...", "r")`的地址改成`constants.json`的绝对路径！**

# 启动Receptionist
**梯子选择带`GPT`后缀的节点，不要用香港的！连不上GPT！（如果已经在巴西了请忽略）**

如果还没有setup，使用sudo权限运行`setup.sh`

使用`tk23_vision`下`scripts`文件夹中的shell文件启动所需节点。
以下是所需的脚本和脚本应该有的内容：
脚本中被注释的内容可有可无，如果当前的脚本中没有被注释也可以（虽然可能会慢一点点）

`audio.sh`
```bash
gnome-terminal --tab -- zsh -c "export ALL_PROXY='' && export all_proxy='' && source install/setup.zsh && ros2 run audio_pakage announce; exec zsh"
gnome-terminal --tab -- zsh -c "export ALL_PROXY='' && export all_proxy='' && source install/setup.zsh && ros2 run audio_pakage phrase_extraction; exec zsh"
gnome-terminal --tab -- zsh -c "export ALL_PROXY='' && export all_proxy='' && source install/setup.zsh && ros2 run audio_pakage get_confirmation; exec zsh"    
gnome-terminal --tab -- zsh -c "export ALL_PROXY='' && export all_proxy='' && source install/setup.zsh && ros2 run audio_pakage listen; exec zsh"    
gnome-terminal --tab -- zsh -c "export ALL_PROXY='' && export all_proxy='' && source install/setup.zsh && ros2 run audio_pakage compare_interest; exec zsh"    
```

`grasp.sh`
```bash
gnome-terminal --tab -- zsh -c "source install/setup.zsh && ros2 launch mobile_bringup grasp_bringup.launch.py; exec zsh"
gnome-terminal --tab -- zsh -c "source install/setup.zsh && ros2 run pick_and_place pick_and_place; exec zsh"
```

`navigation.sh`
```bash
gnome-terminal --tab -- zsh -c "source install/setup.zsh && ros2 launch navigation_bringup driver.launch.py; exec zsh"  
gnome-terminal --tab -- zsh -c "source install/setup.zsh && ros2 launch navigation_bringup bringup_launch.py; exec zsh"     
```

`vision.sh`
```bash
gnome-terminal --tab -- zsh -c "source install/setup.zsh && ros2 launch orbbec_camera femto_bolt.launch.py enable_colored_point_cloud:=true depth_registration:=true; exec zsh"
# gnome-terminal --tab -- zsh -c "source install/setup.zsh && ros2 run object_detection service_sam --ros-args -p visualization_en:=false -p visualization_log_en:=true -p detection_topic_en:=false; exec zsh"
# gnome-terminal --tab -- zsh -c "source ~/.zshrc && cd ~/tk25_ws/src/tk23_vision/src/lang-segment-anything && conda activate anygrasp && python tcp_server.py; exec zsh"
gnome-terminal --tab -- zsh -c "source install/setup.zsh && ros2 run util door_detection; exec zsh"     
gnome-terminal --tab -- zsh -c "source install/setup.zsh && ros2 run pan_tilt ctrl --ros-args -p device:=\"/dev/ttyUSB1\"; exec zsh"
gnome-terminal --tab -- zsh -c "source install/setup.zsh && ros2 run pan_tilt ctrl --ros-args -p device:=\"/dev/ttyUSB0\"; exec zsh"
gnome-terminal --tab -- zsh -c "source install/setup.zsh && ros2 run pan_tilt follow_head; exec zsh"
```

`vision_kimiapi.sh`
```bash
gnome-terminal --tab -- zsh -c "export ALL_PROXY='' && export all_proxy='' && source install/setup.zsh && ros2 run kimi_api feature_recognition; exec zsh"
gnome-terminal --tab -- zsh -c "export ALL_PROXY='' && export all_proxy='' && source install/setup.zsh && ros2 run kimi_api feature_matching; exec zsh"
# gnome-terminal --tab -- zsh -c "export ALL_PROXY='' && export all_proxy='' && source install/setup.zsh && ros2 run kimi_api grocery_categorize; exec zsh"
```

最后，运行
`ros2 run behavior_tree receptionist`
