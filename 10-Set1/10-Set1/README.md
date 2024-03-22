#Preparation：
(1) workspace: 
    #You don't neeed to follow the below steps, just to know that the workspace is "cam2image_ws, and navigate to workspace"
    PS C:\Users\zhuto> scp -r C:\Users\zhuto\Desktop\ASDfR\20240228.rar ubuntu@172.30.112.220:/home/ubuntu/test
    20240228.rar
    unrar x 20240228.rar
    cd test/20240228/ASDFR/cam2image_ws
(2) colcon all packages
    colcon build --symlink-install   #By using --symlink-install you dont need to rebuild after changing launch or config files
    source install/setup.bash

#Assignment:
#Note:Take 80.0 as an example below and change it to the value you want.
1.1
1.1.1
    ros2 run image_tools_sdfr cam2image --ros-args --params-file cam2image.yaml -p depth:=1 -p history:=keep_last
    ros2 run image_tools showimage
1.1.2
    ros2 run brightness brightness_node
    ros2 run brightness brightness_node --ros-args -p brightness_threshold:=80.0
    ros2 param set /brightness_node brightness_threshold 80.0             #using this to change the parameter during the code running
    ros2 topic echo /brightness_topic                                     #using this to check the topic

1.1.3
    ros2 run lpindicator lp_indicator_node
    ros2 run image_tools showimage --ros-args -r image:=/processed_image_topic
    ros2 topic echo /light_position_topic
    ros2 param set /lp_indicator_node brightness_threshold 80.0           #using this to find the appropriate threshold  

1.2
1.2.1
(1) Open cam2image_ws/src/relbot_controller/launch/relbot_controller_launch.py, and comment out the below codes:
        #Node(
        #    package='lpindicator',
        #    executable='lp_indicator_node',
        #    name='lp_indicator_node',
        #),
(2) ros2 launch relbot_controller relbot_controller_launch.py
    rqt

1.2.2
(1) Open relbot_controller_launch, and uncomment out the below codes:
        Node(
            package='lpindicator',
            executable='lp_indicator_node',
            name='lp_indicator_node',
        ),
(2) ros2 launch relbot_controller relbot_controller_launch.py
    ros2 topic echo /light_position_data_topic
    ros2 topic echo /output/camera_position
