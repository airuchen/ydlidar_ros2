# YDLidar ROS 2 Wrapper   
This is ROS2 version pkg based on original EAI-ydlidar ros wrapper repo.  
(original repo: https://github.com/EAIBOT/ydlidar)  

## Developers  
* Alan, Chen (alan.chen@adlinktech.com)  
* HaoChih, LIN (haochih.lin@adlinktech.com)  

## License  
Apache 2.0 (Copyright 2019 ADLINK Technology, Inc.)  
  
## Install udev  
* $ cd AMENT_WS/src/ydlidar_ros2  
* $ sudo chmod 777 ./*  
* $ sudo sh initenv.sh  

## Compile      
$ cd ~/ros2_ws  
$ ament build  
OR (build only)  
$ ament build --only-packages ydlidar  

For isolated build  
$ ament build --isolated --symlink-install --only ydlidar  

## Execute
$ ros2 run ydlidar ydlidar_node  
