#!/bin/bash

# first activate openpc_det 
echo "make sure to activate openpc_det"
source /home/ps/hxw_projects/Hongfeng_LaneDet/ros_cv_bridge_helper/catkin_workspace/devel/setup.bash --extend 
source devel/setup.bash 

#############################################################################################
# class names as English 
#############################################################################################
# original indoor bag  
if [ 1 -eq 0 ]; then 
    roslaunch yolov3_pytorch_ros detector.launch ros_bag_file:="/home/ps/hxw_projects/OpenPCDet_working/catkin_ws/src/hf3d_ros/bag/indoor1.bag" classes_name:="coco.names"  
fi 
# data collected with 32 beams and cameras 
# just test cameras  
if [ 1 -eq 0 ]; then 
    roslaunch yolov3_pytorch_ros detector.launch ros_bag_file:="/media/ps/data/HF3D_Obj_Det/raw_data/2021-01-06-08-55-27.bag" classes_name:="coco.names"  
fi


#############################################################################################
# class names as Manderine  
#############################################################################################
# original indoor bag  
if [ 1 -eq 0 ]; then 
    roslaunch yolov3_pytorch_ros detector.launch ros_bag_file:="/home/ps/hxw_projects/OpenPCDet_working/catkin_ws/src/hf3d_ros/bag/indoor1.bag" classes_name:="coco_cn.names" class_names_cn:=True 
fi 
# data collected with 32 beams and cameras 
# just test cameras  
if [ 1 -eq 0 ]; then 
    roslaunch yolov3_pytorch_ros detector_multi_cameras.launch ros_bag_file:="/media/ps/data/HF3D_Obj_Det/raw_data/2021-01-08-14-28-07.bag" classes_name:="coco_cn.names" class_names_cn:=True  
fi

if [ 1 -eq 1 ]; then 
    roslaunch yolov3_pytorch_ros detector_multi_cameras.launch ros_bag_file:="/media/ps/data/HF3D_Obj_Det/raw_data/hf_indoors_cameras3_lidars2_frame_id_rs16_rs32.bag" classes_name:="coco_cn.names" class_names_cn:=True  
fi



if [ 1 -eq 0 ]; then 
    # launch usb camera dirver in ROS
    # roslaunch usb_cam usb_cam-test.launch
    # detect it 
    CUDA_VISIBLE_DEVICES="1" roslaunch yolov3_pytorch_ros detector_from_camera.launch classes_name:="coco_cn.names" class_names_cn:=True  
fi 
