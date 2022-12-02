#!/usr/bin/env python3

from nuscenes.nuscenes import NuScenes
import rospy
from pyquaternion import Quaternion
from nuscenes.utils.data_classes import Box
from nuscenes.utils.geometry_utils import box_in_image, BoxVisibility
import numpy as np


nusc = NuScenes(version='v1.0-mini', dataroot='/home/yash-motorai/Downloads/v1.0-mini', verbose=True)
classes = ['human.pedestrian.adult','human.pedestrian.police_officer','human.pedestrian.construction_worker','vehicle.car','vehicle.bicycle', 'vehicle.motorcycle', 'vehicle.emergency.ambulance', 'vehicle.emergency.police']

pedestrians = ['human.pedestrian.adult','human.pedestrian.police_officer','human.pedestrian.construction_worker'] 
def get_sample_data(nusc_object, sample_data_token, box_vis_level=BoxVisibility.ANY, selected_anntokens=None):
    """
    Returns the data path as well as all annotations related to that sample_data(single image).
    Note that the boxes are transformed into the current sensor's coordinate frame.
    :param sample_data_token: . Sample_data token(image token).
    :param box_vis_level: . If sample_data is an image, this sets required visibility for boxes.
    :param selected_anntokens: []. If provided only return the selected annotation.
    :return: (data_path , boxes [], camera_intrinsic )
    """

    sd_record = nusc_object.get('sample_data', sample_data_token)
    pose_record = nusc_object.get('ego_pose', sd_record['ego_pose_token'])
    cs_record = nusc_object.get('calibrated_sensor', sd_record['calibrated_sensor_token'])
    sample_record = nusc_object.get('sample',sd_record['sample_token'])


    boxes = nusc_object.get_boxes(sample_data_token)
    selected_anntokens = sample_record['anns']
    timestamp = sample_record['timestamp']
    orig_objects_detected =[]
    for box,ann in zip(boxes,selected_anntokens):

        # Move box to ego vehicle coord system
        box.translate(-np.array(pose_record['translation']))
        box.rotate(Quaternion(pose_record['rotation']).inverse)

        #  Move box to sensor coord system
        box.translate(-np.array(cs_record['translation']))
        box.rotate(Quaternion(cs_record['rotation']).inverse)
        
        if box.name in classes: #if the box.name is in the classes we want to detect
    
            if box.name in pedestrians: 
                orig_objects_detected.append("pedestrian")
            elif box.name == "vehicle.car":
                orig_objects_detected.append("car")
            else:
                orig_objects_detected.append("cyclist")
            #print(box)
            
            visibility = nusc.get('sample_annotation', '%s' %ann)['visibility_token'] #give annotation key
            visibility = int(visibility)

            if visibility > 1:
                center = box.center #get boxe's center