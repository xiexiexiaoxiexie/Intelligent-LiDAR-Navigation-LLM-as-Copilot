#!/home/xiefujing/anaconda3/envs/openai/bin/python3
import sys
import os

os.chdir('/home/xiefujing/catkin_ws')
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
parent_parent_dir = os.path.dirname(parent_dir)

sys.path.append(parent_parent_dir)
import openai
import json
import markdown
import map_handler
import map_drawer
import pub_interact_msg
import csv
import pandas as pd
import time
import xml.etree.ElementTree as ET
import process_osm
import re
import utility 
import rospy
from std_msgs.msg import String
import roslaunch
import logging
import save_experience
import simple_command_move_base
from osmAG_intelligent_navigation.msg import StringList
import subprocess
from functools import partial
# launch-prefix="xterm -e gdb -ex run --args"
'''
E1c-F1-COR-02_to_E1c-F1-COR-01 is closed as the access cost should catch that, and should avoid lobby'''
os.chdir('/home/xiefujing/catkin_ws')
with open('./src/osmAG_intelligent_navigation/scripts/cases/case_template/case_template.json', 'r') as config_file:
    case1_config = json.load(config_file)

with open('./src/osmAG_intelligent_navigation/scripts/config.json', 'r') as config_file:
    config = json.load(config_file)
osm_path_prefix=config['osm_path_prefix']
utm_file_name=config['utm_file_name']
free_pass_index=0

def get_start_position(areas, start_area_name):
    for area_name, area_data in areas.items():
        if area_name == start_area_name:
            start_position = list(area_data['centroid'])
            start_position.append(area_data['level_height'])
            args = {
            "initial_position_x": area_data['centroid'][0]-config['osm_utm_transform']['x'],
            "initial_position_y": area_data['centroid'][1]-config['osm_utm_transform']['y'],
            "level_height": 0.0,
            }
            return args
        
def prepare_arg(areas, start_area_name,case_num):
    args=get_start_position(areas, start_area_name)   
    args['case_num']=case_num  
    return args   

def prepare(areas,case_num):
    free_pass=case1_config[case_num]['free_passes']['free_pass1']

    args=prepare_arg(areas,free_pass[0],case_num)
    args['free_pass_start']=free_pass[0]
    args['free_pass_end']=free_pass[1]
    args['human_natural_language_command']=case1_config['case1']['human_natural_language_command']
    args['world_file']=case1_config['case_template']['world_file']

    # start_area=utility.locate_robot_area(free_pass['start'],areas)

    return args

def start_launch(launch_args,launch_file):
    # launch_file = "osmAG_intelligent_navigation robot2.launch"
    # arg_str = " ".join([f"{arg}:={value}" for arg, value in launch_args.items()])
    arg_str = " ".join([f'{arg}:="{value}"' if isinstance(value, str) else f'{arg}:={value}' for arg, value in launch_args.items()])
    print(f"arg_str={arg_str}")
    # for whatever reason, rviz and gazebo has to be manually started
    cmd = f"gnome-terminal --working-directory=/home/xiefujing/catkin_ws \
    -- zsh -c 'source /opt/ros/noetic/setup.zsh;\
        source ./devel_isolated/setup.zsh; \
    roslaunch {launch_file} {arg_str}; \
    exec bash'"
    subprocess.Popen(cmd, shell=True)
# process one case including several start areas but with single goal since human instruction is the same
# case_num:case1, case_config=value in json
def process_case(case_num,case_config):
    human_natural_language_command=case_config['human_natural_language_command']
    free_passes=case_config['free_passes']
    for i,free_pass in free_passes.items():
        print(f"[case.py]: case_num: {case_num},free_pass_num={i},free_pass={free_pass}")
def navigation_result_callback(data,case_config,pub):
    global free_pass_index
    list_free_passes=list(case_config['free_passes'].values())
    print(f"{utility.PURPLE}[case_template.py]navigation_result_callback: {data.data}, ready for next test navigation task.{utility.RESET}")
    pub.publish(StringList(data=list_free_passes[free_pass_index]))
    print(f"{utility.PURPLE}[case_template.py]:publishing the {free_pass_index} trial: {StringList(data=list_free_passes[free_pass_index])}{utility.RESET}")

    free_pass_index+=1

def set_ros_parameters(case_config):
    # rospy.init_node('param_setter')
    rospy.set_param('/human_natural_language_command', case_config['human_natural_language_command'])
    rospy.set_param('/world_file', case_config['world_file'])
    rospy.set_param('/case_num', case_config['case_num'])

    print(f"[case_node]: All parameters are set.")

if __name__ == "__main__":
    # pipeline: start send msg to history, let it fill in the history. when event is 'navigation', it means all msgs are sent, ready for access cost to start the navigations.
    case_config=case1_config['case_template']
    rospy.init_node('case_node', anonymous=True)
    set_ros_parameters(case_config)

    free_pass_pub = rospy.Publisher('trial_free_pass', StringList, queue_size=10)
    rospy.Subscriber("navigation_result", String, partial(navigation_result_callback,case_config=case_config,pub=free_pass_pub))
    rospy.spin()
    
# rostopic pub /navigation_result std_msgs/String "finished" -1
    

    # publish next trial when the navigation result is received
    # areas_tree = ET.parse(osm_path_prefix+utm_file_name+'.osm')
    # areas= map_drawer.parse_osm(areas_tree)
    # start launch and process the first navigation task, following tasks will be handle by the call_back function.
    # launch_args=prepare(areas,'case1')
    # start_launch(launch_args,'osmAG_intelligent_navigation robot2.launch')
    # free_pass_index=1

    # time.sleep(10)
    # publish the first trial
    # list_free_passes=list(case1_config['case1']['free_passes'].values())
    # free_pass_pub.publish(StringList(data=list_free_passes[0]))
    # print(f"{utility.PURPLE}[case1.py]:publishing the first trial: {StringList(data=list_free_passes[0])}{utility.RESET}")


    # process_case('case1',case1_config['case1'])

    
    # start_launch(launch_args,'osmAG_intelligent_navigation pub_event.launch')


    # pub_interact_msg.send_external_msgs(case1_config)
