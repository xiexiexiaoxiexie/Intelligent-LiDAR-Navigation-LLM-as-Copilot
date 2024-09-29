#!/home/xiefujing/anaconda3/envs/openai/bin/python3
import sys
import os
os.chdir('/home/xiefujing/catkin_ws')
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
parent_parent_dir = os.path.dirname(parent_dir)
sys.path.append(parent_parent_dir)
import json
import utility 
import rospy
from std_msgs.msg import String
from osmAG_intelligent_navigation.msg import StringList
import subprocess
from functools import partial
from jsk_rviz_plugins.msg import OverlayText
from std_srvs.srv import Empty
with open('./src/osmAG_intelligent_navigation/scripts/cases/case4/case4.json', 'r') as config_file:
    case4_config = json.load(config_file)
with open('./src/osmAG_intelligent_navigation/scripts/config.json', 'r') as config_file:
    config = json.load(config_file)

osm_path_prefix=config['osm_path_prefix']
utm_file_name=config['utm_file_name']
free_pass_index=5

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
    free_pass=case4_config[case_num]['free_passes']['free_pass1']
    args=prepare_arg(areas,free_pass[0],case_num)
    args['free_pass_start']=free_pass[0]
    args['free_pass_end']=free_pass[1]
    args['human_natural_language_command']=case4_config['case4']['human_natural_language_command']
    args['world_file']=case4_config['case4']['world_file']
    return args

def start_launch(launch_args,launch_file):
    arg_str = " ".join([f'{arg}:="{value}"' if isinstance(value, str) else f'{arg}:={value}' for arg, value in launch_args.items()])
    print(f"arg_str={arg_str}")
    cmd = f"gnome-terminal --working-directory=/home/xiefujing/catkin_ws \
    -- zsh -c 'source /opt/ros/noetic/setup.zsh;\
        source ./devel_isolated/setup.zsh; \
    roslaunch {launch_file} {arg_str}; \
    exec bash'"
    subprocess.Popen(cmd, shell=True)

def process_case(case_num,case_config):
    human_natural_language_command=case_config['human_natural_language_command']
    free_passes=case_config['free_passes']
    for i,free_pass in free_passes.items():
        print(f"[case.py]: case_num: {case_num},free_pass_num={i},free_pass={free_pass}")

def navigation_result_callback(data,case_config,pub):
    global free_pass_index
    list_free_passes=list(case_config['free_passes'].values())
    print(f"{utility.PURPLE}[case4.py]navigation_result_callback: {data.data}, only send next task when result is true.{utility.RESET}")
    if data.data=='True' or data.data=='Parameter_ready':
        if data.data=='True':
            clear_costmaps()
        pub.publish(StringList(data=list_free_passes[free_pass_index]))
        print(f"{utility.PURPLE}[case4.py]:publishing the {free_pass_index} trial: {StringList(data=list_free_passes[free_pass_index])}{utility.RESET}")
        free_pass_index+=1

def set_ros_parameters(case_config):
    # rospy.init_node('param_setter')
    rospy.set_param('/human_natural_language_command', case_config['human_natural_language_command'])
    rospy.set_param('/world_file', case_config['world_file'])
    rospy.set_param('/case_num', case_config['case_num'])
    rospy.set_param('/Bcommand_move_base', case_config['Bcommand_move_base'])
    rospy.set_param('/closed_passage', case_config['closed_passage'])
    rospy.set_param('/closed_area', case_config['closed_area'])
    case_text=utility.get_text_from_md(case_config['story'])
    rospy.set_param('/case_text', case_text)
    print(f"[case_node]: All parameters are set.")

def publish_text1(case_config):
    events=case_config['events']
    text=String
    for event in events:
        if event=='navigation_task_start':
            pass
        else:
            text+=utility.get_text_from_md(event)
    text_pub1 = rospy.Publisher('/case_node', OverlayText, queue_size=10)
    text_msg = OverlayText()
    text_msg.width = 200
    text_msg.height = 50
    text_msg.left = 10
    text_msg.top = 10
    text_msg.text_size = 14
    text_msg.line_width = 2
    text_msg.font = "DejaVu Sans Mono"
    text_msg.text = text
    text_msg.fg_color.r = 1.0
    text_msg.fg_color.g = 1.0
    text_msg.fg_color.b = 1.0
    text_msg.fg_color.a = 1.0
    text_msg.bg_color.r = 0.0
    text_msg.bg_color.g = 0.0
    text_msg.bg_color.b = 0.0
    text_msg.bg_color.a = 0.8
    text_pub1.publish(text_msg)

def clear_costmaps():
        rospy.wait_for_service('/robot2/move_base/clear_costmaps')
        try:
            clear_obstacle_service = rospy.ServiceProxy('/robot2/move_base/clear_costmaps', Empty)
            clear_obstacle_service()
            print(f"{utility.ORANGE}Costmaps cleared{utility.RESET}")
        except rospy.ServiceException as e:
            print(f"{utility.ORANGE}Costmaps cleared FAILED{utility.RESET}")
            rospy.wait_for_service('/robot2/move_base/clear_costmaps')

if __name__ == "__main__":
    # pipeline: start send msg to history, let it fill in the history. when event is 'navigation', it means all msgs are sent, ready for access cost to start the navigations.
    case_config=case4_config['case4']
    rospy.init_node('case_node', anonymous=True)
    set_ros_parameters(case_config)
    free_pass_pub = rospy.Publisher('trial_free_pass', StringList, queue_size=10)
    rospy.Subscriber("navigation_result", String, partial(navigation_result_callback,case_config=case_config,pub=free_pass_pub))
    rospy.spin()
    
