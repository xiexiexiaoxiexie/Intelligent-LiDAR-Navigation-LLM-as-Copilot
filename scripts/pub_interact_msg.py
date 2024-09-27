#!/home/xiefujing/anaconda3/envs/openai/bin/python3
import rospy
from std_msgs.msg import String
import json
import map_drawer
from map_drawer import parse_osm
import os
import time
import xml.etree.ElementTree as ET
import process_osm
import re
import utility
import os
import logging
logging.disable(logging.CRITICAL)
os.chdir('/home/xiefujing/catkin_ws')

with open('./src/osmAG_intelligent_navigation/scripts/config.json', 'r') as config_file:
    config = json.load(config_file)
# which case to run
with open('./src/osmAG_intelligent_navigation/scripts/cases/case2/case2.json', 'r') as config_file:
    case2_config = json.load(config_file)

osm_path_prefix=config['osm_path_prefix']
utm_file_name=config['utm_file_name']
pathlength_json_file='ShanghaiTech_merge_F2_corrected_id2name_bak_utm_path__.json'
navigation_api_received=False

# only used in testing, to interact in terminal
def talker():
    areas_tree = ET.parse(osm_path_prefix+utm_file_name+'.osm')
    areas=parse_osm(areas_tree)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg_choice = input("Enter your message: ")
        if msg_choice =='shortest_path':
            # start_room=input("Enter start room: ")
            # end_room=input("Enter end room: ")
            start_room='E1d-F1-06'
            end_room='E1b-F1-07'
            no_path=[]
            free_pass=[start_room,end_room]
            cropped_tree=process_osm.del_all_leaves(areas_tree,osm_path_prefix+utm_file_name+'_feedin.osm',free_pass,no_path)
            cropped_tree=process_osm.del_all_leaves(cropped_tree,osm_path_prefix+utm_file_name+'_feedin.osm',free_pass,no_path)
            cropped_tree=process_osm.del_all_leaves(cropped_tree,osm_path_prefix+utm_file_name+'_feedin.osm',free_pass,no_path)
            cropped_areas = map_drawer.parse_osm(cropped_tree)
            with open(osm_path_prefix+pathlength_json_file, 'r') as f:
                areas_paths = json.load(f)
            shortest_area_path,shortest_path_normal=map_drawer.load_plot_json_paths(areas_tree,osm_path_prefix+'ShanghaiTech_merge_F2_corrected_id2name_bak_utm_path__.json',areas,cropped_areas,free_pass,[],[])
            map_drawer.generate_additional_text_4_figure(osm_path_prefix+utm_file_name+'_feedin.osm',osm_path_prefix+utm_file_name+'_text.osm',cropped_areas)

            external_msg="shortest_path:"+str(shortest_area_path)
            pub_incoming_msg.publish(external_msg)
            print(f'publishing: {external_msg}')
        else:
            external_msg=utility.get_text_from_md(msg_choice)
            pub_incoming_msg.publish(external_msg)
            print(f'publishing: {external_msg}')
        rate.sleep()

# used to publish events in a case one by one to NavigationEventMonitor for it to track down events
def send_external_msgs(case2_config,pub_incoming_msg,pub_history_ready):
    global navigation_api_received
    events=case2_config['case2']['events']
    for event in events:
        if event=='navigation_task_start':
            pub_history_ready.publish('history info ready')
            print(f"{utility.RED}[pub-interact_msg]:history info finished shutting down{utility.RESET}")
        else:
            pub_incoming_msg.publish(utility.get_text_from_md(event))
            navigation_api_received=False
            print(f'{utility.YELLOW}publishing: {utility.get_text_from_md(event)}{utility.RESET}')
            while not navigation_api_received and rospy.is_shutdown() is False:
                print(f"navigation_api_received={navigation_api_received}")
                time.sleep(1) #wait for api to get result 

# used to publish events one by one to NavigationEventMonitor for it to track down events
def send_all_external_msgs(pub_incoming_msg,pub_history_ready):
    global navigation_api_received
    # testing NavigationEventMonitor's ability to tell if this msg is relevant to robot navigation
    with open('./src/osmAG_intelligent_navigation/external/email_inrelevant.md', 'r') as file:
        content = file.read()
    blocks = re.findall(r'###.*?(?=\n#|\Z)', content, re.DOTALL)
    blocks_list = [block.split('\n', 1)[1].strip() for block in blocks if '\n' in block]
    for block in blocks_list[0:]:
        pub_incoming_msg.publish(block)
        navigation_api_received=False
        print(f'{utility.YELLOW}publishing: {block}{utility.RESET}')
        while not navigation_api_received and rospy.is_shutdown() is False:
            print(f"navigation_api_received={navigation_api_received}")
            time.sleep(1)

def NavigationEventMonitor_callback(data):
    global navigation_api_received
    navigation_api_received=True
    print(f"NavigationEventMonitor_callback: {navigation_api_received}")

if __name__ == '__main__':
    try:
        rospy.init_node('pub_interact_msg' , anonymous=True)
        print(f"{utility.RED}pub_interact_msg Node initialized successfully{utility.RESET}")
    except Exception as e:
        print("error")
    sub_response=rospy.Subscriber("NavigationEventMonitor_response_json_str", String, NavigationEventMonitor_callback)
    pub_incoming_msg = rospy.Publisher('external_msg', String, queue_size=10)
    pub_history_ready = rospy.Publisher('history_ready', String, queue_size=10)

    time.sleep(2) #wait for NavigationEventMonitor node to start
    send_external_msgs(case2_config,pub_incoming_msg,pub_history_ready)
    # send_all_external_msgs(pub_incoming_msg,pub_history_ready)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        # talker()
        rospy.spin()
        rate.sleep()

        