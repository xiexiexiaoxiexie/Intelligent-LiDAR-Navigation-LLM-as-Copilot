#!/usr/bin/env python
import openai
import json
import os
import utility
import rospy
import time
from std_msgs.msg import String
import logging
from osmAG_intelligent_navigation.msg import StringList
from overlay_text import OverlayTextManager
from std_msgs.msg import ColorRGBA

logging.disable(logging.CRITICAL)
os.chdir('/home/xiefujing/catkin_ws')
with open('./src/osmAG_intelligent_navigation/scripts/config.json', 'r') as config_file:
    config = json.load(config_file)
os.environ["http_proxy"] = config['proxy']['http']
os.environ["https_proxy"] = config['proxy']['https']
os.environ['OPENAI_API_KEY'] = config['openai_api_key']
openai.proxy = config['proxy']

over_lay_text_manager=OverlayTextManager()
over_lay_text_manager.add_text_box('/event_status', '', 600, 100, 10, 460, 12, 2, ColorRGBA(0.9, 0.9, 0.1, 1.0), ColorRGBA(0.0, 0.0, 0.0, 0.7))
pub_response = rospy.Publisher('NavigationEventMonitor_response_json_str', String, queue_size=10)
pub_response_all = rospy.Publisher('NavigationEventMonitor_response_all', String, queue_size=10)
pub_get_next_event = rospy.Publisher('get_next_event', String, queue_size=10)
pub_prompt=rospy.Publisher('NavigationEventMonitor_prompt',String, queue_size=10)
pub_current_history_status=rospy.Publisher('NavigationEventMonitor_history',String, queue_size=10)
# wait for parameter server
while not rospy.has_param('/case_num') and not rospy.is_shutdown():
    print(f"{utility.GREEN}[NavigationEventMonitor]: waiting for parameter to be set")
    time.sleep(1)
case_num_arg = rospy.get_param('/case_num', 'case2')
event_history_json_file=config['cases_folder']+'/history_test_irrelevant.json'
utm_file_name=config['utm_file_name']
last_json_response=''
last_event_prompt=''

# this is the prompt for tracking events
msg_prompt='''
Task : Path Planning Relevant Event Monitoring and Information Update
You are a assist that keep monitoring external messages that may affect a robot path planning task inside a building. This task focuses on continuously monitoring and updating the status based on incoming information that may affect path planning.
The robot could navigate indoor and outdoor, has the ability to take elevator, it will plan a path using tradtional path planning algorithm, its your job to keep track external events that maybe affect the path planning, accroding to the map, you need to decide which area/room will be affected by the event, and update the robot's current status, so the robot could avoid going to the unwanted room. (Not all the external message will affect the path planning, you need to if it is relevant to path planning tasks)
Responsibilities:

1. Monitor Incoming Messages: You will receive notifications such as weather updates, campus news, or specific alerts from emails like elevator maintenance that may affect path planning.
2. Update Status: Use the incoming messages and the provided map to determinate if it is relevant to current or furture path planning tasks and update the robot's current status with the new data.
3. Manage Events: Remove any events from 'current_status' that are no longer relevant to path planning, ensuring that only current threats or considerations are tracked. And please be carefully with room_name according to the map, only include the leaf area rooms (could be room or corridor or stairs or elevators) that absoluate necessary and exist in the map. Please note that there may be several leaf areas that could be affected by one event, you need to include all of them in 'room_name' in 'current_events'.

Inputs: 

1.'external_msg': Time, weather, and other campus news that may or may not affect path planning tasks.
2.'map': Provided in OSM (XML) format detailing the building information.
3.'history_status': A record of past statuses and events stored in the variable 'history_status'. This history is used to update the robot's current status and will itself be updated to become the historical record for the next cycle.

Outputs:

A JSON object that represents the updated status of the robot, which may include newly relevant events or the removal of outdated ones. Fields should be filled out based on the latest data, and left blank if no updates are available. And please do not output any other json format string if it is not your final output.


{{
    "current_status": {{
        "current_date": "", 
        "current_time": "",
        "today_weather": "",
        "current_events": [
            {{
                "event_name": "",
                "event_data": "",
                "event_duration": "",
                "room_name": "",
                "msg_text": ""
            }},
            // Add more events as needed
        ]
    }}
}}

Here is your input:
{{
'map':"{map}",
'external_msg':"{external_msg}",
'history_status':"{history_status}",
}}
'''
# this is the prompt for validating the shortest path according to tracked events
shortest_path_prompt='''
Task: Path Validation and Decision Making
This task involves analyzing the 'current_status' and the provided semantic map to determine if the proposed 'shortest_path' is valid under current conditions. Please don't try to code anything, just analyze the situation using general knowledge like a human.

Responsibilities:

1. Map Interpretation: Analyze the provided map in OSM (XML) format which contains all rooms inside a building. Each room has a similar but unique name indicating its sector and floor. Despite similar names, they maybe far away from each other, so do not make decisions based on the similarity. Additionally, each area/room may have semantic tags that provide further description about this specific area.
2. Decision Making: Evaluate the proposed 'shortest_path' based on the 'current_status' and map analysis, relying solely on facts. Determine if the 'shortest_path' is valid, meaning the rooms in the path are unaffected by 'current_status'. If the path is unaffected, set 'is_Valid' to true. If not, identify and list the room(s) that are not accessible under 'areas_to_Avoid'. Consider the current time and only include rooms in 'areas_to_Avoid' if they are directly afftected and inaccessible at that specific time. Set 'is_Valid' to false and update 'areas_to_Avoid' only when absolutely necessary, based on the map data without assumptions if you are not absolutely sure, do not put the area in 'areas_to_Avoid', put in 'areas_try_to_Avoid'. The robot is operating in a big building, therefore some place maybe occupancied by some event, we better avoid them, but you cannot be too critical otherwise the navigation task would fail because you decide too many place to avoid.
3. Based on semantic information from the map and current events, suggest additional rooms to avoid as less preferred paths (areas_try_to_Avoid). You don't have to output this only if you think it is necessary.

Inputs:

1. 'shortest_path': A list of rooms representing the shortest path to be evaluated. There maybe similar but not same names, please distinguish them carefully.
2. 'map': The building map provided in OSM (XML) format. Rooms and corridors are represented as areas, with the hierarchical structure of the building specified by the 'parent' tag otherwise they are not enclosed with each other. Sequential or similar names do not imply inclusion or proximity. There may be similar but not identical names, so please distinguish them carefully. Treat each room or area separately based on the facts provided, without making assumptions about their relationships.
3. 'current_status': Events tracked by another program that may affect path planning tasks. One event only affect one area if not saying otherwise.


Outputs:

A JSON object indicating Whether the current path is valid, a list of areas names to definitely avoid and a list of area names to preferably avoid (areas_try_to_Avoid), please only output area name that exist in the map to the list.


{{
  "is_Valid": true/false, 
  "areas_to_Avoid": ["AreaName1 (not description)", "AreaName2", ...], 
  "areas_try_to_Avoid": ["AreaName1", "AreaName2", ...], 
 }}
 
Here is your input:
{{
'map':"{map}",
'shortest_path':"{shortest_path}",
'current_status':"{current_status}",
}}
'''

def call_api_msg(osm_map_text,external_msg,event_history_json_file):

    if utility.retrieve_last_entry(event_history_json_file)!=None:
        current_last_entry = utility.retrieve_last_entry(event_history_json_file)['current_status']
    else:
        current_last_entry ='None'
    print(f"current_last_entry={current_last_entry}")
    if len(current_last_entry['current_events'])>0:
        current_last_entry={
        
            "current_date": "Jul 5, 2024",
            "current_time": "11:00 AM",
            "today_weather": "Sunny",
            "current_events": []
        
        }
    print(f"current_last_entry={current_last_entry}")
        
    history_last_entry=str(current_last_entry)
    print(f"{utility.GREEN}[NavigationEventMonitor]: sending prompt to openai api now {external_msg}, status: {history_last_entry}{utility.RESET}")

    prompt=msg_prompt.format(map=osm_map_text,external_msg=f"{external_msg}",history_status=history_last_entry)
    # print("!!!!!!!!!!!!!!!!!")
    # print(prompt)
    # print("!!!!!!!!!!!!!!!!!")
    pub_prompt.publish(prompt)
    params = utility.set_open_params() # 设置参数
    response = utility.get_completion_new(params,prompt)
    response_text = response.choices[0].message.content
    print(f"{utility.GREEN}[NavigationEventMonitor]: get external_msg response from openai api now #{response_text}#{utility.RESET}")
    return response_text

def call_api_shortest_path(osm_map_text,event_history_json_file,shortest_area_path):
    global last_event_prompt

    current_last_entry = utility.retrieve_last_entry(event_history_json_file)['current_status']

    history_last_entry=str(current_last_entry)

    prompt=shortest_path_prompt.format(map=osm_map_text,shortest_path=shortest_area_path,current_status=history_last_entry)
    print(f"{utility.GREEN}[NavigationEventMonitor]: sending shortest path prompt to openai api now {shortest_area_path}, status: {prompt[0:100]}{utility.RESET}")
    last_event_prompt=prompt
    pub_current_history_status.publish(history_last_entry)
    pub_prompt.publish(prompt)

    params = utility.set_open_params() # 设置参数
    response = utility.get_completion_new(params,prompt)
    response_text = response.choices[0].message.content
    print(f"{utility.GREEN}[NavigationEventMonitor]: get shortest path response from openai api now #{response_text}#{utility.RESET}")
    return response_text

# callback function for external_msg
def msg_callback(data):
    global pub_response
    global pub_response_all
    global pub_get_next_event
    global last_json_response
    print(f"{utility.GREEN}[NavigationEventMonitor]: receiving incoming_event msg, which is {data.data}{utility.RESET}")
    # means history file is ready, waiting for shortest path to approve
    if data.data=='navigation_task_start':
        return
    osm_map_text=utility.get_osm_string(config['osm_path_prefix']+"AccessCostEvaluatorOSM.osm")
    response=call_api_msg(osm_map_text,data,event_history_json_file)
    json_str,json_dict,Found=utility.check_response_legal(response)
    last_json_response=json_str.group(0)
    json_dict['event_msg']=data.data
    utility.save_to_json(json_dict,event_history_json_file)
    print(f"{utility.GREEN}[NavigationEventMonitor]: get response of event msg now, sending back to external info :{json_str.group(0)}{utility.RESET}")
    pub_response.publish(json_str.group(0))
    pub_response_all.publish(str(response))
    pub_get_next_event.publish(json_str.group(0))

# callback function for shortest path
def areas_path_callback(areas_path_msg):
    global pub_response
    global pub_response_all
    print(f"{utility.GREEN}[NavigationEventMonitor]: receiving areas_path from osmAGPathPlanning, checking for validatoin now{utility.RESET}")
    # use the map only has areas for now
    osm_map_text=utility.get_osm_string(config['osm_path_prefix']+"AccessCostEvaluatorOSM.osm")
    print(f"{utility.GREEN} [NavigationEventMonitor]: Getting shortest path, sending to openai api, waiting for result{utility.RESET}")
    response=call_api_shortest_path(osm_map_text,event_history_json_file,areas_path_msg.data)
    json_str,json_dict,Found=utility.check_response_legal(response)
    print(f"{utility.GREEN} [NavigationEventMonitor]: Getting shortest path api result now, sending back to osmAGPathPlanning{utility.RESET}")
    print(f"{utility.GREEN} [NavigationEventMonitor]: pub_response sending {type(json_str.group(0))}{utility.RESET}")
    pub_response.publish(str(json_str.group(0))) 
    pub_response_all.publish(str(response))
    # vis in rviz
    temp_text=f"[NavigationEventMonitor]: shortest area path: {areas_path_msg.data}\n Response: valid: {json.loads(json_str.group(0))['is_Valid']}, invalid areas: {json.loads(json_str.group(0))['areas_to_Avoid']}"
    over_lay_text_manager.set_text(temp_text,0)
    over_lay_text_manager.publish_text(0)

# interactive_with_api publish the response to response_json_str, listen to external_msg to call api to get the response
if __name__ == "__main__":
    rospy.init_node('NavigationEventMonitor', anonymous=True)
    print(f"{utility.GREEN}[NavigationEventMonitor]: NavigationEventMonitor is running now{utility.RESET}")
    rospy.Subscriber("external_msg", String, msg_callback)
    rospy.Subscriber("areas_path", StringList, areas_path_callback)
    pub_event_api_get_result = rospy.Publisher('event_monitor_api_result', String, queue_size=10)
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()