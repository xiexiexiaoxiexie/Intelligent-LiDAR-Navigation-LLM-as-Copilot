#!/home/xiefujing/anaconda3/envs/openai/bin/python3
import json
import map_drawer
import os
import time
import utility 
import rospy
from std_msgs.msg import String
import logging
import save_experience
import simple_command_move_base
from osmAG_intelligent_navigation.msg import StringList

logging.disable(logging.CRITICAL)
os.chdir('/home/xiefujing/catkin_ws')
with open('./src/osmAG_intelligent_navigation/scripts/config.json', 'r') as config_file:
    config = json.load(config_file)
osm_file=config['osm_path_prefix']+config['osm_file']
os.environ["http_proxy"] = config['proxy']['http']
os.environ["https_proxy"] = config['proxy']['https']
os.environ['OPENAI_API_KEY'] = config['openai_api_key']
pub_prompt = rospy.Publisher('PassageCostEvaluator_prompt', String, queue_size=10)
pub_response = rospy.Publisher('PassageCostEvaluator_response', String, queue_size=10)
trial_free_pass__=['E1d-F1-14', 'E1c-F1-08']
history_ready=False

# PassageCostEvaluator prompt
PassageCostEvaluator_prompt='''
You are a robot tasked with navigating inside and outside of large buildings containing multiple rooms and doors. Your task involves advanced path planning to reach specific destinations. You need to plan a path ahead and go through several doors to get to your destination. Doors within the building may vary in their accessibility due to human interactions and environmental changes, affecting your ability to pass through them autonomously. You sure could ask human to help open the door, but that could cost time, and not guaranteed to be successful. Some doors might even require special access privileges not universally available.

To efficiently plan your path, you utilize a historical data file that records your past experiences with each door, including accessibility and the type of door. This file helps you estimate the potential time and energy costs associated with each door, influencing your path choices. Note that the 'experience' on the file is sequential, the robot record each door it tried to pass, the length of experience means how many times the robot has tried to pass through the door. 

Your goal is to generate a JSON configuration that outlines the cost associated with each door. Do not try to use code or math to calculate any thing, try to analyse like a human that make door selection decision. The costs should reflect your preference for doors based on ease of access—doors that are typically easier to pass through should be assigned lower costs. And please be brief in your response.

Inputs:
1. Human Command: Natural language instructions provided by humans.
2. Building Map: The layout of the office in OSM (XML) format.
3. Experience File: Data from your previous navigations, detailing your experience with going through different doors.
4. Current Location: The specific room you are presently located in.

Responsibilities:
1. Process the Human Command to determine the start and destination rooms based on the provided map.
2. Analyze the Experience File to assess the cost of traversing through each door, enhancing the efficiency of the navigation algorithm. If a door has been consistently inaccessible in recent attempts, it would be unwise to repeatedly attempt it, so assign a high cost to avoid it. Be generous with the costs for doors you have evidence to avoid, but ensure the costs for other doors are not overly arbitrary; otherwise, the algorithm might favor paths with fewer doors. Use 10 as the base cost for doors with no evidence, use 0 for doors that are highly likely open right now and use higher value for doors that according to experience, you think is currently not accessible. The cost is not mandatory, you could ajust the cost based on the experience data.

Outputs:
Format: JSON{{door_costs: {{door_costs}}, navigation_task: {{navigation_task}}
1. door_costs: A string in JSON format that specifies the cost associated with all doors within the experience file.
Example content: 
    "B1C-F1-09_to_B1C-F1-COR-04": {{
      "cost": 10 (as in meters)
    }}, all other doors in the experience file
2. navigation_task: output a list of start and destination room, for example: ['E1d-F1-06', 'E1c-F1-09'].

Here is you input:
1. Human Command: "{human_natural_language_command}"
2. Building Map: "{osm_map_xml}"
3. Experience File: "{experience_data}"
4. Current Location: "{current_area}"
'''

def call_api(human_natural_language_command,osm_map_str,experience_data,current_area):
    prompt=PassageCostEvaluator_prompt.format(human_natural_language_command=human_natural_language_command,osm_map_xml=osm_map_str,experience_data=experience_data,current_area=current_area)
    pub_prompt.publish(prompt)
    params = utility.set_open_params() # 设置参数
    print(f"{utility.YELLOW}[PassageCostEvaluator]: Access passage costs now, sending to openai api, waiting for result{utility.RESET}")
    response = utility.get_completion_new(params,prompt)
    response_text = response.choices[0].message.content
    response_text_arxiv=f"\n \n \n model: {config['llm']['openai_model']} \n"+response_text
    with open(config['osmAG_intelligent_navigation_prefix']+'gpt4o_response', 'a') as file:  # 'a' opens the file in append mode
        file.write(response_text_arxiv + '\n') 
    return response_text

def deal_with_llm_response(response_text,passage_experience_from_openai,free_pass_start,free_pass_end,free_pass_pub):
    # check response legal
    json_str,json_dict,Found=utility.check_response_legal(response_text)
    print(f"{utility.YELLOW}[PassageCostEvaluator]: Found={Found}, {utility.RESET}")
    if Found:
        passage_costs=json_dict['door_costs']
        navigation_task=json_dict['navigation_task']
        # check if navigation_task from openai is correct, used in case experiment
        if navigation_task[0]!=free_pass_start or navigation_task[1]!=free_pass_end:
            print(f"{utility.RED}[PassageCostEvaluator]: navigation_task[0]!=free_pass_start or navigation_task[1]!=free_pass_end, navigation_task_api={navigation_task}, free_pass= {[free_pass_start,free_pass_end]}, navigation task is wrong, abort now{utility.RESET}")
            return
        print(f'{utility.YELLOW}[PassageCostEvaluator]: Got api response_text, navigation_task is: {navigation_task}')
        utility.save_to_json(json_dict, file_path=passage_experience_from_openai)
        str_list = StringList()
        str_list.data =navigation_task
        # publish the start and destination to osmAG path planning module
        free_pass_pub.publish(str_list)
        print(f"{utility.YELLOW}[PassageCostEvaluator]: INITIALIZING ROBOT NAVIGATION, pulishing {str_list.data}{utility.RESET}")
    else:
        print(f'{utility.YELLOW}[PassageCostEvaluator]: No legal JSON response_text found')

def move_base_fail_callback(data):
    # data [current area, destination area, failed passage]
    print(f"{utility.YELLOW} [PassageCostEvaluator]: Move base failed, now replan{utility.RESET}")

def history_ready_callback(data):
    global history_ready
    history_ready=True
    print(f"{utility.YELLOW}[PassageCostEvaluator]: history_ready_callback, history_ready={history_ready}{utility.RESET}")

# used in experiment, with known destinatin area
def run(free_pass_start,free_pass_end):
    free_pass_pub = rospy.Publisher('free_pass', StringList, queue_size=10)
    case_num_arg = rospy.get_param('/case_num', 'case1')# case1
    human_natural_language_command = rospy.get_param('/human_natural_language_command', '[]')
    print(f"{utility.YELLOW}[PassageCostEvaluator]: free_pass_start: {free_pass_start}{utility.RESET}")
    cases_folder=config['cases_folder']
    # the experience file itself, move_base_command will update this file based on the result of the navigation
    experience_file=cases_folder+case_num_arg+config['experience_file']
    # send to openai with experience and door type
    experience_to_openai_file=cases_folder+case_num_arg+config['experience_to_openai_file']
    # result from openai
    passage_experience_from_openai=cases_folder+case_num_arg+config['passage_experience_from_openai']
    print(f"{utility.YELLOW}[PassageCostEvaluator]: passage_experience_from_openai: {passage_experience_from_openai}{utility.RESET}")
    print(f"{utility.YELLOW}[PassageCostEvaluator]: case_num_arg: {case_num_arg}{utility.RESET}")
    osm_file=config['osm_path_prefix']+config['osm_file']
    #  get osm send to PassageCostEvaluator, only keeps areas, used to figure out the start and destination area name from natural language command 
    PassageCostEvaluator_osm_file=config['osm_path_prefix']+"AccessCostEvaluatorOSM.osm"
    map_drawer.prepare_for_PassageCostEvaluator(osm_file,PassageCostEvaluator_osm_file)
    PassageCostEvaluator_osmstr=utility.get_osm_string(PassageCostEvaluator_osm_file)
    # get while osm file to fill passage experience json file
    # get experience file send to openai with door type and experience
    save_experience.update_and_augment_json(experience_file, osm_file, experience_to_openai_file)
    # save_experience.update_and_augment_json(experience_json_file, osm_file, experience_to_openai_file)
    # get json file with experience and door_type
    if config['useExperience']:
        with open(experience_to_openai_file, 'r') as json_file:
            experience_json_file_ = json.load(json_file)
    else:
        experience_json_file_="no experience"
    response_text=call_api(human_natural_language_command,PassageCostEvaluator_osmstr,experience_json_file_,free_pass_start)
    pub_response.publish(response_text)
    deal_with_llm_response(response_text,passage_experience_from_openai,free_pass_start,free_pass_end,free_pass_pub)

def trial_free_pass_cb(data):
    print(f"{utility.YELLOW}[PassageCostEvaluator]:receiving the trial: {data.data}{utility.RESET}")
    trial_free_pass__[0]=data.data[0]
    trial_free_pass__[1]=data.data[1]
    run(data.data[0],data.data[1])
     
if __name__ == "__main__":
    rospy.init_node('PassageCostEvaluatorNode', anonymous=True)
    while (not rospy.has_param('/case_num') or not rospy.has_param('/human_natural_language_command') or not rospy.has_param('/world_file')) and not rospy.is_shutdown():
        print(f"{utility.YELLOW}[PassageCostEvaluator]: waiting for parameter to be set")
        time.sleep(1)
    sub_trial_free_pass = rospy.Subscriber('trial_free_pass', StringList, trial_free_pass_cb)
    rospy.Subscriber("move_base_fail", StringList, move_base_fail_callback)
    pub_navigation_result=rospy.Publisher('navigation_result', String, queue_size=10)
    time.sleep(2)
    pub_navigation_result.publish("Parameter_ready")
    rospy.spin()

