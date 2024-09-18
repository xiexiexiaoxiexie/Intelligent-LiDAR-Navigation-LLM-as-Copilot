import json
import os
import xml.etree.ElementTree as ET
import openai
import re
import matplotlib.path as mplPath
import numpy as np
from openai import OpenAI
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
RESET = '\033[0m'
ORANGE = '\033[38;5;208m'
PURPLE = '\033[95m'
BLUE = '\033[94m'
CYAN = '\033[96m'
LIGHT_GRAY = '\033[37m'
DARK_GRAY = '\033[90m'
################ openai related
os.chdir('/home/xiefujing/catkin_ws')
with open('./src/osmAG_intelligent_navigation/scripts/config.json', 'r') as config_file:
    config = json.load(config_file)
openai.proxy = config['proxy']
openai_model = config['llm']['openai_model']
openai_temperature = config['llm']['temperature']
openai_max_tokens= config['llm']['max_tokens']
osm_path_prefix=config['osm_path_prefix']
utm_file_name=config['utm_file_name']

# set openai params
def set_open_params(
    model=openai_model, # model
    temperature=openai_temperature, 
    n = 1, 
    max_tokens=openai_max_tokens, 
    top_p=1, 
    frequency_penalty=0,
    presence_penalty=0, 
):
    openai_params = {}    
    openai_params['model'] = model  
    openai_params['temperature'] = temperature 
    openai_params['n'] = n  
    openai_params['max_tokens'] = max_tokens  
    openai_params['top_p'] = top_p  
    openai_params['frequency_penalty'] = frequency_penalty  
    openai_params['presence_penalty'] = presence_penalty  
    return openai_params

# get response from openai
def get_completion(params, prompt):
    messages = [{"role": "user", "content": prompt}]  # 用户角色发送一个消息，消息的内容为我们的提示文本
    response = openai.chat.completions.create(  # 调用ChatCompletion API
        model = params['model'], # 模型
        messages=messages, # Prompt消息
        temperature = params['temperature'], # 多样性
        n = params['n'], # 返回结果的数量
        max_tokens = params['max_tokens'], # 生成结果的最大词数
        top_p = params['top_p'], # 随机抽样的阈值
        frequency_penalty = params['frequency_penalty'], # 生成的文本的常见度
        presence_penalty = params['presence_penalty'], # 生成的文本中的新概念出现的频率
    )
    return response # 接收模型的聊天返回结果

# openai changeed its api interface, use this one
def get_completion_new(params, prompt):
    os.environ["http_proxy"] = config['proxy']['http']
    os.environ["https_proxy"] = config['proxy']['https']
    os.environ["HTTP_PROXY"] = config['proxy']['http']
    os.environ["HTTPS_PROXY"] = config['proxy']['https']
    os.environ['OPENAI_API_KEY'] = config['openai_api_key']
    # print(f"prompt={prompt}")
    # 2024.07.21 api interfence changed
    client = OpenAI()
    print("ready to send get_completion_new api request, waiting for result now")
    completion = client.chat.completions.create(model=params['model'],
                                                temperature=params['temperature'],
                                                messages=[{"role": "system", "content": "You are a helpful assistant."},{"role": "user", "content": prompt}])
    return completion 

# check if the response json str is legal
def check_response_legal(response):
    json_str = re.search(r'(?<=```json\n)([\s\S]*?)(?=\n```)', response, re.DOTALL)
    json_dict=None
    if json_str:
        try:
            json_dict = json.loads(json_str.group(0))
        except json.JSONDecodeError as e:
            print(f"Failed to decode JSON at position {e.pos}: {e.msg}")
    else:
        print(f"{RED}Error:---------------------------------No JSON data from response found.{RESET}")
    return json_str,json_dict,json_dict is not None

def save_to_json(data, file_path='data.json'):
    # check if the file exists
    if os.path.exists(file_path):
        with open(file_path, 'r', encoding='utf-8') as file:
            existing_data = json.load(file)
    else:
        existing_data = []
    existing_data.append(data)
    with open(file_path, 'w', encoding='utf-8') as file:
        json.dump(existing_data, file, ensure_ascii=False, indent=4)
    print(f"json file successfully saved to {file_path}")

# Open the file and retrieve the last dictionary
def retrieve_last_entry(file_path='data.json'):
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            data = json.load(file)
            if isinstance(data,list) and len(data)>0:
                return data[-1]
            else:
                return None
    except FileNotFoundError:
        print(f"{RED}json file not found{RESET}, creating a new one, file_path={file_path}")
        base_data = [
            {
                "current_status": {
                    "current_date": "",
                    "current_time": "",
                    "today_weather": "sunny",
                    "current_events": [
                        {
                            "event_name": "",
                            "event_data": "",
                            "event_duration": "",
                            "duration": "",
                            "place": "",
                            "msg_text": ""
                        }
                    ]
                }
            }
        ]
        with open(file_path, 'w', encoding='utf-8') as file:
            json.dump(base_data, file, ensure_ascii=False, indent=4)
        return 'None'
    
def retrieve_json_data(json_data,key):
    try:
        return json_data[key]
    except KeyError:
        return None
    
# convert osm file to string
def get_osm_string(input_file):
    AccessCostEvaluator_tree=ET.parse(input_file)
    return ET.tostring(AccessCostEvaluator_tree.getroot(), encoding='utf-8', method='xml')

# extract the section from a md file, used to get external info
def extract_md_section(filepath, section_header):
    with open(filepath, 'r', encoding='utf-8') as file:
        lines = file.readlines()
    target = section_header.strip().lower()
    inside_section = False
    extracted_text = []
    current_level = target.count('#')  
    for line in lines:
        if line.startswith('#'):
            clean_line = line.strip().lower()
            level = clean_line.count('#')
            
            if level <= current_level:
                if inside_section:
                    break  
                inside_section = (clean_line == target)
                continue  
        if inside_section:
            extracted_text.append(line)
    return ''.join(extracted_text)

# check if the pose is inside the polygon/area
def check_pose_inside_polygon( current_pose,polygon_vertices,area_name=None,contour_vertices=None):
    pose_x = current_pose.pose.pose.position.x
    pose_y = current_pose.pose.pose.position.y
    if area_name=='outside':
        polygon_path = mplPath.Path(np.array(contour_vertices))
        is_inside = not polygon_path.contains_point((pose_x, pose_y))
        return is_inside

    polygon_path = mplPath.Path(np.array(polygon_vertices))
    is_inside = polygon_path.contains_point((pose_x, pose_y))
    return is_inside

# locate robot in which area
def locate_robot_area(current_pose,areas):
    inside_area_name=None
    for area_name, area_data in areas.items():
            if check_pose_inside_polygon(current_pose,area_data['nodes']):
                # areas contain structures and rooms
                if area_data['area_type']=='structures':
                    print(f"{ORANGE}current pose is INSIDE {area_name} structure now, CHECKING ALL AREAS NOW{RESET}")
                    continue
                if area_data['area_type']=='room':
                    inside_area_name=area_name
                    continue
    return inside_area_name

def get_text_from_md(title):
    external_msg=extract_md_section("./src/osmAG_intelligent_navigation/external_info/email.md",'### '+title)
    return external_msg