import xml.etree.ElementTree as ET
import json
import map_drawer

# Function used to generate a json file with doors with multiple passages in order to save experience of all passages
# There is no point to keep track of the only door to a room since there is no other choice.
def get_experience_json():
    osm_file = '/home/xiefujing/catkin_ws/src/osmAG_intelligent_navigation/osmAG/real/Jiajie_2F_ShanghaiTech_merge_F2_corrected_id2name_outside_structure_utm.osm'
    tree = ET.parse(osm_file)
    root = tree.getroot()
    areas=map_drawer.parse_osm(tree)
    # Initialize a dictionary to hold door data
    doors = {}
    lonely_doors = set()
    for area_name,area in areas.items():
        passages = area['passages']
        if len(passages)==1:
            print(list(passages.keys())[0])
            lonely_doors.add(list(passages.keys())[0])
            continue
    print(f"lonely_doors={lonely_doors}")
    for area_name,area in areas.items():
        for passage_id,passage_data in area['passages'].items():
            name = passage_data['name']
            # if passage_data['type']=='passage':
            if passage_id not in lonely_doors:
                doors[passage_id] = []

    with open('./experience_2floor.json', 'w') as f:
        json.dump(doors, f, indent=4)
    print("Initial door access data saved.")

# update door access fail/success to json file
def update_door_access(experience_file,door_name, result):
    with open(experience_file, 'r') as f:
        doors = json.load(f)
    if door_name in doors:
        doors[door_name].append(result)
    else:
        print(f'!!!!No door found with name {door_name}')
        pass
    with open(experience_file, 'w') as f:
        json.dump(doors, f, indent=4)

# add passage type from the osm file to the json file, this is used to a send in PassageCostEvaluator
def update_and_augment_json(json_experience_file, xml_file, output_json_file):
    with open(json_experience_file, 'r') as f:
        doors = json.load(f)
    tree = ET.parse(xml_file)
    root = tree.getroot()
    passage_types = {}
    # extract passage type information from the osm file
    for way in root.findall('way'):
        name = None
        passage_type = None
        if way.find("tag[@k='osmAG:type']")!=None and way.find("tag[@k='osmAG:type']").get('v')=='passage':
            for tag in way.findall('tag'):
                if tag.get('k') == 'name':
                    name=tag.get('v')
                if tag.get('k') == 'osmAG:passage_type':
                    passage_type = tag.get('v')
        if name and passage_type:
            passage_types[name] = passage_type
    # update the JSON data
    updated_doors = {}
    for door, experience in doors.items():
        new_name = door
        updated_doors[new_name] = {
            'experience': experience,
            'door_type': passage_types.get(new_name, 'unknown')  # Use passage type from XML or default to 'unknown'
        }
    with open(output_json_file, 'w') as f:
        json.dump(updated_doors, f, indent=4)
    print("Updated JSON data saved to", output_json_file)


if __name__ == '__main__':
    get_experience_json()