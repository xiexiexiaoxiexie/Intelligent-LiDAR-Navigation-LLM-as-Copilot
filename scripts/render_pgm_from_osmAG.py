#!/usr/bin/env python3
import numpy as np
from PIL import Image, ImageDraw
import map_drawer
import xml.etree.ElementTree as ET
import json
from nav_msgs.msg import OccupancyGrid
import rospy

with open('./src/osmAG_intelligent_navigation/scripts/config.json', 'r') as config_file:
    config = json.load(config_file)

# occupancy grid map parameters
transition_osm=(config['osm_utm_transform']['x'],config['osm_utm_transform']['y'])
map_width, map_height = 5000, 3000  
resolution = 0.05  # resolution in meters/pixel
unknown_gray_value = 205

# translate the biiiiiig utm coordinates to near zero for easier visualization and rendering
def tanslate_osm(areas):
    # centroid is not transferred in area or passage
    for area_name, area_data in areas.items():
        nodes = area_data['nodes']
        centroid_utm = area_data['centroid']
        centroid_ = (centroid_utm[0]-transition_osm[0],centroid_utm[1]-transition_osm[1])
        # donot translate the nodes multiple times
        if nodes[0][0]<10000:
            print("already translated, return")
            return
        for i in range(len(nodes)):
            nodes[i] = (nodes[i][0]-transition_osm[0],nodes[i][1]-transition_osm[1])
        
        area_data['nodes'] = nodes
        area_data['centroid'] = centroid_
        passages = area_data.get('passages', [])
        for passage_id,passage_data in passages.items():
            coordinates = passage_data['coordinates']
            for i in range(len(coordinates)):
                coordinates[i] = (coordinates[i][0]-transition_osm[0],coordinates[i][1]-transition_osm[1])
            passage_data['coordinates'] = coordinates
            area_data['passages'] = passages
    return areas

def meters_to_pixels(coords):
    return [(int(x / resolution), map_height-int(y / resolution)) for x, y in coords]

# according to the osmAGPathPlanner module, render the occupancy grid map in order to send to move_base
# passages not connected the areas sequences should be marked as black, to make sure the move_base pannner follow the path by osmAG planner
def render_pgm(areas,yaml_filename, areas_path=None,multi_passageid_path=None,building_contour=None,end_area=None):
    # Initialize the image with gray for unknown areas
    # for areas with 2 passage, use the shortest path to only include one passage in the image
    map_img = Image.new('L', (map_width, map_height), unknown_gray_value)
    draw = ImageDraw.Draw(map_img)
    rooms = []
    if building_contour is not None:
        rooms.append(meters_to_pixels(building_contour))
    else: 
        print("!!!!!!!!!!!!!!!!!building_contour is None!!!!!!!!!!!!!!!!!!!!")
    for area_name, area_data in areas.items():
        nodes = area_data['nodes']
        nodes = meters_to_pixels(nodes)
        if area_name=='outside':
            continue
        if area_name=='E1c-F1' or area_name=='E1b-F1' or area_name=='E1d-F1':
            continue
        if area_name=='E1-F1':
            # draw.polygon(nodes, fill=0)
            draw.line(nodes+ [nodes[0]], fill=0, width=2)
            continue
        rooms.append(nodes)
    
    # only draw the doors between the areas in the areas_path, close other passages 
    doors = []
    areas_path_whole=areas_path.append(end_area)
    for area_name, area_data in areas.items():
        next_item = None
        if areas_path_whole is not None:
            index_ = areas_path_whole.index(area_name)
            if index_ < len(areas_path_whole) - 1:
                next_item = areas_path_whole[index_ + 1]
        passages = area_data.get('passages', [])
        for passage_id,passage_data in passages.items():
            if passage_id not in multi_passageid_path:
                continue
            coordinates = passage_data['coordinates']
            coordinates = meters_to_pixels(coordinates)
            if areas_path_whole is None:
                doors.append(coordinates)
            if (passage_data['from']==area_name and passage_data['to']==next_item) or (passage_data['to']==area_name and passage_data['from']==next_item) and areas_path_whole is not None:
                doors.append(coordinates)
    print(f"size of doors={len(doors)},size of rooms={len(rooms)}")
    for room in rooms:
        # Fill the room with white
        draw.polygon(room, fill=255)
        # Draw the boundary with black
        draw.line(room + [room[0]], fill=0, width=1)
    # Draw doors (as white lines or rectangles)
    for door in doors:
        draw.line(door, fill=255, width=3)  
    # save the PGM file
    pgm_filename = yaml_filename[0:-4]+'pgm'
    parts = yaml_filename.split('/')
    map_img.save(pgm_filename)
    # create YAML file
    yaml_content = f"""image: {'./'+parts[-1].split('.')[0]+'.pgm'}
resolution: {resolution}
origin: [0.0, 0.0, 0.0]  # Assuming the bottom-left corner of the map at (0,0) in world coordinates
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
"""
    with open(yaml_filename, 'w') as yaml_file:
        yaml_file.write(yaml_content)
    print(f"Map and YAML files created: {pgm_filename}, {yaml_filename}")
    return pgm_filename, yaml_filename

# coor means coordinates in gazebo world, utm is in osmAG world
def get_pixel_from_coor(coordinate):
    return (int(coordinate[0] / resolution), map_height-int(coordinate[1] / resolution))

def get_utm_from_coors(coordinate):
    return (coordinate[0]+transition_osm[0],coordinate[1]+transition_osm[1])

def choose_areas(chosen_areas,areas):
    chosen_areas_dict={}
    for area_name in chosen_areas:
        chosen_areas_dict[area_name]=areas[area_name]
    return chosen_areas_dict


if __name__ == '__main__':
    osm_path_prefix='./src/osmAG_intelligent_navigation/osmAG/real/'
    utm_file_name='ShanghaiTech_merge_F2_corrected_id2name_outside_structure_utm'
    map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    
    areas_tree = ET.parse(osm_path_prefix+utm_file_name+'.osm')
    areas=map_drawer.parse_osm(areas_tree)
    yaml_filename = './osm_occupancy_map_.yaml'
    tanslate_osm(areas)
    render_pgm(areas,yaml_filename)

    part_areas=choose_areas(['E1d-F1-13','E1d-F1-COR-01','E1d-F1-06'],areas)    
    yaml_filename_part = './src/osmAG_intelligent_navigation/osmAG/occupancy_grid_map/osm_occupancy_map_part.yaml'
    pgm_filename, yaml_filename=render_pgm(part_areas,yaml_filename_part,['E1d-F1-13','E1d-F1-COR-01','E1d-F1-06'])



    