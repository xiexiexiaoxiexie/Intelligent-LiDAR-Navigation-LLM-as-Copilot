#!/home/xiefujing/anaconda3/envs/openai/bin/python3

import xml.etree.ElementTree as ET
import map_handler
import process_osm
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union
import networkx as nx
import math
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import nearest_points
from shapely.geometry import Point, Polygon
import json
from matplotlib.patches import Polygon as MplPolygon
from shapely.ops import nearest_points
from itertools import islice
from collections import Counter
from numpy.linalg import norm
import os
import utility

os.chdir('/home/xiefujing/catkin_ws')
with open('./src/osmAG_intelligent_navigation/scripts/config.json', 'r') as config_file:
    config = json.load(config_file)
transition_osm=(config['osm_utm_transform']['x'],config['osm_utm_transform']['y'])
avoid_floor2=config['avoid_floor2']

def create_polygon(way,nodes):
    polygon = [nodes[nd.get('ref')] for nd in way.findall('nd')]
    return polygon

def cal_centroid_poly(polygon):
    x_list = [vertex[0] for vertex in polygon]
    y_list = [vertex[1] for vertex in polygon]
    _len = len(polygon)
    x = sum(x_list) / _len
    y = sum(y_list) / _len
    return (x, y)

def create_visibility_graph(polygon_points, passage_points):
    poly = Polygon(polygon_points)
    graph = nx.Graph()
    for i, point in enumerate(polygon_points):
        next_point = polygon_points[(i + 1) % len(polygon_points)]
        graph.add_edge(point, next_point, weight=LineString([point, next_point]).length)
    all_points = polygon_points + passage_points
    for i, point_a in enumerate(all_points):
        for point_b in all_points[i+1:]:
            line = LineString([point_a, point_b])
            if poly.contains(line):
                graph.add_edge(point_a, point_b, weight=line.length)
    return graph

def create_hollow_visibility_graph(polygon_points, passage_points, hole_points):
    outer_poly = Polygon(polygon_points)
    bigger_hole_points=hole_points
    graph = nx.Graph()
    polygon_with_hole = Polygon(shell=polygon_points, holes=[hole_points])
    for i, point in enumerate(bigger_hole_points):
        next_point = bigger_hole_points[(i + 1) % len(bigger_hole_points)]
        graph.add_edge(point, next_point, weight=LineString([point, next_point]).length)
    # check visibility between all pairs of points (vertices and passages)
    all_points = passage_points+bigger_hole_points
    for i, point_a in enumerate(all_points):
        for point_b in all_points[i+1:]:
            line = LineString([point_a, point_b])
            # if point_a in passage_points or point_b in passage_points:
            # only add an edge if the line does not cross the polygon boundary
            if outer_poly.contains(line) and polygon_with_hole.contains(line):
                graph.add_edge(point_a, point_b, weight=line.length)
    for edge in graph.edges():
        plt.plot([edge[0][0], edge[1][0]], [edge[0][1], edge[1][1]], 'g--')
    plt.plot([passage_points[0][0], passage_points[1][0]], [passage_points[0][1], passage_points[1][1]], 'ro')
    return graph

def calculate_path_length(path):
    total_length = 0
    for i in range(1, len(path)):
        point_a = path[i - 1]
        point_b = path[i]
        segment_length = math.sqrt((point_b[0] - point_a[0]) ** 2 + (point_b[1] - point_a[1]) ** 2)
        total_length += segment_length
    return total_length

# find_path using nx
def find_path(graph, start, end):
    try:
        path = nx.astar_path(graph, start, end, weight='weight')
        name_path = [graph.nodes[node]['name'] for node in path]
        print(f"name_path= {name_path}")
        return path,name_path
    except nx.NetworkXNoPath:
        print("No valid path found between", start, "and", end)
        return None,None
    
def calculate_centroid(start_point, end_point):
    return ((start_point[0] + end_point[0]) / 2, (start_point[1] + end_point[1]) / 2)

def enlarge_polygon(vertices, scale_factor):
    vertices_array = np.array(vertices)
    centroid = np.mean(vertices_array, axis=0)
    scaled_vertices = centroid + (vertices_array - centroid) * scale_factor
    scaled_polygon = list(map(tuple, scaled_vertices))
    return scaled_polygon

def enlarge_polygon_shapely(vertices, scale_factor):
    poly = Polygon(vertices)
    enlarged_poly = poly.buffer(scale_factor - 1, resolution=16, join_style=2)
    return list(enlarged_poly.exterior.coords)

# parse osm file to areas structure
def parse_osm(tree):
    root = tree.getroot()
    nodes = {}
    areas={}
    for element in root:
        if element.tag == 'node':
            nodes[element.attrib['id']] = (float(element.attrib['x']), float(element.attrib['y']))
    for element in root:
        if element.tag == 'way':
            for tag in element.findall('tag'):
                # ignore all university
                if tag.attrib['v'] == 'university' :
                    break
                if tag.attrib['v'] == 'area':
                    area_name=element.find("tag[@k='name']").get('v')
                    area_nodes = []
                    passages={}
                    area={}
                    area['name']=area_name
                    for nd in element.findall('nd'):
                        # add node references to way
                        area_nodes.append(nodes[nd.attrib['ref']])
                    area['nodes']=area_nodes
                    area['centroid']=cal_centroid_poly(area_nodes)
                    area['area_type']=element.find("tag[@k='osmAG:areaType']").get('v') # room or structure
                    area['level_height']=(int(element.find("tag[@k='level']").get('v'))-1)*3 # area level height, be careful the stairs or elevator area is on level 1.
                    # area['area_usage']=element.find("tag[@k='osmAG:area_usage']").get('v')
                    for element_ in root:
                        if element_.tag == 'way':
                            for tag_ in element_.findall('tag'):
                                if tag_.attrib['v'] == 'passage':
                                    osm_from_name=element_.find("tag[@k='osmAG:from']").get('v')
                                    osm_to_name=element_.find("tag[@k='osmAG:to']").get('v')
                                    passage_name=element_.find("tag[@k='name']").get('v')
                                    passage_degree=element_.find("tag[@k='osmAG:degree']").get('v')
                                    passage_type=element_.find("tag[@k='osmAG:passage_type']").get('v')
                                    # passage level height is different from area level height
                                    passage_level_height=(int(element_.find("tag[@k='level']").get('v'))-1)*3
                                    node_refs=[]
                                    for nd in element_.findall('nd'):
                                        # add node references to way
                                        node_refs.append(nd.attrib['ref'])
                                    if osm_from_name==area_name or osm_to_name==area_name:
                                        passage_id=element_.find("tag[@k='name']").get('v')
                                        passages[passage_id]={}
                                        passages[passage_id]['coordinates']=[nodes[node_refs[0]],nodes[node_refs[1]]]
                                        passages[passage_id]['passage_id']=passage_id
                                        passages[passage_id]['name']=passage_name
                                        passages[passage_id]['degree']=passage_degree
                                        passages[passage_id]['passage_type']=passage_type
                                        passages[passage_id]['passage_level_height']=passage_level_height
                                        centroid=calculate_centroid(nodes[node_refs[0]],nodes[node_refs[1]])
                                        # due to round or whatever, the centroid is not guarteed to be inside the polygon, therefore shrink the polygon a little bit and calculate the nearset_point in the shrinked polygon as the centroid
                                        passages[passage_id]['centroid']=centroid
                                        if osm_from_name==area_name:
                                            passages[passage_id]['from']=area_name
                                            passages[passage_id]['to']=osm_to_name
                                        else:
                                            passages[passage_id]['from']=area_name
                                            passages[passage_id]['to']=osm_from_name
                    area['passages']=passages
                    areas[area_name]=area
    return areas

# function to save distances between all pairs of passages in an area to a json file
def cal_save_astar_paths(input_file, output_file,output_json_file,areas):
# output file is json file with path, maybe also with length
    tree = ET.parse(input_file)
    root = tree.getroot()
    plt.figure(figsize=(8, 8))
    plt.axis('equal')
    areas_paths_data = {}
    for element in root:
        if element.tag == 'way':
            for tag in element.findall('tag'):
                if tag.attrib['v'] == 'university' :
                    break
                if tag.attrib['v'] == 'area':
                    area_name=element.find("tag[@k='name']").get('v')
                    if len(areas[area_name]['passages'])>1:
                        areas_paths_data[area_name]=[]
                        print(f'!!!!!!!!!!!!area with more than one passages, area_name= {area_name}')
                        # calculate the path length between all pairs of passages
                        for start_passage_id, start_passage in areas[area_name]['passages'].items():
                            for end_passage_id, end_passage in areas[area_name]['passages'].items():
                                if start_passage_id != end_passage_id:
                                    start_passage_centroid = start_passage['centroid']
                                    end_passage_centroid = end_passage['centroid']
                                    polygon_nodes=areas[area_name]['nodes']
                                    polygon = Polygon(polygon_nodes)
                                    start_Point_centroid=Point(start_passage_centroid)
                                    end_Point_centroid=Point(end_passage_centroid)
                                    polygon_shrink=enlarge_polygon_shapely(polygon,0.95)
                                    polygon_shrink = Polygon(polygon_shrink)
                                    nearest_start_on_boundary = nearest_points(start_Point_centroid, polygon_shrink)[1]
                                    nearest_end_on_boundary = nearest_points(end_Point_centroid, polygon_shrink)[1]
                                    graph = create_visibility_graph(polygon_nodes, [(nearest_start_on_boundary.x,nearest_start_on_boundary.y), (nearest_end_on_boundary.x,nearest_end_on_boundary.y)])
                                    visibility_graph = nx.Graph()
                                    visibility_graph = nx.compose(visibility_graph, graph)
                                    polygon_x, polygon_y = zip(*areas[area_name]['nodes'])
                                    
                                    plt.plot(polygon_x, polygon_y, 'b-', label='Polygon Boundary')
                                    plt.plot([start_passage_centroid[0], end_passage_centroid[0]], [start_passage_centroid[1], end_passage_centroid[1]], 'ro', label='Passages')
                                    path,name_path = find_path(visibility_graph, (nearest_start_on_boundary.x,nearest_start_on_boundary.y), (nearest_end_on_boundary.x,nearest_end_on_boundary.y))
                                    path[0]=start_passage_centroid
                                    path[-1]=end_passage_centroid
                                    path_x, path_y = zip(*path)
                                    plt.plot(path_x, path_y, 'g--', label='Path')
                                    path_length = calculate_path_length(path)
                                    areas_paths_data[area_name].append({
                                        "start_passage": start_passage_id,
                                        "end_passage": end_passage_id,
                                        "path": path,
                                        "path_length": path_length
                                    })
                                    print(f'path_length= {path_length}')
                                    ET.SubElement(element, 'tag', {'k': start_passage_id+','+end_passage_id, 'v': str(round(path_length,2))})
    with open(output_json_file, 'w') as f:
        json.dump(areas_paths_data, f, indent=4)
    tree.write(output_file, "UTF-8",short_empty_elements=True)

def visualize_graph(graph):
    pos = {node: (node[0], node[1]) for node in graph.nodes()}
    nx.draw_networkx_nodes(graph, pos, node_size=5, node_color='red')
    nx.draw_networkx_edges(graph, pos, width=1, alpha=0.5)
    edge_labels = nx.get_edge_attributes(graph, 'weight')
    edge_labels = {e: round(w, 1) for e, w in edge_labels.items()}
    nx.draw_networkx_edge_labels(graph, pos, edge_labels=edge_labels, font_size=8,bbox=dict(facecolor='none', edgecolor='none', pad=0))
    plt.axis('equal')  # So the graph is not distorted

# function to save distances between all pairs of passages outside the building to a json file
def cal_save_astar_outside_paths(input_file, output_file,output_json_file,areas,outside_passage_ids):
    # numerical handle strategy: enlarge or shrink the polygon, make sure the nearest point on the boundary is inside the polygon, use the origin polygon to form the graph
    tree = ET.parse(input_file)
    root = tree.getroot()
    plt.figure(figsize=(8, 8))
    plt.axis('equal')
    areas_paths_data = {}
    for element in root:
        if element.tag == 'way':
            for tag in element.findall('tag'):
                if tag.attrib['v'] == 'university' :
                    break
                if tag.attrib['v'] == 'area':
                    area_name=element.find("tag[@k='name']").get('v')
                    if area_name=='E1-F1':
                        hole_polygon=areas[area_name]['nodes']
                    if area_name=='outside':
                        outer_polygon=areas[area_name]['nodes']
    for element in root:
        if element.tag == 'way':
            for tag in element.findall('tag'):
                if tag.attrib['v'] == 'university' :
                    break
                if tag.attrib['v'] == 'area':
                    area_name=element.find("tag[@k='name']").get('v')
                    if len(areas[area_name]['passages'])>1:
                        areas_paths_data[area_name]=[]
                        print(f'!!!!!!!!!!!!area with more than one passages, area_name= {area_name}')
                        # calculate the path length between all pairs of passages
                        for start_passage_id, start_passage in areas[area_name]['passages'].items():
                            for end_passage_id, end_passage in areas[area_name]['passages'].items():
                                if start_passage_id != end_passage_id and (start_passage_id in outside_passage_ids and end_passage_id in outside_passage_ids):
                                    start_passage_centroid = start_passage['centroid']
                                    end_passage_centroid = end_passage['centroid']
                                    hole_polygon_=Polygon(enlarge_polygon_shapely(hole_polygon,1.05))
                                    nearest_start_on_boundary = nearest_points(Point(start_passage_centroid), hole_polygon_.boundary)[1]
                                    nearest_end_on_boundary = nearest_points(Point(end_passage_centroid), hole_polygon_.boundary)[1]
                                    graph = create_hollow_visibility_graph(outer_polygon, [(nearest_start_on_boundary.x,nearest_start_on_boundary.y), (nearest_end_on_boundary.x,nearest_end_on_boundary.y)],hole_polygon)
                                    visibility_graph = nx.Graph()
                                    visibility_graph = nx.compose(visibility_graph, graph)
                                    polygon_x, polygon_y = zip(*areas[area_name]['nodes'])
                                    plt.plot(polygon_x, polygon_y, 'b-', label='Polygon Boundary')
                                    plt.plot([start_passage_centroid[0], end_passage_centroid[0]], [start_passage_centroid[1], end_passage_centroid[1]], 'ro', label='Passages')
                                    path,name_path = find_path(visibility_graph,(nearest_start_on_boundary.x,nearest_start_on_boundary.y), (nearest_end_on_boundary.x,nearest_end_on_boundary.y))
                                    # make sure the start and end point are the passage centroid, so no make inside one area or outside the area, it is using the same passage centroid
                                    path[0]=start_passage_centroid
                                    path[-1]=end_passage_centroid
                                    path_x, path_y = zip(*path)
                                    plt.plot(path_x, path_y, 'g--', label='Path')
                                    path_length = calculate_path_length(path)
                                    areas_paths_data[area_name].append({
                                        "start_passage": start_passage_id,
                                        "end_passage": end_passage_id,
                                        "path": path,
                                        "path_length": path_length
                                    })
                                    print(f'outside path_length= {path_length}')
                                    ET.SubElement(element, 'tag', {'k': start_passage_id+','+end_passage_id, 'v': str(round(path_length,2))})
    with open(output_json_file, 'w') as f:
        json.dump(areas_paths_data, f, indent=4)

def is_passage_in_areas(passage_id, areas):
    for area in areas.values():
        for passage in area["passages"].values():
            if passage["passage_id"] == passage_id:
                return True
    return False

def round_numbers(data):
    if isinstance(data, dict):
        return {key: round_numbers(value) for key, value in data.items()}
    elif isinstance(data, list):
        return [round_numbers(item) for item in data]
    elif isinstance(data, float):
        rounded = round(data, 3)
        # check if the rounded number is a whole number and convert to int if so
        if rounded.is_integer():
            return int(rounded)
        else:
            return rounded
    else:
        return data
    
def normalize_angle(angle):
    angle = angle % 360
    if angle > 90 and angle <= 270:
        angle -= 180
    elif angle > 270:
        angle -= 360
    return angle

def plot_polygon_with_label(ax, polygon, label):
    min_rect = polygon.minimum_rotated_rectangle
    rect_coords = np.array(min_rect.exterior.coords)
    center = np.mean(rect_coords, axis=0)
    edge1 = np.linalg.norm(rect_coords[0] - rect_coords[1])
    edge2 = np.linalg.norm(rect_coords[1] - rect_coords[2])
    width, height = min(edge1, edge2), max(edge1, edge2)
    angle = np.degrees(np.arctan2(*(rect_coords[1] - rect_coords[0])[::-1]))
    angle=normalize_angle(90+angle)
    fontsize = 0.5 * min(width,height)  
    ax.text(center[0], center[1], label, ha='center', va='center', fontsize=min(3,fontsize), rotation=angle)

# function to convert a seq of passage id to a seq of area name
def transfer_passage_id_2_area_seq(free_pass,passage_id_path,tree):
    root = tree.getroot()
    area_seq=[]
    area_seq.append(free_pass[0])
    next_area=free_pass[0]
    for passage_id in passage_id_path:
        for element in root:
            if element.tag == 'way':
                for tag in element.findall('tag'):
                    if tag.attrib['v'] == 'passage':
                        # if element.attrib['id']==passage_id:
                        if element.find("tag[@k='name']").get('v')==passage_id:
                            if next_area==element.find("tag[@k='osmAG:from']").get('v'):
                                area_seq.append(element.find("tag[@k='osmAG:to']").get('v'))
                                next_area=element.find("tag[@k='osmAG:to']").get('v')
                                break
                            if next_area==element.find("tag[@k='osmAG:to']").get('v'):
                                area_seq.append(element.find("tag[@k='osmAG:from']").get('v'))
                                next_area=element.find("tag[@k='osmAG:from']").get('v')
                                break
    return area_seq

def calculate_directed_normal(p1, p2, p3, p4):
    # P1,P2=passage node coordinates, P3,P4=start/end centroid coordinates, the normal should be less than 90 degree with path direction
    # Convert 2D points to 3D by appending a zero z-component
    p1, p2, p3, p4 = np.append(p1, 0), np.append(p2, 0), np.append(p3, 0), np.append(p4, 0)
    d12 = p2 - p1
    d34 = p4 - p3
    normal = np.array([-d12[1], d12[0], 0])
    if np.dot(normal, d34) < 0:
        normal = -normal  # Flip the normal if necessary
    normal_unit = normal / norm(normal)
    theta = math.atan2(normal_unit[1], normal_unit[0])
    qz = math.sin(theta / 2.0)
    qw = math.cos(theta / 2.0)
    quaternion =[0, 0, qz, qw]
    normal_unit[0]=(p1[0]+p2[0])/2
    normal_unit[1]=(p1[1]+p2[1])/2
    # normal_unit is passage centroid for now, for send move-base goal
    return normal_unit, quaternion

def get_path_normal(shortest_path,passage_id_path,areas):
    # shortest_path utm
    path_normal=[]
    # in near zero
    passage_lines=[]
    normals=[]
    for i,passage_id_1 in enumerate(passage_id_path):
        for area in areas.values():
            find=False
            for passage in area["passages"].values():
                if passage["passage_id"] == passage_id_1:
                    passage_line = passage["coordinates"]
                    passage_lines.append(passage_line)
                    find=True
                    break
            if find:
                break
    for i,centroid_1 in enumerate(shortest_path):
        if i==len(shortest_path)-2:
            break
        p3=[centroid_1[0]-transition_osm[0],centroid_1[1]-transition_osm[1]]
        centroid_2 = shortest_path[(i + 1) % len(shortest_path)]
        p4=[centroid_2[0]-transition_osm[0],centroid_2[1]-transition_osm[1]]
        normal_unit, quaternion=calculate_directed_normal(np.array(passage_lines[i][0]),np.array(passage_lines[i][1]),np.array(p3),np.array(p4))
        path_normal.append(quaternion)
        normals.append(normal_unit)
    # normals is passage centroid + normal in place orientation,path_normal is quaternion
    return normals,path_normal

# transfer seq of points to seq of passage_id
def tranfer_path_2_passage_id(shortest_path,shortest_name_path,areas_paths,areas,no_pass):
    point_path=[]
    # delete start and end area name
    passage_id_path=shortest_name_path[1:-1]
    area_path=[shortest_name_path[0]]
    for i,this_passage in enumerate(passage_id_path):
        if i==len(passage_id_path)-1:
            break
        next_passage = passage_id_path[(i + 1) % len(passage_id_path)]
        # may have several sandwich area, choose the one with shortest path length that is not in no_pass
        shortest_area_name=None
        shortest_area_length=100000
        shortest_path_point=[]
        for area_name,area_paths in areas_paths.items():
            if area_name in no_pass:
                continue
            for path_ in area_paths:
                if path_['start_passage']==this_passage and path_['end_passage']==next_passage:
                    if path_['path_length']<shortest_area_length:
                        shortest_area_name=area_name
                        shortest_area_length=path_['path_length']
                        shortest_path_point=path_['path']
        point_path+=shortest_path_point
        area_path.append(shortest_area_name)         
    return passage_id_path,point_path,area_path

def get_sandwich_area_between_passageid(passage_id_path,tree,no_pass):
    root = tree.getroot()
    sandwich_area=None
    area_seq=[]
    # area_seq.append(free_pass[0])
    # next_area=free_pass[0]
    for passage_id in passage_id_path:
        for element in root:
            if element.tag == 'way':
                for tag in element.findall('tag'):
                    if tag.attrib['v'] == 'passage':
                        # if element.attrib['id']==passage_id:
                        if element.find("tag[@k='name']").get('v')==passage_id:
                            if element.find("tag[@k='osmAG:to']").get('v') in no_pass or element.find("tag[@k='osmAG:from']").get('v') in no_pass:
                                continue
                            area_seq.append(element.find("tag[@k='osmAG:to']").get('v'))
                            area_seq.append(element.find("tag[@k='osmAG:from']").get('v'))
    item_counts = Counter(area_seq)
    sandwich_area=[item for item, count in item_counts.items() if count > 1]
    return sandwich_area

def add_edge_if_valid(polygon_with_hole,G, p1, p2, name1, name2):
    line = LineString([p1, p2])
    if polygon_with_hole.contains(line):
        G.add_edge(name1, name2, weight=line.length)

def cal_outside_pose_2_passage(polygon_with_hole,robot_pose,areas,boundary_point,destination_passage_name,hole_polygon):
    print(f"passage_name={destination_passage_name}, robot_pose= {robot_pose},boundary_point= {boundary_point}")
    G = nx.Graph()
    # Add the inside and boundary points as nodes
    G.add_node((robot_pose.x, robot_pose.y), pos=(robot_pose.x, robot_pose.y))
    G.add_node((boundary_point.x, boundary_point.y), pos=(boundary_point.x, boundary_point.y))
    for passageid, passage in areas['outside']['passages'].items():
        G.add_node(passageid, pos=Point(passage['centroid'][0], passage['centroid'][1]))
    for node in polygon_with_hole.exterior.coords:
        G.add_node(node, pos=(node[0], node[1]))
    for node in polygon_with_hole.interiors[0].coords:
        G.add_node(node, pos=(node[0], node[1]))
    nodes = list(G.nodes(data=True))
    for i, (name1, data1) in enumerate(nodes):
        for j, (name2, data2) in enumerate(nodes):
            if i < j:
                add_edge_if_valid(polygon_with_hole,G, data1['pos'], data2['pos'], name1, name2)
    shortest_path = nx.shortest_path(G, source=(robot_pose.x, robot_pose.y), target=(boundary_point.x, boundary_point.y), weight='weight')
    # Calculate the distance
    distance = nx.shortest_path_length(G, source=(robot_pose.x, robot_pose.y), target=(boundary_point.x, boundary_point.y), weight='weight')
    print(f"Shortest path: {shortest_path}")
    print(f"Distance: {distance}")
    return distance

# build graph to find the shortest path including cost from PassageCostEvaluator, delete edge in infeasible area
def build_graph(areas,areas_paths,free_pass,no_pass,areas_try_to_Avoid,color,tree,visited_areas,omit_area=None,invalid_search=0,passage_extra_weight=None, current_pose=None,infeasible_passage=None):
    ##################only for tesing
    if passage_extra_weight is not None:
        passage_extra_weight=passage_extra_weight['door_costs']
    to_floor2_passages=config['elevator_floor2_passage']+config['stairs_floor2_passage']
    # assume robot current pose is in start area
    if current_pose!=None:
        # current pose is near zero, json is in utm, transfer pose back to utm
        current_pose[0]+=transition_osm[0]
        current_pose[1]+=transition_osm[1]
    # areas_paths is the json
    # we want to visit as much as areas as possible, so we increase the weight of the edge in visited area
    shortest_graph = nx.Graph()
    start_area_name=free_pass[0]
    end_area_name=free_pass[1]
    x_coords, y_coords=None,None
    start_area_level_height=int(areas[free_pass[0]]['level_height'])
    start_area_centroid=[current_pose[0],current_pose[1],start_area_level_height]
    # tranverse all the passages in the start/end area, add the edge from current pose to the passage centroid
    for area_passage in areas[free_pass[0]]['passages'].values():
        passage_centroid=area_passage['centroid']
        passage_centroid_height=area_passage['passage_level_height']
        passage_name=area_passage['passage_id']
        if passage_name in infeasible_passage:
                print(f"!!!!!!!!!!!!!!!!!!!passage_name= {passage_name}, infeasible_passage= {infeasible_passage}")
                continue
        if free_pass[0]=='outside':
            print(f"start area in outside {passage_centroid}, {current_pose}, {transition_osm}")
            passage_centroid__=(passage_centroid[0]-transition_osm[0],passage_centroid[1]-transition_osm[1])
            current_pose__=[current_pose[0]-transition_osm[0],current_pose[1]-transition_osm[1]]
            hole_polygon=areas['E1-F1']['nodes']
            outer_polygon=areas['outside']['nodes']
            polygon_with_hole = Polygon(shell=outer_polygon, holes=[hole_polygon])
            hole_polygon_=Polygon(enlarge_polygon_shapely(hole_polygon,1.05))
            print(f"passage_centroid[0]={passage_centroid__[0]}30000, hole_polygon_.boundary={hole_polygon_.boundary}100, polygon_with_hole={polygon_with_hole}100, hole_polygon={hole_polygon}100")
            point_on_boundary = nearest_points(Point(passage_centroid__[0], passage_centroid__[1]), hole_polygon_.boundary)[1]
            distance=cal_outside_pose_2_passage(polygon_with_hole,Point(current_pose__[0],current_pose__[1]),areas,point_on_boundary,passage_name,hole_polygon)
            pass
        else:
            distance=math.sqrt((passage_centroid[0]-start_area_centroid[0])**2+(passage_centroid[1]-start_area_centroid[1])**2+(passage_centroid_height-start_area_level_height)**2)
        if passage_extra_weight is not None and area_passage['passage_id'] in passage_extra_weight:
            distance+=passage_extra_weight[area_passage['passage_id']]['cost']
        shortest_graph.add_node((start_area_centroid[0],start_area_centroid[1],start_area_centroid[2]),name=start_area_name)
        shortest_graph.add_node((passage_centroid[0],passage_centroid[1],passage_centroid_height),name=passage_name)
        shortest_graph.add_edge((start_area_centroid[0],start_area_centroid[1],start_area_centroid[2]), (passage_centroid[0],passage_centroid[1],passage_centroid_height),weight=distance)

    end_area_level_height=int(areas[free_pass[1]]['level_height'])
    end_area_pose=[areas[free_pass[1]]['centroid'][0],areas[free_pass[1]]['centroid'][1],end_area_level_height]
    end_area_centroid=end_area_pose
    for area_passage in areas[free_pass[1]]['passages'].values():
        passage_centroid=area_passage['centroid']
        passage_centroid_height=area_passage['passage_level_height']
        passage_name=area_passage['passage_id']
        if passage_name in infeasible_passage:
            print(f"!!!!!!!!!!!!!!!!!!!passage_name= {passage_name}, infeasible_passage= {infeasible_passage}")
            continue
        distance=math.sqrt((passage_centroid[0]-end_area_pose[0])**2+(passage_centroid[1]-end_area_pose[1])**2+(passage_centroid_height-end_area_pose[2])**2)
        if passage_extra_weight is not None and area_passage['passage_id'] in passage_extra_weight:
            distance+=passage_extra_weight[area_passage['passage_id']]['cost']
        shortest_graph.add_node((end_area_pose[0],end_area_pose[1],end_area_pose[2]),name=end_area_name)
        shortest_graph.add_node((passage_centroid[0],passage_centroid[1],passage_centroid_height),name=passage_name)
        shortest_graph.add_edge((end_area_pose[0],end_area_pose[1],end_area_pose[2]), (passage_centroid[0],passage_centroid[1],passage_centroid_height),weight=distance)
    
    for area_name,area_paths in areas_paths.items():
        # free_path edge already added, and no_pass is the areas has to avoid
        if area_name in free_pass or area_name in no_pass:
            continue
        area_level_height=int(areas[area_name]['level_height'])
        for path in area_paths:
            start_passage = path["start_passage"]
            end_passage = path["end_passage"]
            start_passage_height=(areas[area_name]['passages'][start_passage]['passage_level_height'])
            end_passage_height=(areas[area_name]['passages'][end_passage]['passage_level_height'])
            # delete the passage we know is infeasible
            if start_passage in infeasible_passage or end_passage in infeasible_passage:
                # print(f"!!!!!!!!!!!!!!!!!!!start_passage= {start_passage},end_passage= {end_passage}, infeasible_passage= {infeasible_passage}")
                continue
            # if not test going to second floor, then not include passages go to second floor
            if config['avoid_floor2'] and (start_passage in to_floor2_passages or end_passage in to_floor2_passages):
                continue
            # If either the start or end passage is in the areas, plot the path
            # build the graph based on the cropped areas, not all areas
            if is_passage_in_areas(start_passage, areas) and is_passage_in_areas(end_passage, areas):
                x_coords, y_coords = zip(*path["path"])
                sandwich_area=get_sandwich_area_between_passageid([start_passage,end_passage],tree,no_pass)
                if not sandwich_area:
                    continue
                # add config cost from openai output
                edge_weight=math.sqrt(path['path_length']**2+(start_passage_height-end_passage_height)**2)
                if passage_extra_weight!=None:
                    if start_passage in passage_extra_weight:
                        edge_weight+=passage_extra_weight[start_passage]['cost']
                    if end_passage in passage_extra_weight:
                        edge_weight+=passage_extra_weight[end_passage]['cost']
                if len(sandwich_area)>1:
                    pass
                if sandwich_area[0] in areas_try_to_Avoid:
                    edge_weight+=10
                visited_counts = Counter(visited_areas)
                if sandwich_area[0] in visited_areas:
                    pass
                if shortest_graph.has_edge((path["path"][0][0],path["path"][0][1],start_passage_height), (path["path"][-1][0],path["path"][-1][1],end_passage_height)):
                    current_weight = shortest_graph[(path["path"][0][0],path["path"][0][1],start_passage_height)] [(path["path"][-1][0],path["path"][-1][1],end_passage_height)]['weight']
                    if edge_weight < current_weight:
                        shortest_graph.add_node((path["path"][0][0],path["path"][0][1],start_passage_height),name=start_passage)
                        shortest_graph.add_node((path["path"][-1][0],path["path"][-1][1],end_passage_height),name=end_passage)
                        shortest_graph[(path["path"][0][0],path["path"][0][1],start_passage_height)][(path["path"][-1][0],path["path"][-1][1],end_passage_height)]['weight'] = edge_weight
                else:
                    shortest_graph.add_node((path["path"][0][0],path["path"][0][1],start_passage_height),name=start_passage)
                    shortest_graph.add_node((path["path"][-1][0],path["path"][-1][1],end_passage_height),name=end_passage)
                    shortest_graph.add_edge((path["path"][0][0],path["path"][0][1],start_passage_height), (path["path"][-1][0],path["path"][-1][1],end_passage_height),weight=edge_weight)
    return shortest_graph,start_area_centroid,end_area_centroid,x_coords,y_coords

#  areas_paths is the json file
def find_shortest_path(areas,areas_paths,free_pass,no_pass,areas_try_to_Avoid,color,tree,passage_experience=None,current_pose=None,infeasible_passage=[]):
    invalid_search=0
    visited_areas=[]
    point_path_x,point_path_y=[],[]
    multi_passageid_path=[]
    color_list=['g','c','m','y','k','#33DFFF' ]
    # point_path_ is the path with mediate points inside all areas, without start and end area
    # passage_id_path_ only include passage_ids
    # shortest_name_path is the start_area_name+passage_ids+end_area_name
    # shortest_path from current_pose+passage centroid+end_area_centroid
    # 
    for i in range(1):
        # start_area_centroid add area_height to current pose
        shortest_graph,start_area_centroid,end_area_centroid,x_coords, y_coords=build_graph(areas,areas_paths,free_pass,no_pass,areas_try_to_Avoid,color,tree,visited_areas,None,invalid_search,passage_experience,current_pose,infeasible_passage)
        # shortest_path using passage centroid coordinates, from one passage to other without mediate point, on other words staight line
        if current_pose== None:
            shortest_path, shortest_name_path=find_path(shortest_graph, (start_area_centroid[0],start_area_centroid[1],start_area_centroid[2]),(end_area_centroid[0],end_area_centroid[1],end_area_centroid[2]))
        else: 
            shortest_path, shortest_name_path=find_path(shortest_graph, (start_area_centroid[0],start_area_centroid[1],start_area_centroid[2]),(end_area_centroid[0],end_area_centroid[1],end_area_centroid[2]))
            print(f"shortest_path= {shortest_path}")
            print("shortest_name_path= ",shortest_name_path)
        # deal with infeasible path due to too much area to avoid
        if shortest_path==None:
            print(f"shortest_path is None!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!{invalid_search}")
            return None,None,None,None

        passage_id_path_,point_path_,area_path=tranfer_path_2_passage_id(shortest_path,shortest_name_path,areas_paths,areas,no_pass)
        passage_centroid,shortest_path_normal=get_path_normal(shortest_path,passage_id_path_,areas)
        # area_path=transfer_passage_id_2_area_seq(free_pass,passage_id_path_,tree)
        if passage_id_path_ in multi_passageid_path:
            print(f"same passage_id_path!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!{invalid_search}")
            invalid_search+=1
            continue
        multi_passageid_path=passage_id_path_
        visited_areas+=transfer_passage_id_2_area_seq(free_pass,passage_id_path_,tree)
        # area_path=transfer_passage_id_2_area_seq(free_pass,passage_id_path_,tree)
        point_path_x.append([point[0] for point in point_path_])
        point_path_y.append([point[1] for point in point_path_])
        print(f"point_path_x= {point_path_x}")
    for  i,(point_x,point_y) in enumerate(zip(point_path_x,point_path_y)):
        plt.plot(point_x, point_y, color_list[i%len(color_list)], linewidth=1)
    return shortest_path,area_path,shortest_path_normal,multi_passageid_path

def load_plot_json_paths(input_tree,json_file_path,areas,cropped_areas,free_pass,no_pass,areas_try_to_Avoid,example_free_pass,passage_experience=None,current_pose=None,infeasible_passage=[]):
    print(f"{utility.GREEN}[map_drawer]: map_drawer planning now, free_path= {free_pass}, no_path= {no_pass}, areas_try_to_Avoid= {areas_try_to_Avoid}{utility.RESET}")
    # use free_pass to represent start and destination area
    # plot all areas 
    # only plot relevant passages(start/end/passages in areas with more than two passages)
    # do not plot structure
    # calculate the shortest path between start and end area
    with open(json_file_path, 'r') as f:
        areas_paths = json.load(f)
    plt.axis('equal')
    root = input_tree.getroot()
    for element in root:
        if element.tag == 'way':
            if element.find("tag[@k='osmAG:areaType']")!=None and element.find("tag[@k='osmAG:areaType']").get('v')=='structure':
                continue
            for tag in element.findall('tag'):
                if tag.attrib['v'] == 'area':
                    area_name=element.find("tag[@k='name']").get('v')
                    try:
                        polygon_x, polygon_y = zip(*cropped_areas[area_name]['nodes'])
                    except KeyError:
                        continue
                    poly_area = Polygon(cropped_areas[area_name]['nodes'])
                    minx, miny, maxx, maxy = poly_area.bounds
                    bbox_width = maxx - minx
                    bbox_height = maxy - miny
                    plt.plot(polygon_x, polygon_y, 'b-',linewidth=0.5)
                    plot_polygon_with_label(ax, poly_area, area_name)
                    if area_name in cropped_areas:
                        if len(cropped_areas[area_name]['passages'])>1 or area_name in free_pass:
                            for passage_id, passage in cropped_areas[area_name]['passages'].items():
                                passage_coordinates = passage['coordinates']
                                passage_coordinates_x, passage_coordinates_y = zip(*passage_coordinates)
                                plt.plot(passage_coordinates_x, passage_coordinates_y, 'r-',linewidth=0.8)
    shortest_centroid_path,shortest_area_path,shortest_path_normal,multi_passageid_path=find_shortest_path(areas,areas_paths,free_pass,no_pass,areas_try_to_Avoid,'b--',input_tree,passage_experience,current_pose,infeasible_passage)
    if len(example_free_pass)>0:
        find_shortest_path(cropped_areas,areas_paths,example_free_pass,'y--',input_tree)
    '''
    shortest area path: start area name+inter area
    shortest_path_normal: only have passage centroid direction 
    shortest_centroid_path: start +passage centroid+end area center
    multi_passageid_path:  only inter passage names
    
    '''
    return shortest_area_path,shortest_path_normal,shortest_centroid_path,multi_passageid_path

def generate_additional_text_4_figure(input_file, output_file,cropped_areas):
    tree = ET.parse(input_file)
    process_osm.del_all_node(input_file,output_file)
    process_osm.cleanup_way(output_file,output_file)  
    process_osm.del_all_parents(output_file,output_file)   

def prepare_for_PassageCostEvaluator(osm_file,output_file):
    # delete all passages and nodes, only with areas that has name
    process_osm.del_all_node(osm_file,output_file)
    process_osm.cleanup_way(output_file,output_file)  
    process_osm.del_all_passages(output_file,output_file)


