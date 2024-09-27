import xml.etree.ElementTree as ET
from utility_map import *
import xmltodict
import json
import copy

# remove empty way
def check_remove_way(input_file,output_file):
    tree = ET.parse(input_file)
    root = tree.getroot()
    for way in root.findall('way'):
        if any(tag.get('k') == 'osmAG:type'  for tag in way.findall('tag')):
            continue
        else:
            root.remove(way)
    tree.write(output_file,encoding='utf-8', xml_declaration=True)

def remove_parent_and_structure(input_file,output_file):
    tree = ET.parse(input_file)
    root = tree.getroot()
    for way in root.findall('way'):
        for tag in way.findall('tag'):
            if(tag.attrib['k']=='osmAG:parent'):
                way.remove(tag)
            if(tag.attrib['v']=='structure'):
                root.remove(way)
    tree.write(output_file,encoding='utf-8', xml_declaration=True)

# check for 'half' passage(only one or none room connected to it)
def check_invalid_passage(input_file,output_file):
    tree = ET.parse(input_file)
    root = tree.getroot()
    for way in root.findall('way'):
        from_=-10000000
        to_=-10000000
        for tag in way.findall('tag'):
            if(tag.attrib['k']=='osmAG:from'):
                from_=tag.attrib['v']
            if(tag.attrib['k']=='osmAG:to'):
                to_=tag.attrib['v']
            if(tag.attrib['k']=='osmAG:type' and tag.attrib['v']=='passage'):
                count=0
                for way_ in root.findall('way'):
                    if any(tag.get('v') == 'room' for tag in way_.findall('tag')):
                        for tag in way_.findall('tag'):
                            room_name='__'
                            if(tag.attrib['k']=='name') :
                                room_name=tag.attrib['v']
                            if room_name==from_ or room_name==to_:
                                count+=1
                    
                if(count<=1):
                    print(f'invalid passage found, from_={from_}, to_={to_}')
                    root.remove(way)
    tree.write(output_file,encoding='utf-8', xml_declaration=True)
  
def areaid2semantic(input_file, output_file):
    tree = ET.parse(input_file)
    root = tree.getroot()
    for way in root.findall('way'):
        for tag in way.findall('tag'):
            if(tag.attrib['k']=='osmAG:from' or tag.attrib['k']=='osmAG:to' or tag.attrib['k']=='osmAG:parent'):
                id=tag.attrib['v']
                for way in root.findall('way'):
                    if(way.attrib['id']==id):
                        for tag_ in way.findall('tag'):
                            # name_element = tag_.find('name')
                            # Get the 'v' attribute value from the 'tag' element
                            if tag_.attrib['k']=='name':
                                print("setting from or to"+id+" to "+tag_.get('v'))
                                tag.set('v',tag_.get('v'))
    tree.write(output_file,encoding='utf-8', xml_declaration=True)

# del height,indoor,level,action, visible in way
def cleanup_way(input_file,output_file):
    # delete all ref node in way 
    tree = ET.parse(input_file)
    root = tree.getroot()
    for way in root.findall('way'):
        del way.attrib['action']
        del way.attrib['visible']
        for tag in way.findall('tag'):
            if(tag.attrib['k']=='height' or tag.attrib['k']=='indoor' or tag.attrib['k']=='level'):
                way.remove(tag)
            if tag.attrib['k']=='name' and any(tag.get('v') == 'passage' for tag in way.findall('tag')):
                way.remove(tag)
        for nd in way.findall('nd'):
            way.remove(nd)
    tree.write(output_file,encoding='utf-8', xml_declaration=True)

def del_all_parents(input_file,output_file):
    tree = ET.parse(input_file)
    root = tree.getroot()
    for way in root.findall('way'):
        for tag in way.findall('tag'):
            if(tag.attrib['k']=='parent' or tag.attrib['k']=='osmAG:parent'):
                way.remove(tag)
    tree.write(output_file,encoding='utf-8', xml_declaration=True)

def del_all_passages(input_file,output_file):
    tree = ET.parse(input_file)
    root = tree.getroot()
    for way in root.findall('way'):
        if any(tag.get('k') == 'osmAG:type' and tag.get('v') == 'passage' for tag in way.findall('tag')):
            root.remove(way)
    tree.write(output_file,encoding='utf-8', xml_declaration=True)

def del_all_node(input_file,output_file):
    tree = ET.parse(input_file)
    root = tree.getroot()
    for node in root.findall('node'):
        root.remove(node)
    tree.write(output_file,encoding='utf-8', xml_declaration=True)

def xml2json(xml_file,json_name):
    with open(xml_file, 'r') as file:
        xml_string = file.read()
    dict_data = xmltodict.parse(xml_string)
    json_data = json.dumps(dict_data, indent=4)
    with open(json_name, 'w') as json_file:
        json_file.write(json_data)

# delete all leaves in the map that is not in the free_pass set, delete no_pass set
def del_all_leaves(input_tree,output_file,free_pass,no_pass):
    output_tree=copy.deepcopy(input_tree)
    root = output_tree.getroot()
    # key=area name, value=set of connected area name
    passage_dict={}
    all_name_set=set()
    for way in root.findall('way'):
        if any(tag.get('k') == 'osmAG:type' and tag.get('v') == 'passage' for tag in way.findall('tag')):
            # if way.find("tag[@k='osmAG:from']") in no_pass or way.find("tag[@k='osmAG:to']") in no_pass:
            #     continue
            if way.find("tag[@k='osmAG:from']") is not None and way.find("tag[@k='osmAG:to']") is not None:
                if way.find("tag[@k='osmAG:from']").get('v') !='None' and way.find("tag[@k='osmAG:to']").get('v')!='None':
                    all_name_set.add(way.find("tag[@k='osmAG:from']").get('v'))
                    all_name_set.add(way.find("tag[@k='osmAG:to']").get('v'))
                    empty_set=set()
                    empty_set1=set()
                    passage_dict.get(way.find("tag[@k='osmAG:from']").get('v'), empty_set).add(str(way.find("tag[@k='osmAG:to']").get('v')))
                    if(len(empty_set)==1):
                        passage_dict[way.find("tag[@k='osmAG:from']").get('v')]=empty_set
                    
                    passage_dict.get(way.find("tag[@k='osmAG:to']").get('v'), empty_set1).add(way.find("tag[@k='osmAG:from']").get('v'))
                    if(len(empty_set1)==1):
                        passage_dict[way.find("tag[@k='osmAG:to']").get('v')]=empty_set1
    # delete area with only one passage
    for way in root.findall('way'):
        if any(tag.get('k') == 'osmAG:type' and tag.get('v') == 'area' for tag in way.findall('tag')):
            name=way.find("tag[@k='name']").get('v')
            if (len(passage_dict.get(name, empty_set)) <=1 and name not in free_pass) or name in no_pass :
                root.remove(way)
    # remove the passage connected to the removed area    
    for way in root.findall('way'):
        appera_time=0
        if any(tag.get('k') == 'osmAG:type' and tag.get('v') == 'passage' for tag in way.findall('tag')):
            # print(way.attrib['id'])
            from_tag = way.find("tag[@k='osmAG:from']").get('v')
            to_tag = way.find("tag[@k='osmAG:to']").get('v')

            for way_ in root.findall('way'):
                if any(tag.get('k') == 'osmAG:type' and tag.get('v') == 'area' for tag in way_.findall('tag')):
                    name=way_.find("tag[@k='name']").get('v')
                    if from_tag==name or to_tag==name:
                        appera_time+=1
            # only one or none of the areas are still in the map
            if appera_time<=1:
                root.remove(way)
    # delete unused node
    used_node_refs = set()
    for way in root.findall('way'):
        for nd in way.findall('nd'):
            ref = nd.get('ref')
            if ref:
                used_node_refs.add(ref)
    for node in root.findall('node'):
        if node.get('id') not in used_node_refs:
            root.remove(node)
    if output_file!=None:
        output_tree.write(output_file,encoding='utf-8', xml_declaration=True)
    return output_tree

