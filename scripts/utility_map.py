from pyproj import Transformer
from pyproj.transformer import Transformer
from collections import OrderedDict
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import networkx as nx

# transfer wgs84 to utm
def lat_lon_to_utm(latitude, longitude):
    #ACCORDING TO https://developers.arcgis.com/javascript/3/jshelp/pcs.htm WGS84 TO UTM 51N(SHANGHAI) is 32651
    transformer = Transformer.from_crs("epsg:4326", "epsg:32651")
    utm_easting, utm_northing = transformer.transform(latitude, longitude)
    return utm_easting, utm_northing

# transfer osm file from wgs to utm
def xml_from_wgs2utm(input_file, output_file):
    tree = ET.parse(input_file)
    root = tree.getroot()
    ordered_dict_id = OrderedDict()
    min_node_id=0
    min_east=10000000
    min_north=10000000
    for i,node in enumerate(root.findall('node')):
        # make its use less token
        del node.attrib['action']
        del node.attrib['visible']
        if(min_node_id>int(node.get('id'))):
            min_node_id=int(node.get('id'))
        ordered_dict_id[str(node.get('id'))]=str((i+1))
        node.set('id',str((i+1)))
        
        east,north=lat_lon_to_utm(float(node.get('lat')),float(node.get('lon')))
        if(min_east>east):
            min_east=east
        if(min_north>north):
            min_north=north
        node.set('x',str(round(east,2)))
        node.set('y',str(round(north,2)))
        del node.attrib['lat']
        del node.attrib['lon']
    for node in root.findall('node'):
        node.set('x',str(round(float(node.get('x'))-min_east,2)))
        node.set('y',str(round(float(node.get('y'))-min_north,2)))
        
    for way in root.findall('way'):
        for nd in way.findall('nd'):
            nd.set('ref',ordered_dict_id[str(nd.get('ref'))])
    xml_string = ET.tostring(root, encoding='unicode', method='xml')
    tree.write(output_file, "UTF-8",short_empty_elements=True)

# plot the nodes in the osm file
def plot_nodes(utm_file_name:str):
    tree = ET.parse(utm_file_name)  
    root = tree.getroot()
    x_coords = []
    y_coords = []
    for node in root.findall('node'):
        x = float(node.get('x'))
        y = float(node.get('y'))
        x_coords.append(x)
        y_coords.append(y)
    plt.figure(figsize=(8, 6))
    plt.scatter(x_coords, y_coords, color='blue', s=10)
    plt.title('Node Coordinates Plot')
    plt.xlabel('X Coordinates')
    plt.ylabel('Y Coordinates')
    plt.axis('equal')  # Set equal aspect ratio
    plt.grid(True)
    plt.show()

