
import xml.etree.ElementTree as ET
import utility_map
osm_path_prefix='./src/osmAG_intelligent_navigation/osmAG/real/'

def convert_wgs_2_cartesian(input_file,output_file):
    tree = ET.parse(input_file)
    root = tree.getroot()
    min_east=10000000
    min_north=10000000
    for i,node in enumerate(root.findall('node')):
        east,north=utility_map.lat_lon_to_utm(float(node.get('lat')),float(node.get('lon')))
        if(min_east>east):
            min_east=east
        if(min_north>north):
            min_north=north
        node.set('x',str(round(east,2)))
        node.set('y',str(round(north,2)))
        del node.attrib['lat']
        del node.attrib['lon']
    for node in root.findall('node'):
        node.set('x',str(float(node.get('x'))))
        node.set('y',str(float(node.get('y'))))
    tree.write(output_file, "UTF-8",short_empty_elements=True)

if __name__ == '__main__':
    utm_file_name='/2F_ShanghaiTech_merge'
    convert_wgs_2_cartesian(osm_path_prefix+utm_file_name+'.osm',osm_path_prefix+utm_file_name+'_utm.osm')