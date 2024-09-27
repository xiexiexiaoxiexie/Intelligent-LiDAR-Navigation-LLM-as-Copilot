#!/home/xiefujing/anaconda3/envs/openai/bin/python3
import rospy
import json
import time
import xml.etree.ElementTree as ET
import logging
import math
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import ColorRGBA
import save_experience
import render_pgm_from_osmAG as map_generator
import map_drawer
import process_osm
import os
import utility
from std_msgs.msg import String
from osmAG_intelligent_navigation.msg import StringList
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

os.chdir('/home/xiefujing/catkin_ws')
logging.disable(logging.CRITICAL)

# note:
# /move_base_simple/goal Type: geometry_msgs/PoseStamped    /move_base/goal   Type: move_base_msgs/MoveBaseActionGoal

# the class interact with move_base, and send goal to move_base
class RobotNavigation:
    def __init__(self, free_pass, no_path=[], areas_try_to_Avoid=[],case_num='case2',over_lay_text_manager=None):
        print(f"{utility.ORANGE}[simple_command_move_base]: initilizing RobotNavigation obj{utility.RESET}")
        # robot1 is the one using move_base
        # self.client1 = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)
        # robot2 is the smart robot with a Ph.D. in PathPlanning
        self.client2 = actionlib.SimpleActionClient('robot2/move_base', MoveBaseAction)
        self.robot2_goal = None
        rospy.Subscriber("/robot2/odom", Odometry, self.gazebo_odom_listener)
        rospy.Subscriber("/robot2/move_base/result", MoveBaseActionResult, self.move_base_result_callback)
        # data [current area, destination area, failed passage], used to tell fail status
        self.pub_move_base_fail=rospy.Publisher("move_base_fail", StringList, queue_size=10)
        self.pub_change_map = rospy.Publisher('/change_map', String, queue_size=10)
        self.pub_astar_fail = rospy.Publisher('/Astar_fail', String, queue_size=10)
        self.pub_actual_areas_path=rospy.Publisher("actual_areas_path",StringList,queue_size=10)
        self.pub_actual_passages_path=rospy.Publisher("actual_passages_path",StringList,queue_size=10)
        self.current_pose = None
        self.next_area=None
        self.next_passage=None  #only change when the passage is successfully passed
        self.current_goal_passage=None   #current goal passage, change when the robot is close the next_passage and send the next goal
        self.passed_passage=None #successfully passed passage
        self.shortest_area_path = None # path in area name
        self.multi_passageid_path = None #path in passage id
        # utm, not transferred to near zero
        self.shortest_path_normal = None # passage path's normal, used to set move_base goal
        self.shortest_centroid_path = None # passage path's centroid, used to set move_base goal
        self.config = self.load_config()
        self.case_num=case_num
        self.use_experience=self.config['useExperience']
        # utm cordinate is too large, let's transfer to near zero for better visualization
        self.transition_osm = (self.config['osm_utm_transform']['x'], self.config['osm_utm_transform']['y'])
        # prefix of osm file"
        self.osm_path_prefix = self.config['osm_path_prefix']
        self.pathlength_json=self.config['path_length_json']
        self.grid_map_yaml = self.config['grid_map_yaml']
        self.utm_file_name = self.config['utm_file_name'] #osm file in utm cordinate
        self.experience_file = self.config['cases_folder']+self.case_num+self.config['experience_file']
        #save success and failed to pass some passage
        self.passage_experience_from_openai = self.config['cases_folder']+self.case_num+self.config['passage_experience_from_openai'] # passage cost evaluateed by openai
        self.free_pass = free_pass  #start and destination area
        self.actual_areas_path=[]
        self.actual_passages_path=[]
        self.current_area=self.free_pass[0]
        self.actual_areas_path=[self.free_pass[0]]
        self.areas_tree = ET.parse(self.osm_path_prefix+self.utm_file_name+'.osm')
        self.areas = map_drawer.parse_osm(self.areas_tree)
        self.areas=map_generator.tanslate_osm(self.areas)
        print(f"finished transfer areas")

        self.cropped_areas = None # areas without leaves, no point to go into one leaf area and go out
        self.no_path=no_path # areas to avoid
        self.areas_try_to_Avoid=areas_try_to_Avoid #areas with extra cost
        self.infeasible_passage=[] # infeasible path, not able to pass
        self.wait=None # move_base return status
        self.PLANNER_FAILED=False #bool sign
        self.move_base_result_status=None
        self.total_travel_distance=0
        self.total_travel_time=0
        self.last_position = None
        self.Bsimulate_command_move_base=False
        self.area_index=0
        # visualization
        self.over_lay_text_manager=over_lay_text_manager
        self.over_lay_text_manager.add_text_box('/robot2_status', '', 600, 120, 10, 570, 12, 2, ColorRGBA(0.0, 0.5, 1.0, 1.0), ColorRGBA(0.0, 0.0, 0.0, 0.7))
        self.place_robot_in_start_area_gazebo(self.free_pass[0])    
        print(f"{utility.ORANGE}[simple_command_move_base]: finishing initilizing RobotNavigation obj{utility.RESET}")

    # place the robot in the start area in Gazebo
    def place_robot_in_start_area_gazebo(self,start_area_name):
        rospy.wait_for_service('/gazebo/set_model_state')  # Wait for the service to be available
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)  # Create a service proxy
            # Define the model state
            model_state = ModelState()
            model_state.model_name = 'robot2'
            model_state.pose = Pose()
            if self.areas[start_area_name]['centroid'][0]>10000:
                print(f"{utility.RED}areas not transfered to near zero yet")
            model_state.pose.position.x = self.areas[start_area_name]['centroid'][0]
            model_state.pose.position.y = self.areas[start_area_name]['centroid'][1]
            model_state.pose.position.z = self.areas[start_area_name]['level_height']
            model_state.pose.orientation.x = 0
            model_state.pose.orientation.y = 0
            model_state.pose.orientation.z = 0
            model_state.pose.orientation.w = 1

            model_state.twist = Twist()
            model_state.twist.linear.x = 0
            model_state.twist.linear.y = 0
            model_state.twist.linear.z = 0
            model_state.twist.angular.x = 0
            model_state.twist.angular.y = 0
            model_state.twist.angular.z = 0
            model_state.reference_frame = ''  # empty string means world frame
            # Call the service
            response = set_state(model_state)
            print(response)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def crop_areas(self):
        cropped_tree=process_osm.del_all_leaves(self.areas_tree,self.osm_path_prefix+self.utm_file_name+'_feedin.osm',self.free_pass,self.no_path)
        cropped_tree=process_osm.del_all_leaves(cropped_tree,self.osm_path_prefix+self.utm_file_name+'_feedin.osm',self.free_pass,self.no_path)
        cropped_tree=process_osm.del_all_leaves(cropped_tree,self.osm_path_prefix+self.utm_file_name+'_feedin.osm',self.free_pass,self.no_path)
        self.cropped_areas = map_drawer.parse_osm(cropped_tree)
    
    def load_config(self):
        with open('./src/osmAG_intelligent_navigation/scripts/config.json', 'r') as config_file:
            return json.load(config_file)

    def create_move_base_action_goal(self, x, y, z, qx, qy, qz, qw):
        if rospy.get_param('second_floor')==True:
            z=3
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "world"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose.position = Point(x, y, z)
        move_base_goal.target_pose.pose.orientation = Quaternion(qx, qy, qz, qw)
        return move_base_goal

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2 + (point1.z - point2.z) ** 2)
    
    # find next area according to the area path
    def get_next_area(self):
        if self.shortest_area_path is not None:
            # index_ = self.shortest_area_path.index(self.current_area)
            indices = [index for index, element in enumerate(self.shortest_area_path) if element == self.current_area]
            for index in indices:
                if index == self.area_index:
                    index_ = index
                    self.area_index+=1
            if self.area_index < len(self.shortest_area_path) - 1:
                self.next_area = self.shortest_area_path[self.area_index]
                print(f"setting next area={self.next_area}")
            else:
                self.next_area = self.free_pass[1]
                print(f"setting next area={self.next_area}")
        else:
            print(f"{utility.ORANGE}Error!!!shortest_area_path is None{utility.RESET}")
            self.next_area = None

    def command_move_base(self):
        print(f"{utility.ORANGE}[simple_command_move_base]command_move_base start{utility.RESET}")
        
        part_areas=map_generator.choose_areas(self.shortest_area_path,self.areas)
        # the outside contour of the building
        building_contour=self.areas['E1-F1']['nodes']
        # render and send part map to map_server, only render chosed areas and open chosed passages
        pgm_filename, yaml_filename=map_generator.render_pgm(part_areas,self.grid_map_yaml,self.shortest_area_path,self.multi_passageid_path,building_contour,self.free_pass[1])
        # waiting for map to be saved
        time.sleep(5)
        # send the map to map_server
        self.pub_change_map.publish("change_map")
        self.client2.wait_for_server()

        # robot1
        position_1=self.shortest_centroid_path[-2]
        orientation_1=self.shortest_path_normal[-1]
        move_base_goal_1=self.create_move_base_action_goal(position_1[0]-self.transition_osm[0],position_1[1]-self.transition_osm[1],0,orientation_1[0],orientation_1[1],orientation_1[2],orientation_1[3])
        # self.client1.send_goal(move_base_goal_1)
        # self.current_area=self.shortest_area_path[0]
        # self.current_goal_passage=self.multi_passageid_path[0]
        # self.passed_passage=self.multi_passageid_path[0]
        if len(self.multi_passageid_path)>1:
            self.next_passage=self.multi_passageid_path[0]
            self.current_area_passage=self.multi_passageid_path[0]
        else: 
            self.next_passage=None
            print(f"{utility.ORANGE}ONLY ONE PASSAGE TO GO!{utility.RESET}")

        # BUG:
        index=-1000
        print(f"{utility.ORANGE}client2 is ready{utility.RESET}")
        if self.config['avoid_floor2']==False:
            temp_list=[]
            for i in range(len(self.shortest_centroid_path[1:-1])):
                temp_list.append([0,0,0,1])
            self.shortest_path_normal=temp_list
        for i,(position,orientation) in enumerate(zip(self.shortest_centroid_path[1:-1],self.shortest_path_normal)):
            print(f"i={i},size of shortest_centroid_path={self.shortest_centroid_path[1:-1]},size of shortest_path_normal={self.shortest_path_normal}")
            if i<index:
                print(f"{utility.ORANGE}i={i},index={index},skip{utility.RESET},self.shortest_centroid_path[1:-1]:{self.shortest_centroid_path[1:-1]}")
                continue

            if self.PLANNER_FAILED:
                print(f"{utility.ORANGE}WHOOPS, PLANNER_FAILED, {self.next_passage} IS NOT ACCESSIBLE, break for{utility.RESET}")
                break
            self.robot2_goal=self.create_move_base_action_goal(position[0]-self.transition_osm[0],position[1]-self.transition_osm[1],0,orientation[0],orientation[1],orientation[2],orientation[3])
            
            self.current_goal_passage=self.multi_passageid_path[i]

            self.client2.send_goal(self.robot2_goal)

            self.distance_to_goal=self.calculate_distance(self.current_pose.pose.pose.position,self.robot2_goal.target_pose.pose.position)
            print(f"{utility.ORANGE}MoveBaseGoal has been sent, go to passage: {self.current_goal_passage}, distance_to_goal:{self.distance_to_goal}, all path:{self.shortest_area_path}{utility.RESET}")
            self.over_lay_text_manager.set_text(f"[osmAGPathPlanner]: move_base Goal has been sent, go to passage: {self.current_goal_passage}, distance_to_goal:{self.distance_to_goal}, all path:{self.shortest_area_path}",1)
            self.over_lay_text_manager.publish_text(1)

            building_contour=self.areas['E1-F1']['nodes']
            # Wait for the server to finish performing the action
            # set the next goal when the robot is close to the current goal, so the robot don't stop at the current goal
            # self.wait==true means failed to reach the goal
            # BUG: True if the goal finished. False if the goal didn't finish within the allocated timeout
            
            if (self.next_area=='E1-P2' or self.current_area=='E1-P2') and self.config['avoid_floor2']==False:
                    self.distance_to_goal=2.5
                    self.wait=False
            if self.config['avoid_floor2']==False:
                self.wait=False
            while not rospy.is_shutdown() and self.distance_to_goal>1.5 and not self.wait:
                # fix elevator keep distance < 1.5 while entering and also<1.5 while leaving
                if self.config['avoid_floor2']==False:
                    self.wait=False
                if rospy.get_param('/in_elevator') and self.config['avoid_floor2']==False:
                    self.distance_to_goal=2.5
                    # BUG: FIX TOMORROW
                    self.next_area='E1-F2-COR-01'
                    self.current_area='E1-P2'
                    print(f"rospy.get_param('/Change_floor')={rospy.get_param('/Change_floor')}")
                    while not rospy.is_shutdown() and rospy.get_param('/Change_floor')==False:
                        time.sleep(0.5)
                        print(f"{utility.ORANGE}Waiting to change to another floor{utility.RESET}")
                    rospy.set_param('/in_elevator', False)

                    break
 
                # get next area according to area path
                if rospy.get_param('/Change_floor')==True and self.config['avoid_floor2']==False:
                    print(f"{utility.YELLOW}Change_floor is True{utility.RESET}")
                    index=4
                    self.next_area='E1d-F2-COR-01'
                    self.current_area='E1-F2-COR-01'
                    rospy.set_param('/Change_floor', False)
                    self.client2.send_goal(self.robot2_goal)

                    break

                if self.config['avoid_floor2']==True:
                    self.get_next_area()
                print(f"{utility.ORANGE}current_area:{self.current_area}, next_area:{self.next_area}, current_goal_passage:{self.current_goal_passage}, next_passage:{self.next_passage}{utility.RESET}")
                if utility.check_pose_inside_polygon(self.current_pose,self.cropped_areas[self.current_area]['nodes'],self.current_area,building_contour):
                    print(f"{utility.ORANGE}still in current area:{self.current_area}, next area is {self.next_area}{utility.RESET}")
                    pass
                elif self.next_area is not None and utility.check_pose_inside_polygon(self.current_pose,self.areas[self.next_area]['nodes'],self.next_area,building_contour):
                    # BUG: will always be in outside area
                    # successfully move to the next area, congratulations!!!!!!!!!!
                    print(f"{utility.ORANGE}successful to next area: {self.next_area}{utility.RESET}")
                    self.current_area=self.next_area
                    self.actual_areas_path.append(self.current_area)

                    self.get_next_area()

                    self.passed_passage=self.next_passage
                    save_experience.update_door_access(self.experience_file,self.passed_passage,'success')
                    self.actual_passages_path.append(self.passed_passage)

                    # if len(self.multi_passageid_path)>1:
                    # print(f"i:{i},multi_passageid_path:{self.multi_passageid_path},next_passage:{self.next_passage}")
                    # because this is already successful to next area, i start from 1 within 'elif'
                    if i<=len(self.multi_passageid_path)-1:
                        self.next_passage=self.multi_passageid_path[i]
                    else:
                        self.next_passage=None
                    print(f"{utility.ORANGE}passed_passage:{self.passed_passage}{utility.RESET}")
                    print(f"{utility.ORANGE}next_passage:{self.next_passage}{utility.RESET}")
                    # self.over_lay_text_manager.set_text(f"[osmAGPathPlanner]: Currently inside area:{self.current_area}, next area:{self.next_area}, distance to current goal:{self.distance_to_goal}, next passage:{self.next_passage}",1)
                    # self.over_lay_text_manager.publish_text(1)
                elif self.next_area is not None:
                    print(f"{utility.ORANGE} distance is bigger than 1, self.next_area is not None{utility.RESET}")
                else:
                    locate_robot_area=utility.locate_robot_area(self.current_pose,self.areas)
                    print(f"{utility.ORANGE}For whatever reason, robot not in current or next area{utility.RESET}, locate the robot in area {locate_robot_area} now")
                self.wait = self.client2.wait_for_result(rospy.Duration(0.5))

                self.distance_to_goal=self.calculate_distance(self.current_pose.pose.pose.position,self.robot2_goal.target_pose.pose.position)
                
                if (self.next_area=='E1-P2' or self.current_area=='E1-P2') and self.config['avoid_floor2']==False:
                    self.distance_to_goal=2.5
                
                if self.config['avoid_floor2']==False:
                    self.wait=False
                
                print(f"{utility.ORANGE}wtf,wait:{self.wait}, distance_to_goal:{self.distance_to_goal}{utility.RESET}")
        # CONSIDER SUCCESS IF THE LAST PASSAGE IS REACHED WITHIN 1M, no need to pass through the last passage yet, also don't save as success in experience, less than 1m considered as successful reach the final passage
        if self.next_passage == self.multi_passageid_path[-1] and self.distance_to_goal<1.5:
            print(f"{utility.ORANGE}Congratulations! All passages have been passed, in front of the goal area now (not entering)!{utility.RESET}")
            str_list = StringList()
            str_list.data =self.actual_areas_path
            self.pub_actual_areas_path.publish(str_list)
            str_list.data =self.actual_passages_path
            self.pub_actual_passages_path.publish(str_list)
            return True
            # break
        else:
            save_experience.update_door_access(self.experience_file,self.next_passage,'fail')
            print(f"{utility.ORANGE}WHOOPS! {self.next_passage} NOT ACCESSIBLE! CURRENT LOCATE IN {self.current_area}, GOAL AREA IS {self.shortest_area_path[-1]}{utility.RESET}")
            
            # data [current area, destination area, failed passage]
            self.pub_move_base_fail.publish([self.current_area,self.free_pass[-1],self.next_passage])
            return False
    
    def simulate_command_move_base(self,closed_passage,areas_path):


        if len(self.multi_passageid_path)>1:
            self.next_passage=self.multi_passageid_path[0]

        else: 
            self.next_passage=None
            print(f"{utility.ORANGE}ONLY ONE PASSAGE TO GO!{utility.RESET}")
        current_area=areas_path[0]
        for i,passage_id in enumerate(self.multi_passageid_path):
            print(f"{utility.ORANGE}i:{i},passage_id:{passage_id},multi_passageid_path:{self.multi_passageid_path}{utility.RESET}")
            self.current_goal_passage=passage_id
            if passage_id in closed_passage:
                print(f"{utility.ORANGE}[simple_command_move_base]simulate_command_move_base WHOOPS, {passage_id} IS NOT ACCESSIBLE{utility.RESET}")
                break
            else:
                print(f"{utility.ORANGE}[simulate_command_move_base] passage success: {passage_id}{utility.RESET}")
                self.passed_passage=passage_id
                print(self.passed_passage)
                self.next_passage=self.multi_passageid_path[(i+1)%len(self.multi_passageid_path)]
                print(self.next_passage)
                save_experience.update_door_access(self.experience_file,self.passed_passage,'success')
                self.actual_passages_path.append(passage_id)
                self.actual_areas_path.append(current_area)
                current_area=areas_path[(i+1)%len(areas_path)]

        if self.passed_passage == self.multi_passageid_path[-1]:
            print(f"{utility.ORANGE}Congratulations! All passages have been passed, in front of the goal area now (not entering)!{utility.RESET}")
            str_list = StringList()
            str_list.data =self.actual_areas_path
            self.pub_actual_areas_path.publish(str_list)
            str_list.data =self.actual_passages_path
            self.pub_actual_passages_path.publish(str_list)
            return True
            # break
        else:
            save_experience.update_door_access(self.experience_file,self.current_goal_passage,'fail')
            print(f"{utility.ORANGE}WHOOPS! {self.current_goal_passage} NOT ACCESSIBLE! CURRENT LOCATE IN {current_area}, GOAL AREA IS {self.shortest_area_path[-1]}{utility.RESET}")
            # data [current area, destination area, failed passage]
            self.pub_move_base_fail.publish([current_area,self.free_pass[-1],self.current_goal_passage])
            fakepose=Odometry()
            fakepose.pose.pose = Pose(Point(self.areas[current_area]['centroid'][0], self.areas[current_area]['centroid'][1], 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
            # Set velocity
            self.current_pose=fakepose
            self.current_area=current_area
            return False

    def PathPlanning(self):
        print(f"{utility.ORANGE}[simple_command_move_base]: PathPlanning start{utility.RESET}")
        if self.use_experience:
            with open(self.passage_experience_from_openai, 'r') as file:
                self.passage_experience_data = json.load(file)[-1]
        else:
            self.passage_experience_data =None
        # transfer area graph from utm to near zero 
        self.crop_areas()
        self.cropped_areas=map_generator.tanslate_osm(self.cropped_areas)

        # TESTING, use the area centroid as current location
        while self.current_pose is None and rospy.is_shutdown() is False:
            print(f"{utility.ORANGE}current_pose is None{utility.RESET}, wait for 2 second")
            time.sleep(2)

        current_pose=[self.current_pose.pose.pose.position.x,self.current_pose.pose.pose.position.y]
        # check if current pose is inside start area in free_pass before PathPlanning
        if not utility.check_pose_inside_polygon(self.current_pose,self.cropped_areas[self.free_pass[0]]['nodes']):
            print(f"{utility.ORANGE}[simple_command_move_base]:current pose is not in start area, CHECKING ALL AREAS NOW{utility.RESET}")
            print(f" self.current_pose{self.current_pose}, self.cropped_areas[self.free_pass[0]]['nodes']{self.cropped_areas[self.free_pass[0]]['nodes']}")
            locate_area_name=utility.locate_robot_area(self.current_pose,self.areas)
            print(f"{utility.ORANGE}[simple_command_move_base]: current pose is INSIDE {locate_area_name} now, change start area to {locate_area_name} now {utility.RESET}")
            self.free_pass[0]=locate_area_name
            self.crop_areas()
            self.cropped_areas=map_generator.tanslate_osm(self.cropped_areas)
        '''
        shortest_area_path: start area name+inter area
        shortest_path_normal: only have passage centroid direction 
        shortest_centroid_path: start +passage centroid+end area center
        multi_passageid_path:  only inter passage names
        '''
        self.shortest_area_path,self.shortest_path_normal,self.shortest_centroid_path,self.multi_passageid_path=map_drawer.load_plot_json_paths(input_tree=self.areas_tree,json_file_path=self.osm_path_prefix+self.pathlength_json,areas=self.areas,cropped_areas=self.cropped_areas,free_pass=self.free_pass,no_pass=self.no_path,areas_try_to_Avoid=self.areas_try_to_Avoid,example_free_pass=[],passage_experience=self.passage_experience_data,current_pose=current_pose,infeasible_passage=self.infeasible_passage)
        if self.shortest_area_path==None:
            print(f"{utility.RED}[simple_command_move_base]: A star failed, shortest_area_path:{self.shortest_area_path}{utility.RESET}")
            self.pub_astar_fail.publish("failed")

        # delete the first move_base goal, because it is the /odom used to find the shortest path,fix the wrong goal orientation bug, it is deleted in map_drawer shortest_path
        #TESTING
        # self.shortest_area_path=['E1d-F1-03', 'E1d-F1-COR-01', 'E1-F1-COR-01', 'E1-P2', 'E1-F2-COR-01', 'E1d-F2-COR-01']
        
        # self.multi_passageid_path=['E1d-F1-03_to_E1d-F1-COR-01_2', 'E1-F1-COR-01_to_E1d-F1-COR-01', 'E1-F1-COR-01_to_E1-P2', 'E1-F2-COR-01_to_E1-P2', 'E1-F2-COR-01_to_E1d-F2-COR-01', 'E1d-F2-03_to_E1d-F2-COR-01_2']
        # self.shortest_centroid_path=[(365730.0737664588, 3450357.440927563, 0), (365725.04500000004, 3450356.825, 0), (365704.29000000004, 3450356.85, 0), (365697.99, 3450370.685, 0), (365697.99, 3450370.685, 3), (365704.29000000004, 3450356.85, 3), (365725.04500000004, 3450356.825, 3), (173.01384615380084, 50.451538460794836, 3)]
        #END TESTING
        print(f"{utility.ORANGE}[simple_command_move_base]: free_pass:{self.free_pass}{utility.RESET}")
        print(f"{utility.ORANGE}[simple_command_move_base]: multi_passageid_path:{self.multi_passageid_path}, infeasible passages={self.infeasible_passage}{utility.RESET}")
        print(f"{utility.ORANGE}[simple_command_move_base]: shortest_area_path:{self.shortest_area_path}{utility.RESET}")
        return self.shortest_area_path,self.multi_passageid_path
    
    def reverse_goal_orientation(self):
        pass
    def calculate_distance(self, pos1, pos2):
        return math.sqrt((pos2.x - pos1.x)**2 + (pos2.y - pos1.y)**2)
    
    def gazebo_odom_listener(self, data):
        # self.current_pose=data   #################3testing

        if self.Bsimulate_command_move_base and  self.current_pose!=None:
            return
        self.current_pose=data
        # save current robot pose from gazebo /odom

        current_position = data.pose.pose.position
        if self.last_position is not None:
            dist = self.calculate_distance(current_position, self.last_position)
            self.total_travel_distance += dist
            print(f"TTTTTTTTTTTTTotal path length: {self.total_distance:.2f} meters")
        self.last_position = current_position

    #  TODO: FIND OUT HOW TO DEFINE WHOLE SUCCESS, WHOLE FAIL, PASSAGE SUCCESS, PASSAGE FAIL CONDITIONS
    def move_base_result_callback(self, data):
        # uint8 PENDING=0
        # uint8 ACTIVE=1
        # uint8 PREEMPTED=2   interrupted by another goal
        # uint8 SUCCEEDED=3
        # uint8 ABORTED=4
        # uint8 REJECTED=5
        # uint8 PREEMPTING=6
        # uint8 RECALLING=7
        # uint8 RECALLED=8
        # uint8 LOST=9
        self.move_base_result_status=data.status.status
        print(f"{utility.ORANGE}receiving move_base_result_status:{self.move_base_result_status}{utility.RESET}")
        if self.move_base_result_status==2 or self.move_base_result_status==3:
            pass
        elif self.move_base_result_status==4:
            self.PLANNER_FAILED=True
            print(f"{utility.ORANGE}receiving move_base_result_status:{self.move_base_result_status}, WHOOPS from move_base_result, {self.next_passage} IS NOT ACCESSIBLE{utility.RESET}")
            # save_experience.update_door_access(self.experience_file,self.next_passage,'fail')
    
    def run(self):
        # only for test
        # TODO:
        self.crop_areas()
        
        self.PathPlanning()
        # self.command_move_base()
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            rospy.spin()
            rate.sleep()

    def extract_passage_weight_from_json(self, json_data):
        # Extract the passage weights from the JSON experience data
        passage_weights = {}
        for passage_name, passage_data in json_data['passages'].items():
            passage_weights[passage_name] = passage_data['cost']
        return passage_weights

if __name__ == '__main__':
    try:
        # robot_nav = RobotNavigation(free_pass=['E1d-F1-13', 'E1b-F1-07'])
        # robot_nav = RobotNavigation(free_pass=['E1d-F1-13', 'E1d-F1-06'])# pick right passage   osm_with_collision.world
        # floor2_passages=config['elevator_floor2_passage']+config['stairs_floor2_passage']
        robot_nav = RobotNavigation(free_pass=['E1c-F1-COR-01', 'E1d-F1-08'],no_path=['E1d-F1-COR-01'])# lobby close
        robot_nav.run()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

















