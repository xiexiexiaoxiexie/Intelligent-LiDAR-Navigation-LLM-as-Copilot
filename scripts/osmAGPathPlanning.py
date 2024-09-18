#!/home/xiefujing/anaconda3/envs/openai/bin/python3
import json
import os
import xml.etree.ElementTree as ET
import logging
import time
import utility 
import rospy
from std_msgs.msg import String
import simple_command_move_base
from osmAG_intelligent_navigation.msg import StringList
from overlay_text import OverlayTextManager
from std_msgs.msg import ColorRGBA

logging.disable(logging.CRITICAL)
os.chdir('/home/xiefujing/catkin_ws')

with open('./src/osmAG_intelligent_navigation/scripts/config.json', 'r') as config_file:
    config = json.load(config_file)

class osmAGPathPlanning:
    def __init__(self):
        print(f"{utility.BLUE} [osmAGPathPlanning]: osmAGPathPlanning obj is initilizing{utility.RESET}")
        rospy.init_node('osmAGPathPlanning', anonymous=True)
        while not rospy.has_param('/case_num') and not rospy.is_shutdown():
            print(f"{utility.BLUE}[osmAGPathPlanning]: waiting for parameter to be set{utility.RESET}")
            time.sleep(1)
        self.case_num = rospy.get_param('/case_num', 'case1')
        self.case_text=rospy.get_param('/case_text','')
        self.Bcommand_move_base=rospy.get_param('/Bcommand_move_base','fasle')
        self.closed_passage=rospy.get_param('/closed_passage',[])
        self.closed_area=rospy.get_param('/closed_area','')
        print(f"{utility.BLUE} [osmAGPathPlanning]: case_num is {self.case_num}{utility.RESET}")

        self.osm_file=config['osm_path_prefix']+config['osm_file']
        self.areas_path_pub = rospy.Publisher('areas_path', StringList, queue_size=10)
        self.robot_nav=None
        self.NavigationEventMonitor_feedback=None
        self.infeasible_passage=[]
        self.robot_path_planning_result=None # the result of current path planning area path
        self.passage_id_path=None # the result of current path planning passage id path
        self.is_Valid=False # the areas path approve result from NavigationEventMonitor
        self.replan_time=0
        self.mission_result=False #command move_base success or not
        self.BAstar_fail=False
        # receive the initial free pass from PassageCostEvaluator, start and destination area
        rospy.Subscriber("free_pass", StringList, self.free_pass_callback)
        # receive the NavigationEventMonitor feedback, decide to replan or not
        rospy.Subscriber("NavigationEventMonitor_response_json_str", String, self.NavigationEventMonitor_callback)
        # move_base failed, some passages are not accessible, get current area, destination area, and the failed passage, don't use the failed passage again, and replan from simple_command_move_base.
        rospy.Subscriber("move_base_fail", StringList, self.move_base_fail_callback)
        rospy.Subscriber("Astar_fail", StringList, self.Astar_fail_callback)
        self.pub_navigation_result=rospy.Publisher('navigation_result', String, queue_size=10)
        self.pub_event_monitor_fail=rospy.Publisher('event_monitor_fail', String, queue_size=10)
        # publish infeasible area to NavigationEventMonitor in case of avoid fail
        self.pub_avoid_fail=rospy.Publisher('avoid_fail', String, queue_size=10) 
        self.re_monitor_time=0
        self.over_lay_text_manager=OverlayTextManager()
        self.overall_text = self.case_text
        self.over_lay_text_manager.add_text_box('/overlay_text_overall_msg', self.overall_text, 600, 370, 10, 80, 9, 2, ColorRGBA(1.0, 1.0, 1.0, 1.0), ColorRGBA(0.0, 0.0, 0.0, 0.7))
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            rospy.spin()
            rate.sleep()

    def Astar_fail_callback(self,Astar_fail_msg):
        self.BAstar_fail=True

    def __del__(self):
            print('destroying osmAGPathPlanning object')

    # For whatever reason, move_base to the current passage failed, don't use the failed passage again, and replan and send the planned path to NavigationEventMonitor again
    # data [current area, destination area, failed passage]
    def move_base_fail_callback(self,move_base_fail_msg):
        print(f"{utility.BLUE} [osmAGPathPlanning]: Move base failed, currently at {move_base_fail_msg.data[0]}, goal is {move_base_fail_msg.data[1]}, not accessible passages are {move_base_fail_msg.data[2]}, now replan to: {move_base_fail_msg.data[1]}{utility.RESET}")
        # replan and send to NavigationEventMonitor again
        self.infeasible_passage.append(move_base_fail_msg.data[2])
        self.robot_nav.infeasible_passage=self.infeasible_passage
        self.robot_nav.free_pass=[move_base_fail_msg.data[0],move_base_fail_msg.data[1]]
        self.robot_nav.area_index=0
        self.robot_path_planning_result,self.passage_id_path=self.robot_nav.PathPlanning()
        # publishing the areas path to NavigationEventMonitor to check areas
        self.publish_areas_path()

    def publish_areas_path(self):
        str_list = StringList()
        str_list.data =self.robot_path_planning_result
        print(f"{utility.BLUE} [osmAGPathPlanning]: osmAGPathPlanning is done, now publish areas_path to NavigationEventMonitor{utility.RESET}")
        self.areas_path_pub.publish(str_list)

    # the function get start and destination area, plan a path and publish 
    def free_pass_callback(self,free_pass_msg):
        print(f"{utility.BLUE} [osmAGPathPlanning]: Rececived free pass from PassageCostEvaluator, now plan a area path in osmAGPathPlanning {free_pass_msg.data}{utility.RESET}")
        overall_text=""
        self.robot_nav=simple_command_move_base.RobotNavigation(free_pass=free_pass_msg.data,no_path=[],case_num=self.case_num,over_lay_text_manager=self.over_lay_text_manager)
        self.robot_nav.free_pass=free_pass_msg.data
        # place robot in start area in gazebo
        self.robot_nav.place_robot_in_start_area_gazebo(free_pass_msg.data[0])
        self.robot_nav.case_num=self.case_num
        self.robot_nav.actual_areas_path=[]
        self.robot_nav.actual_passages_path=[]
        self.robot_nav.area_index=0
        self.infeasible_passage=[]
        self.re_monitor_time=0
        self.robot_path_planning_result,self.passage_id_path=self.robot_nav.PathPlanning()
        # publishing the areas path to NavigationEventMonitor to check areas
        self.publish_areas_path()

    # check if avoid all areas in areas_to_avoid, there is a feaible path or not
    def check_avoid_feasible(self,areas_to_Avoid):
        temp_robot_nav=simple_command_move_base.RobotNavigation(free_pass=self.robot_nav.free_pass,no_path=areas_to_Avoid,case_num=self.case_num,over_lay_text_manager=self.over_lay_text_manager)
        temp_robot_path_planning_result,temp_passage_id_path=temp_robot_nav.PathPlanning()
        print(f"{utility.BLUE}[osmAGPathPlanning]: check_avoid_feasible, areas_to_Avoid: {areas_to_Avoid}, temp_robot_path_planning_result: {temp_robot_path_planning_result}{utility.RESET}")
        if temp_robot_path_planning_result==None:
            print(f"{utility.BLUE}[osmAGPathPlanning]: check_avoid_feasible, Failed, NO PATH available{utility.RESET}")
            self.pub_avoid_fail.publish(f"{str(areas_to_Avoid)}")
            return False
        else:
            return True

    def not_in(self,set_,list_):
        for ele in list_:
            if ele in set_:
                return False
        return True
    
    # getting feedback from NavigationEventMonitor, if is_Valid, command move_base, if not, replan and send to NavigationEventMonitor again
    def NavigationEventMonitor_callback(self,NavigationEventMonitor_msg):
        print(f"{utility.BLUE}[osmAGPathPlanning]: Receciving NavigationEventMonitor feedback now{utility.RESET}")
        try:
            self.NavigationEventMonitor_feedback = json.loads(NavigationEventMonitor_msg.data)
        except Exception as e:
            print(f"{utility.BLUE}[osmAGPathPlanning]:!!!!!!!!!!!!!!!!!!!!Error decoding JSON{utility.RESET}")
            print(e)

        self.is_Valid=self.NavigationEventMonitor_feedback['is_Valid']
        self.areas_to_Avoid=self.NavigationEventMonitor_feedback['areas_to_Avoid']
        self.areas_try_to_Avoid=self.NavigationEventMonitor_feedback['areas_try_to_Avoid']
        # for visualization
        temp_text=f"[External Notification]:\n"+self.case_text
        self.over_lay_text_manager.set_text(temp_text,0)
        self.over_lay_text_manager.publish_text(0)
        print(f"{utility.BLUE}[osmAGPathPlanning]: NavigationEventMonitor result is is_Valid={self.is_Valid}, areas_to_Avoid: {self.areas_to_Avoid}, areas_try_to_avoid={self.areas_try_to_Avoid}{utility.RESET}")
        # check if the areas_try_to_Avoid is in the path
        areas_try_to_Avoid_in_path=self.area_in_to_go(self.areas_try_to_Avoid)
        print(f"{utility.BLUE}[osmAGPathPlanning]: areas_try_to_Avoid_in_path: {areas_try_to_Avoid_in_path}{utility.RESET}")
        # Bevent_feasible=self.check_avoid_feasible(self.areas_to_Avoid)
        Bevent_feasible=True
        self.over_lay_text_manager.publish_text(1)

        if Bevent_feasible==False:
            print(f"{utility.BLUE}[osmAGPathPlanning]: event is not feasible, areas_to_Avoid: {self.areas_to_Avoid}, areas_try_to_Avoid: {self.areas_try_to_Avoid}, now republish to NavigationEventMonitor{utility.RESET}")
            self.re_monitor_time+=1
            # self.publish_areas_path()
            return
        
        for area in self.areas_to_Avoid:
            if self.re_monitor_time>3:
                string_="resend to event_monitor more than 3 times, abort now"
                self.pub_event_monitor_fail.publish(string_)
                return
            # used in case testing, check if the area in areas_to_Avoid is really the closed area according to external info
            if area not in self.closed_area:
                string_=f"closed area is {self.closed_area}, event monitor's answer is {area}"
                self.pub_event_monitor_fail.publish(string_)
                self.re_monitor_time+=1
                self.publish_areas_path()
                print(f"{utility.BLUE}[osmAGPathPlanning]: {string_}, republish to NavigationEventMonitor{utility.RESET}")
                return
        # approvalled by NavigationEventMonitor, now command move_base
        if self.is_Valid==True :
            # if no area in try_to_Avoid or already tried to avoid before, simply command move_base
            if len(areas_try_to_Avoid_in_path)==0 or self.replan_time>0:
                print(f"{utility.BLUE}[osmAGPathPlanning]: Approved by NavigationEventMonitor, now commanding move_base{utility.RESET}")
                self.replan_time=0
                if self.Bcommand_move_base==True:
                    self.robot_nav.PLANNER_FAILED=False
                    self.robot_nav.wait=False
                    self.mission_result=self.robot_nav.command_move_base()
                else:
                    self.robot_nav.Bsimulate_command_move_base=True
                    self.robot_nav.area_index=0
                    self.mission_result=self.robot_nav.simulate_command_move_base(self.closed_passage,self.robot_path_planning_result)

                self.pub_navigation_result.publish(str(self.mission_result))
                print(f"{utility.BLUE} [osmAGPathPlanning]: navigation result is {self.mission_result}, publishing now{utility.RESET}")
                if self.replan_time>1:
                    print(f"{utility.BLUE}[osmAGPathPlanning]: already tried two times to avoid, ignore try to avoid suggestions, now robot avoid= {self.robot_nav.no_path}, trys to avoid= {self.robot_nav.areas_try_to_Avoid}{utility.RESET}")
            # try to avoid the areas in areas_try_to_Avoid by adding more cost to that area
            else:
                print(f"{utility.BLUE} [osmAGPathPlanning]: NavigationEventMonitor recommend avoid {areas_try_to_Avoid_in_path}, it is in the to-go list, now robot avoid= {self.robot_nav.no_path}, try to avoid= {self.robot_nav.areas_try_to_Avoid}, try to replan now{utility.RESET}")
                for item in areas_try_to_Avoid_in_path:
                    if item not in self.robot_nav.areas_try_to_Avoid:
                        self.robot_nav.areas_try_to_Avoid.append(item)
                self.robot_path_planning_result,self.passage_id_path=self.robot_nav.PathPlanning()
                self.publish_areas_path()
                self.replan_time+=1
        # not approved by NavigationEventMonitor, replan and send to NavigationEventMonitor again
        else:
            # replan, put areas_to_Avoid to no_path, send to NavigationEventMonitor again
            if len(self.area_in_to_go(self.areas_to_Avoid))>0:
                for item in self.areas_to_Avoid:
                    if item not in self.robot_nav.no_path:
                        self.robot_nav.no_path.append(item)
                for item in areas_try_to_Avoid_in_path:
                    if item not in self.robot_nav.areas_try_to_Avoid:
                        self.robot_nav.areas_try_to_Avoid.append(item)
                print(f"{utility.BLUE} [osmAGPathPlanning]: NavigationEventMonitor request avoid {self.robot_nav.no_path}, replan now{utility.RESET}")
                self.robot_path_planning_result,self.passage_id_path=self.robot_nav.PathPlanning()
                if self.robot_path_planning_result == None:
                    print(f"{utility.BLUE} [osmAGPathPlanning]: A star failed to find a path, since destination is guarateed to be accessible by world, delete no_pass, replan and resent to NavigationEventMonitor request {utility.RESET}")
                    self.robot_nav.no_path=[]
                    self.robot_path_planning_result,self.passage_id_path=self.robot_nav.PathPlanning()
                    self.publish_areas_path()
                else:
                    # The replaned path will also be assessd by NavigationEventMonitor
                    self.publish_areas_path()
            # areas_to_Avoid not in the path, ignore the request,stick to the current plan
            else:
                print(f"{utility.BLUE} [osmAGPathPlanning]: NavigationEventMonitor request avoid {self.areas_to_Avoid}, but it is not in the to-go list, ignore, current area is {self.robot_nav.current_area}, self.robot_nav.shortest_area_path:{self.robot_nav.shortest_area_path}, keep going{utility.RESET}")
                if self.robot_nav.current_area == self.robot_nav.shortest_area_path[0]:
                    if self.Bcommand_move_base==True:
                        self.robot_nav.PLANNER_FAILED=False
                        self.robot_nav.wait=False
                        self.robot_nav.area_index=0
                        self.mission_result=self.robot_nav.command_move_base()
                    # only used in debugging
                    else:
                        self.robot_nav.Bsimulate_command_move_base=True
                        self.robot_nav.area_index=0
                        self.mission_result=self.robot_nav.simulate_command_move_base(self.closed_passage,self.robot_path_planning_result)
                    self.pub_navigation_result.publish(str(self.mission_result))
                    print(f"{utility.BLUE} [osmAGPathPlanning]: navigation result is {self.mission_result}, publishing now{utility.RESET}")
                else:
                    pass

    # check if the areas_to_check is in the path, if yes, return the areas_to_check in the path
    def area_in_to_go(self,areas_to_check):
        # Find the index of the current element to determine the split point
        try:
            current_index = self.robot_path_planning_result.index(self.robot_nav.current_area) + 1
        except ValueError:
            print(f"{utility.BLUE}[osmAGPathPlanning]: current_index{current_index}, self.robot_path_planning_result: {self.robot_path_planning_result}, self.robot_nav.current_area: {self.robot_nav.current_area} Current element not in list{utility.RESET}")
            return False
        # Split the list based on the current index
        first_half = self.robot_path_planning_result[:current_index]
        second_half = self.robot_path_planning_result[current_index:]
        # Check in which half the received string lies
        in_elements=[element for element in areas_to_check if element in second_half]
        if all(element in second_half for element in areas_to_check):
            return in_elements
        else:
            print(f"{utility.BLUE}[osmAGPathPlanning]:current_index{current_index}, self.robot_path_planning_result: {self.robot_path_planning_result}, self.robot_nav.current_area: {self.robot_nav.current_area} areas_to_check: {areas_to_check}, second_half: {second_half} {utility.RESET}")
            return in_elements

def run():
    osmAGPathPlanning_obj = osmAGPathPlanning()

if __name__ == "__main__":
    run()


