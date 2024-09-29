#!/home/xiefujing/anaconda3/envs/openai/bin/python3
# robot1 is the one only using ROS move_base
import json
import os
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
import actionlib
from move_base_msgs.msg import MoveBaseActionResult
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Odometry
import sys
import time
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
os.chdir('/home/xiefujing/catkin_ws')
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
parent_parent_dir = os.path.dirname(parent_dir)

sys.path.append(parent_parent_dir)
import utility

def process_json(input_files):
    data = []
    # Read each file and extend the combined_data list
    for file_path in input_files:
        with open(file_path, 'r') as file:
            this_data = json.load(file)
            data.extend(this_data)
    results = []
    for entry in data:
        if '/robot2/odom' in entry['topics']:
            odom_data = entry['topics']['/robot2/odom']
            if odom_data:
                first_odom = odom_data[0]
                last_odom = odom_data[-1]

                result = {
                    "case_num": entry['parameters']['case_num'],
                    "trial_free_pass": entry['parameters'].get('trial_free_pass', []),
                    "first_odom": first_odom,
                    "last_odom": last_odom
                }
                results.append(result)
    return results
def create_move_base_action_goal( x, y, z, qx, qy, qz, qw):
    move_base_goal = MoveBaseGoal()
    move_base_goal.target_pose.header.frame_id = "map"
    move_base_goal.target_pose.header.stamp = rospy.Time.now()
    move_base_goal.target_pose.pose.position = Point(x, y, z)
    move_base_goal.target_pose.pose.orientation = Quaternion(qx, qy, qz, qw)
    return move_base_goal

class Robot1_movebase:
    def __init__(self,odom_data_list):
        
        rospy.init_node('command_robot')
        self.client1 = actionlib.SimpleActionClient('robot1/move_base', MoveBaseAction)
        self.robot1_file_path = './src/osmAG_intelligent_navigation/scripts/cases/case1/robot1_archive_1_noclear.json'
        # stop move_base from receiveing scan while placing the robot, otherwise error in global map from scan in old place.
        self.sub=rospy.Subscriber('/robot1/scan', LaserScan, self.laser_callback)
        self.republisher = rospy.Publisher('/robot1/scan_', LaserScan, queue_size=10)
        self.Brepublisher=True
        self.robot_goal = None
        self.move_base_result_status=None
        self.odom_data_list=odom_data_list
        self.trials=[]
        self.current_trial={}
        self.odom_received_num=0
        self.odom_interval=50
        self.wait=False
        self.receive_odom=False
        self.current_trial={
            "case_num": '',
            "trial_free_pass": [],
            "/robot1/odom": []
            }
        rospy.Subscriber("/robot1/odom", Odometry, self.gazebo_odom_listener)
        rospy.Subscriber("/robot1/move_base/result", MoveBaseActionResult, self.move_base_result_callback)
    def laser_callback(self,scan_msg):
        if self.Brepublisher:
            self.republisher.publish(scan_msg)
        else:
            pass
    def move_base_result_callback(self,data):
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
            print(f"{utility.ORANGE}MOVE_BASE SUCCESS{utility.RESET}")
        elif self.move_base_result_status==4:
            print(f"{utility.ORANGE}WHOOPS MOVE_BASE FAILED{utility.RESET}")
            # save_experience.update_door_access(self.experience_file,self.next_passage,'fail')
        # return move_base_result_status

    def gazebo_odom_listener(self,data):
        self.receive_odom=True
        self.odom_received_num+=1
        if self.odom_received_num%self.odom_interval==0:
            self.current_trial['/robot1/odom'].append({
                'timestamp': rospy.Time.now().to_sec(),
                'pose': {
                    'position': {
                        'x': data.pose.pose.position.x,
                        'y': data.pose.pose.position.y,
                        'z': data.pose.pose.position.z
                    }
                }
                
            })

    def command_movebase(self):
        start_index=0
        print(f"{utility.ORANGE}commanding movebase{utility.RESET}")
        print(self.odom_data_list)
        for i,odom_data in enumerate(self.odom_data_list):
            if i<start_index:
                continue
            
            # self.clear_costmaps()
            self.current_trial={
                "case_num": odom_data['case_num'],
                "trial_free_pass": odom_data['trial_free_pass'],
                "/robot1/odom": []
                }
            print(f"{utility.ORANGE}current_trial:{self.current_trial}{utility.RESET}")
            self.move_base_result_status=-1
            self.odom_received_num=0
            self.wait=False
            print(f"{utility.ORANGE}Placing robot in start area{utility.RESET}")
            self.place_robot_in_start_area_gazebo(odom_data['first_odom']['pose']['position'])
            time.sleep(5)
            self.Brepublisher=True
            robot1_goal=create_move_base_action_goal(odom_data['last_odom']['pose']['position']['x'],odom_data['last_odom']['pose']['position']['y'],0,0,0,0,1)
            print(f"{utility.ORANGE}Sending move_base goal{utility.RESET}")
            time.sleep(5)#wait for move-base to be ready
            self.client1.send_goal(robot1_goal)
            while not rospy.is_shutdown() and not self.wait and self.move_base_result_status==-1:
                self.wait = self.client1.wait_for_result(rospy.Duration(1))
                print(f"{utility.ORANGE}waiting for move_base result{utility.RESET}")
            self.trials.append(self.current_trial)
            self.save_to_file()
            self.Brepublisher=False
        self.save_to_file()
        
    def save_to_file(self):
        with open(self.robot1_file_path, 'w') as f:
            json.dump(self.trials, f, indent=4)
        print(f"Archive json saved to {self.robot1_file_path}")   

    def clear_costmaps(self):
        rospy.wait_for_service('/robot1/move_base/clear_costmaps')
        try:
            # Create a service proxy for the obstacle layer clear service
            clear_obstacle_service = rospy.ServiceProxy('/robot1/move_base/clear_costmaps', Empty)
            # Call the service to clear the obstacle layer
            clear_obstacle_service()
            print(f"{utility.ORANGE}Costmaps cleared{utility.RESET}")
        except rospy.ServiceException as e:
            print(f"{utility.ORANGE}Costmaps cleared FAILED{utility.RESET}")
            rospy.wait_for_service('/robot1/move_base/clear_costmaps')
        


    def place_robot_in_start_area_gazebo(self,position):
        rospy.wait_for_service('/gazebo/set_model_state')  # Wait for the service to be available
        try:
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)  # Create a service proxy
            # Define the model state
            model_state = ModelState()
            model_state.model_name = 'robot1'
            model_state.pose = Pose()
            model_state.pose.position.x = position['x']
            model_state.pose.position.y = position['y']
            model_state.pose.position.z = 0
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
            model_state.reference_frame = ''  # Empty string means world frame
            # Call the service
            response = set_state(model_state)
            print(response)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


if __name__ == "__main__":
    robot2_file_path2 = './src/osmAG_intelligent_navigation/scripts/cases/case1/archive1-13.json'
    file_list=[robot2_file_path2]
    odom_data_list = process_json(file_list)
    robot1_movebase=Robot1_movebase(odom_data_list)

    while not rospy.is_shutdown() and robot1_movebase.receive_odom==False:
        time.sleep(1)
        print("waiting for gazebo odom data")
    robot1_movebase.command_movebase()


