#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import subprocess

class MapLaunchTrigger:
    def __init__(self):
        rospy.init_node('map_server_manager', anonymous=True)
        rospy.Subscriber('change_map', String, self.trigger_launch)

    def trigger_launch(self, data):
        print(f"Received trigger with data: {data.data}")
        launch_command = "roslaunch osmAG_intelligent_navigation map_server.launch"
        try:
            print("Launching map server...")
            subprocess.Popen(launch_command.split())
            print("Map server launched.")
        except Exception as e:
            print(f"Failed to launch map server: {str(e)}")

if __name__ == '__main__':
    map_launcher = MapLaunchTrigger()
    rospy.spin()
