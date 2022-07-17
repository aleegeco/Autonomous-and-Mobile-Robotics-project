#!/usr/bin/env python
# license removed for brevity

import rospy
# Brings in the SimpleActionClient
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import os

goals_file = 'goals.txt'
def movebase_client(x,y):

   # Create an action client called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
   # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()

   # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y

   # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1

   # Sends the goal to the action server.
    client.send_goal(goal)
   # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
   # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':
    try:
       # Initializes a rospy node to let the SimpleActionClient publish and subscribe
        rospy.init_node('movebase_client_py')
        dir_path = os.path.dirname(os.path.realpath(goals_file))
        text = open(dir_path + '/' + goals_file, 'r')
        next(text)
        next(text)
        goal = {'x':0,'y':0}

        while True:
            point = text.readline().split()
            if not point:
                break
            goal['x'] = float(point[0])
            goal['y'] = float(point[1])
            rospy.loginfo("Goal x:{}  y:{}".format(goal['x'],goal['y']))
            result = movebase_client(goal['x'],goal['y'])
            if result:
                rospy.loginfo("Goal execution done!")
                rospy.sleep(5)
        text.close()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

