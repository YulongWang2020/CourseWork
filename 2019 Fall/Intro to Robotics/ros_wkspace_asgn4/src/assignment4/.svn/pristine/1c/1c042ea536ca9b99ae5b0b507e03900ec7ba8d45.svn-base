#!/usr/bin/env python

import numpy
import random
import sys

import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class MoveArm(object):

    def __init__(self):

        #Loads the robot model, which contains the robot's kinematics information
	self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[0]

	#Subscribe to topics
	rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
	rospy.Subscriber('/motion_planning_goal', Transform, self.motion_planning)
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

	#Set up publisher
	self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

    '''This callback provides you with the current joint positions of the robot 
     in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    '''This is the callback which will implement your RRT motion planning.
    You are welcome to add other functions to this class (i.e. an
    "is_segment_valid" function will likely come in handy multiple times 
    in the motion planning process and it will be easiest to make this a 
    seperate function and then call it from motion planning). You may also
    create trajectory shortcut and trajectory sample functions if you wish, 
    which will also be called from the motion planning function.'''    
    def motion_planning(self, ee_goal):
        print "Starting motion planning"
	########INSERT YOUR RRT MOTION PLANNING HERE##########
        #print(ee_goal)
        ee_goal_T_matrix = tf.transformations.translation_matrix((ee_goal.translation.x,
            ee_goal.translation.y,ee_goal.translation.z))
        ee_goal_R_matrix = tf.transformations.quaternion_matrix([ee_goal.rotation.x,
            ee_goal.rotation.y,ee_goal.rotation.z,ee_goal.rotation.w])
        ee_goal_matrix = ee_goal_T_matrix.dot(ee_goal_R_matrix)
        q_des = self.IK(ee_goal_matrix)
        q_des = list(q_des)
        for i in range(len(q_des)):
            if q_des[i] > 3.1415:
                q_des[i] = q_des[i] - 3.1415*2 
            if q_des[i] < -3.1415:
                q_des[i] = q_des[i] + 3.1415*2

        q_des = tuple(q_des)
        q_start = numpy.array(self.q_current)
        q_end = numpy.array(q_des)
        q_start = RRTBranch(None,q_start)
        q_end = RRTBranch(None,q_end)
        #print(q_end.q,"q_end-----------------------------")
        if self.is_segment_valid(q_start.q,q_end.q):
            chain = numpy.array([q_start,q_end])
            q_points = self.final_sample(chain)
            msg = self.convert_to_msg(q_points)
            #print(msg)
            self.pub.publish(msg)
        else:   
            #creat object
            q_list = [q_start]
            #print(q_list,"loop start")
            #print(q_list[0].q,"asdfsadfsadfsadfsaf")
            while True:
                #initial new object
                new_random_point = RRTBranch(None,numpy.random.uniform(-3.1415,3.1415,self.num_joints))
                neighbor = self.find_nearest_neighbor(q_list,new_random_point)
                #print(self.is_segment_valid(new_random_point.q,neighbor.q))
                if self.is_segment_valid(new_random_point.q,neighbor.q):
                    q_list.append(new_random_point);
                    new_random_point.parent = neighbor;
                    #print(self.is_segment_valid(new_random_point.q,q_end.q))
                    if self.is_segment_valid(new_random_point.q,q_end.q):
                        break
            #print(len(q_list))
            #print(q_list[0].q-q_start.q,"q_lisy")
            chain = [];
            chain.append(q_end);
            curr = q_list[-1]
            #print("start finding chain")
            while curr != None:
                chain.append(curr)
                curr = curr.parent
                #print(curr)
            #print(chain[-1].q-q_start.q,chain[0].q-q_end.q,"before shortcut")

            #print(q_list)
            #print(chain,"this is the chain--------------------------------------")
            chain = self.shortcut(chain)
            q_points = self.final_sample(chain)

            q_points = numpy.flipud(q_points)

            #print(q_points[0]-q_start.q,q_points[-1]-q_end.q,"start point & end points check")
            msg = self.convert_to_msg(q_points)
            #check if the chain is from start to final
            #print(msg)
            self.pub.publish(msg)
            
            

        ######################################################
    def convert_to_msg(self,q_list):
        #receive a set of q_values and convert it to msg
        waypoints = JointTrajectory()
        waypoints.joint_names = self.joint_names
        waypoints.points = []
        #print(q_list,"----------------------------------------")
        for each in q_list:
            points = JointTrajectoryPoint()
            points.positions = each
            waypoints.points.append(points)
        #print(waypoints)
        return waypoints;
    def final_sample(self,chain):
        #receive a set of object chains,sample it
        #print(chain.shape)
        #print(len(chain),"----------------------------------")
        new_chain = numpy.array([])
        length = 0;
        for i in range(len(chain)-1):
            length = numpy.linalg.norm(chain[i].q-chain[i+1].q) + length;
        sample_rate = int(length/0.5)
        #print(len(chain))
        for i in range(len(chain)-1):
            #print(self.sample(chain[i].q,chain[i+1].q,sample_rate).shape,"sampleddddd-----------")
            new_chain = numpy.append(new_chain,self.sample(chain[i].q,chain[i+1].q,sample_rate)[:])
            #print(new_chain.shape)
        new_chain = new_chain.reshape((len(chain)-1)*(sample_rate+1),self.num_joints)
        #print(new_chain.shape)
        #new_chain = numpy.squeeze(new_chain,0)
        #print(new_chain.shape)
        #print(len(new_chain),"123213213213213")

        return new_chain;

    def find_nearest_neighbor(self,q_list,q_new_point):
        distance = numpy.zeros(len(q_list))
        for i in range(len(q_list)):
            #print(q_list[i].q,q_new_point.q)
            distance[i] = numpy.linalg.norm(q_list[i].q-q_new_point.q);
        return q_list[numpy.argmin(distance)]

    def is_segment_valid(self,q_start,q2_end):
        #check if this branch is vaild
        #sample the line
        sample_rate = int(numpy.linalg.norm(q_start-q2_end)/0.1);
        q_sampled = self.sample(q_start,q2_end,sample_rate);
        check  = True;
        for each in q_sampled:
            if not self.is_state_valid(each):
                check = False;
        return check;

    def shortcut(self,chain):
        #start from the q_end
        prev = chain[0];
        curr = chain[1];
        new_chain = [chain[0]];
        while curr != None:
            if self.is_segment_valid(curr.q,prev.q):
                prev.parent = curr;
                curr = curr.parent;
                continue
            new_chain.append(prev.parent)
            prev = prev.parent

        new_chain.append(prev.parent);
        return new_chain
    def sample(self,q_start,q2_end,sample_rate):
        #return a list of q value sampled between to q points
        number_of_sample = sample_rate;
        joint_nums = q_start.shape;
        q_difference = (q2_end - q_start)/sample_rate;
        q_sample = numpy.zeros((number_of_sample+1,joint_nums[0]))
        for i in range(number_of_sample+1):
            q_sample[i] = q_start + q_difference * i;
        return q_sample

    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the 
    joints of the robot arm, ordered from proximal to distal. If no IK solution 
    is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link


    """ This function checks if a set of joint angles q[] creates a valid state,
    or one that is free of collisions. The values in q[] are assumed to be values
    for the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid


'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list 
(your 'tree'). The class has a parent field and a joint position field (q). 
You can initialize a new branch like this:
RRTBranch(parent, q)
Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent, q):
	self.parent = parent
	self.q = q

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

