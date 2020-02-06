#!/usr/bin/env python3


import rospy
import csv
import os.path
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped, BoolStamped, FSMState, StopLineReading
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
from collections import namedtuple
import threading

scripts = { # actions,list of (v,omega,duration)
    "path":[(0,0,4), (0.8,0,4.2), (0,0,2), (0,4,1.7), (0,0,2), (0.8,0,2), (0,0,2),
     (0,-4,1.7), (0,0,2), (1,0,1), (0,0,4)],
    "360_slow":[(0,0,4),(0,4,7), (0,0,4)],
    "360_fast":[(0,0,4),(0,8,3.5), (0,0,4)],
    "circle":[(0,0,4),(0.5,8,3.5), (0,0,4)]
}

current_script = "path"

class static_controller(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.op_start_time = None
        self.last_known_pose = None
        self.runs = []

        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_env_reset = rospy.Publisher("~reset_env", Bool, queue_size=1)

        self.sub_wheels_cmd_executed = rospy.Subscriber("~wheels_cmd_executed", WheelsCmdStamped, self.updateWheelsCmdExecuted, queue_size=1)

        self.sub_actuator_limits = rospy.Subscriber("~actuator_limits", Twist2DStamped, self.updateActuatorLimits, queue_size=1)
        self.sub_pose = rospy.Subscriber("~pose2d", Pose2D, self.update_pose, queue_size=1)

        rospy.on_shutdown(self.safe_shutdown)
        # Create a thread for a fixed-rate loop that publishes controls
        worker = threading.Thread(target=self.read_script)
        worker.start()
       

    def updateWheelsCmdExecuted(self, msg_wheels_cmd):
        self.wheels_cmd_executed = msg_wheels_cmd

    def updateActuatorLimits(self, msg_actuator_limits):
        self.actuator_limits = msg_actuator_limits
        rospy.logdebug("actuator limits updated to: ")
        rospy.logdebug("actuator_limits.v: " + str(self.actuator_limits.v))
        rospy.logdebug("actuator_limits.omega: " + str(self.actuator_limits.omega))

    def update_pose(self, msg_pose2d):
        if self.last_known_pose is None and msg_pose2d is not None:
            rospy.loginfo("Received initial pose, starting script")
        self.last_known_pose = Pose(msg_pose2d.x, msg_pose2d.y, msg_pose2d.theta)

    def read_script(self):
        id = 0
        rate = rospy.Rate(5) #5hz   TODO change
        self._reset_environment()
        rospy.loginfo("Initialized env")

        script = scripts[current_script].copy()
        current_run = None

        while not rospy.is_shutdown():

             # Stall until we receive first location
            while self.last_known_pose is None:
                rospy.loginfo("Waiting for initial pose ...")
                rate.sleep()

            if current_run is None:
                current_run = Run(id,self.last_known_pose,None)
                id+=1

            if self.op_start_time is None:
                self.op_start_time = rospy.Time.now()

            # Execute script

            car_control_msg = Twist2DStamped()                
            execution_time = (rospy.Time.now() - self.op_start_time).to_sec()

            try: # see if there is an action remaining in the script
                op_v,op_w,op_dur = script[0]
                # If the duration is  over, remove command from list
                if execution_time > op_dur:
                    script.pop(0)
                    self.op_start_time = None
                    continue # This skips the sleep, as intended
                else:
                    car_control_msg.v = op_v
                    car_control_msg.omega = op_w
                self.pub_car_cmd.publish(car_control_msg)
                rospy.loginfo(f"Sent command of: v{car_control_msg.v}, w{car_control_msg.omega}")
            except IndexError: # No action left in script
                # update and commit run
                current_run.end_pose=self.last_known_pose
                self.runs.append(current_run)
                script = scripts[current_script].copy()     #reload script
                rospy.loginfo("Script over, sending reset env request")
                self._reset_environment()
                self.last_known_pose = None
                current_run = None

            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo("Sleep interrupted")

    def _reset_environment(self):
        self.pub_env_reset.publish(Bool(data=True))
        rospy.loginfo("Environment reset")

    def safe_shutdown(self):
        rospy.loginfo("Shutting down, sending stop command")
        car_control_msg = Twist2DStamped()                
        car_control_msg.v = 0.0
        car_control_msg.omega = 0.0
        self.pub_car_cmd.publish(car_control_msg)
        rospy.loginfo(f"Sent command of: v{car_control_msg.v}, w{car_control_msg.omega}")

        self.write_runs()

    def write_runs(self):
        filenum = 1
        file_path = "/duckietown/custom/{}-{}".format(current_script, filenum)
        while os.path.isfile(file_path):
            filenum+=1
            file_path = "/duckietown/custom/{}-{}".format(current_script, filenum)

        rospy.loginfo("Writing runs to {}".format(file_path))

        with open(file_path, 'w') as f:
            w = csv.writer(f)
            # field headers
            w.writerow(('ID','start_x','start_y','start_theta','end_x','end_y','end_theta'))
            w.writerows([(r.id, r.start_pose, r.end_pose) for r in self.runs])

class Pose(object):
    def __init__(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return "{:.4f},{:.4f},{:.4f}".format(self.x,self.y,self.theta)

class Run(object):
    def __init__(self, id, start_pose, end_pose):
        self.id = id
        self.start_pose = start_pose
        self.end_pose = end_pose

    def __str__(self):
        return "{},{},{}".format(self.id,self.start_pose,self.end_pose)

if __name__ == "__main__":

    rospy.init_node("static_controller_node", anonymous=False)  # adapted to sonjas default file

    static_controller_node = static_controller()

    rospy.spin()