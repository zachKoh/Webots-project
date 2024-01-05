# PioneerNavigation Class Definition
# File: pioneer_nav2.py
# Date: 24th Nov 2022
# Description: Simple Navigation Class support (2022)
# Author: Terry Payne (trp@liv.ac.uk)

from controller import Supervisor, Node

import math
import pose
from enum import Enum
import pioneer_proxsensors1 as pps

class PioneerNavigation:
    """ A custom class to initialise and manage simple navigation on a Pioneer Adept robot """

    WHEEL_RADIUS = 0.0957 # in meters - found using CONFIGURE
    AXEL_LENGTH = 0.323   # in meters- found using CONFIGURE

    def __init__(self, robot, init_pose, ps):
        
        self.prox_sensors = ps       #reference to proximity sensors

        self.robot = robot                        # reference to the robot
        self.robot_node = self.robot.getSelf()    # reference to the robot node
    
        # enable motors
        self.left_motor = self.robot.getDevice('left wheel')
        self.right_motor = self.robot.getDevice('right wheel')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        self.max_vel = self.left_motor.getMaxVelocity() - 0.1 # Fudge: just under max vel
        
        timestep = int(robot.getBasicTimeStep())

        # set up pose
        self.robot_pose = pose.Pose(init_pose.x, init_pose.y, init_pose.theta)
        
        # Initialise motor velocity
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)   

        self.prev_error = 0
        self.total_error = 0
        
    def get_real_pose(self):
        if self.robot_node is None:
            return pose.Pose(0, 0, 0)
            
        real_pos = self.robot_node.getPosition()
        rot = self.robot_node.getOrientation()
        theta = math.atan2(-rot[0], rot[3])
        halfpi = math.pi / 2
        theta2 = theta + halfpi
        if (theta > halfpi):
            theta2 = -(3 * halfpi) + theta
        
        return pose.Pose(real_pos[0], real_pos[1], theta2)
    
    def forward(self, target_dist, robot_linearvelocity):
        wheel_av = (robot_linearvelocity/self.WHEEL_RADIUS)
        target_time = target_dist/robot_linearvelocity
        
        self.left_motor.setVelocity(wheel_av)
        self.right_motor.setVelocity(wheel_av)
        
        # return target_time as millisecs          
        return 1000.0*target_time
          

    def stop(self):   
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)


    def set_velocity(self, base, control=0):
        # base gives the velocity of the wheels in m/s
        # control is an adjustment on the main velocity
        base_av = (base/self.WHEEL_RADIUS)
    
        if (control != 0):
            control_av = (control/self.WHEEL_RADIUS)
            # Check if we exceed max velocity and compensate
            correction = 1
            lv = base_av - control_av
            rv = base_av + control_av
        
            if (lv > self.max_vel):
                correction = self.max_vel / lv
                lv = lv * correction
                rv = rv * correction
            
            if (rv > self.max_vel):
                correction = self.max_vel / rv
                lv = lv * correction
                rv = rv * correction
                                        
        else:
            lv = rv = base_av
                
        self.left_motor.setVelocity(lv)
        self.right_motor.setVelocity(rv)


    #Changed pid controller to limit control output and adjusted some parameters
    def pid(self, error):
        kp = 0.5  # Proportional weight (may need tuning)
        kd = 3.0  # Derivative weight (may need tuning)
        ki = 0.0  # Integral weight (may need tuning)

        prop = error
        diff = error - self.prev_error
        self.total_error += error

        control = (kp * prop) + (ki * self.total_error) + (kd * diff)
        #print(control)

        #limit the control output to prevent big changes
        max_control = 1
        control = max(min(control, max_control), -max_control)

        self.prev_error = error

        return control

    #Improved to handle corner turning better.
    def follow_wall(self, robot_linearvelocity, set_point, right=True):

        if right:
            direction_coeff = -1
        else:
            direction_coeff = 1
        
        # Approaching a wall, turn
        if(min(self.prox_sensors.get_value(1), 
               self.prox_sensors.get_value(2),
               self.prox_sensors.get_value(3),
               self.prox_sensors.get_value(4),
               self.prox_sensors.get_value(5),
               self.prox_sensors.get_value(6)) < set_point):
            self.set_velocity(robot_linearvelocity/3, -0.4*direction_coeff) #changed multiplier for rate of change of direction so that robot turns quicker when it approaches a wall
        else:
            if not right: 
                wall_dist = min(self.prox_sensors.get_value(1),
                                self.prox_sensors.get_value(0))
            else:
                wall_dist = min(self.prox_sensors.get_value(7),
                                self.prox_sensors.get_value(8))
  
            # Running aproximately parallel to the wall
            if (wall_dist < self.prox_sensors.max_range and self.prox_sensors.get_value(7) < 1):
                error = wall_dist - set_point
                control = self.pid(error)
                # adjust for right wall
                self.set_velocity(robot_linearvelocity, control*direction_coeff)
            #More accurate detection of a corner in the wall
            elif (self.prox_sensors.get_value(7) > 1):
                # No wall, so turn
                self.set_velocity(robot_linearvelocity, 0.20*direction_coeff)
            else:
                # No wall, so turn
                self.set_velocity(robot_linearvelocity, 0.20*direction_coeff)


    
    def turn_towards_target(self, target_x, target_y, robot, distance):

        robot_pose = pose.Pose(0.0, 0.0, 0.0)
        prox_sensors = pps.PioneerProxSensors(robot, "sensorDisplay", robot_pose)
        nav = PioneerNavigation(robot, robot_pose, prox_sensors)
        display_status = "Turning towards target..."
        telemetry_display = robot.getDevice('telemetryDisplay')
        
        #getting and displaying all the telemetry data
        if telemetry_display is not None:
            telemetry_display.setColor(0xFFFFFF)
            telemetry_display.fillRectangle(0,0,
                    telemetry_display.getWidth(),
                    telemetry_display.getHeight())
        
            telemetry_display.setColor(0x000000)
            telemetry_display.setFont("Arial", 18, True)
            telemetry_display.drawText("Robot Telemetry", 1, 1)
  
            telemetry_display.setFont("Arial", 12, True)
            if (display_status != ""):
                telemetry_display.drawText(display_status, 1, 30)
                        
            true_pose = nav.get_real_pose()
            telemetry_display.drawText(f"Location: {true_pose}", 1, 50)

            #Get and display distance travelled
            telemetry_display.drawText(f"Distance travelled: {distance}", 1, 70)

        #calculate the angle to the target
        delta_x = target_x - self.get_real_pose().x
        delta_y = target_y - self.get_real_pose().y
        target_angle = math.atan2(delta_y, delta_x)

        #calculate the difference between the target angle and the robot's current angle
        angle_difference = target_angle - self.get_real_pose().theta

        #set the turning velocity based on the sign of the angle difference
        turn_velocity = 0.3 if angle_difference >= 0 else -0.3

        #make the robot turn
        self.left_motor.setVelocity(-turn_velocity)
        self.right_motor.setVelocity(turn_velocity)
        self.robot.step(100)

        #keep turning until the angle difference is small enough
        while abs(angle_difference) > 0.05:
            angle_difference = target_angle - self.get_real_pose().theta
            self.robot.step(100)

        #stop turning
        self.stop()


    def calculate_distance(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.theta - point2.theta)**2)
    
    def calculate_euclidean_distance(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)