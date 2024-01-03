# PioneerProxSensors Class Definition
# File: pioneer_proxsensors.py
# Date: 24th Jan 2022
# Description: Pose Class support for COMP329 Programming Assignment (2022)
# Author: Terry Payne (trp@liv.ac.uk)

#import numpy as np
import math
import pose


class PioneerProxSensors:
    """ A custom class to manage the 16 proximity sensors on a Pioneer Adept robot """

    # define the display constants
    DARKGREY = 0x3C3C3C
    BLACK = 0x000000
    WHITE = 0xFFFFFF
    LABEL_OFFSET = 0.3
    MAX_NUM_SENSORS = 16

    def __init__(self, robot, display_name, robot_pose):
        self.robot = robot
        self.robot_pose = robot_pose
        
        timestep = int(robot.getBasicTimeStep())

        # Dimensions of the Robot
        # Note that the dimensions of the robot are not strictly circular, as 
        # according to the data sheet the length is 485mm, and width is 381mm
        # so we assume for now the aprox average of the two (i.e. 430mm), in meters
        self.radius = 0.215

        # Insert Constructor Code Here

        # ------------------------------------------
        # set up proximity detectors
        self.ps = []
        for i in range(self.MAX_NUM_SENSORS):
            sensor_name = 'so' + str(i)
            self.ps.append(robot.getDevice(sensor_name))
            self.ps[i].enable(timestep)
    

        # The following array determines the orientation of each sensor, based on the
        # details of the Pioneer Robot Stat sheet.  Note that the positions may be slightly
        # inaccurate as the pioneer is not perfectly round.  Also these values are in degrees
        # and so may require converting to radians.  Finally, we assume that the front of the
        # robot is between so3 and so4.  As the angle between these is 20 deg, we assume that 
        # they are 10 deg each from the robot heading         

        ps_degAngles = [
            90, 50, 30, 10, -10, -30, -50, -90,
            -90, -130, -150, -170, 170, 150, 130, 90
        ]
        ps_angles = [None] * len(ps_degAngles)
        for i in range(len(ps_angles)):
            ps_angles[i] = math.radians(ps_degAngles[i])
                
        # -------------------------------------------
        # Determine the poses of each of the sensors
        self.ps_pose = []
        for i in ps_angles:
            p = pose.Pose(math.cos(i) * self.radius, math.sin(i) * self.radius, i)
            self.ps_pose.append(p)

        # ------------------------------------------
        # Determine max range from lookup table
        lt = self.ps[0].getLookupTable()
        print(f"Lookup Table has {len(lt)} entries")
        self.max_range = 0.0
        for i in range(len(lt)):
            if ((i%3) == 0):
                self.max_range = lt[i]
            print(f" {lt[i]}", end='')
            if ((i%3) == 2):
                print("") # Newline
        self.max_value = self.ps[0].getMaxValue()

        self.display = robot.getDevice(display_name)
        if self.display is not None:
            self.device_width = self.display.getWidth()
            self.device_height = self.display.getHeight()
            self.scalefactor = min(self.device_width, self.device_height) / (2 * (self.max_range + self.radius))
 


    # ==================================================================================
    # Internal (Private) methods
    # ==================================================================================

    # helper methods for mapping to the display
    # Map the real coordinates to screen coordinates assuming
    #   the origin is in the center and y axis is inverted

    def scale(self, l):
        return int(l * self.scalefactor)
    def mapx(self, x):
        return int((self.device_width / 2.0) + self.scale(x))
    def mapy(self, y):
        return int((self.device_height / 2.0) - self.scale(y))
    def rotx(self, x, y, theta):
        return math.cos(theta) * x - math.sin(theta) * y
    def roty(self, x, y, theta):
        return math.sin(theta) * x + math.cos(theta) * y

    # ==================================================================================
    # External (Public) methods
    # ==================================================================================
  
    # Insert public methods here

    def get_maxRange(self):
        return self.max_range

    def get_number_of_sensors(self):
        return len(self.ps)

    def get_value(self, i):
        if (i < len(self.ps)):
            return self.max_range - (self.max_range/self.max_value * self.ps[i].getValue())
        else:
            print("Out of range errorr in get_value")
            return None

    def set_pose(self, p):
        self.robot_pose.set_position(p.x, p.y, p.theta)

    def paint(self):
        if self.display is None:
            return
        
        # draw a background
        self.display.setColor(0xF0F0F0)
        self.display.fillRectangle(0, 0, self.device_width, self.device_height)

        theta = self.robot_pose.theta
        
        # draw robot body
        self.display.setColor(self.WHITE)
        self.display.fillOval(self.mapx(0.0),
                              self.mapy(0.0),
                              self.scale(self.radius),
                              self.scale(self.radius))
                              
        self.display.setColor(self.DARKGREY)
        self.display.drawOval(self.mapx(0.0),
                              self.mapy(0.0),
                              self.scale(self.radius),
                              self.scale(self.radius))
        
        # For each sensor, get the value of the sensor and display the distance
        # Note that we assume an array of three values.  The label will be
        # managed seperately
        xarc = [0, 0, 0]
        yarc = [0, 0, 0]
        x = [0.0, 0.0, 0.0]
        y = [0.0, 0.0, 0.0]

        # Need to indicate heading          
        self.display.drawLine(self.mapx(0.0),
                              self.mapy(0.0),
                              self.mapx(math.cos(self.robot_pose.theta) * self.radius),
                              self.mapy(math.sin(self.robot_pose.theta) * self.radius))                             

        # Insert View based code here

        # Draw triangles of 2*Math.PI/36.0 either side
        # of the sensor orientation
        for i in range(len(self.ps)):
            d = self.get_value(i)
           
            p = self.ps_pose[i]

            x[0] = p.x
            x[1] = p.x + (d * math.cos(p.theta - math.pi/18.0))            
            x[2] = p.x + (d * math.cos(p.theta + math.pi/18.0))

            y[0] = p.y
            y[1] = p.y + (d * math.sin(p.theta - math.pi/18.0))
            y[2] = p.y + (d * math.sin(p.theta + math.pi/18.0))

            for j in range(len(x)):
                xarc[j] = self.mapx(self.rotx(x[j], y[j], self.robot_pose.theta))
                yarc[j] = self.mapy(self.roty(x[j], y[j], self.robot_pose.theta))
            
            self.display.setColor(self.DARKGREY)
            self.display.fillPolygon(xarc, yarc)

        for i in range(len(self.ps)):
            # for display clarity, skip sensors 8 & 15 as they overla with 7 and 0
            if not (i%8):
                continue
                
            d = self.get_value(i)
           
            p = self.ps_pose[i]

            x_label = p.x + ((d+self.LABEL_OFFSET) * math.cos(p.theta))
            y_label = p.y + ((d+self.LABEL_OFFSET) * math.sin(p.theta))
            xl = self.mapx(self.rotx(x_label, y_label, self.robot_pose.theta))
            yl = self.mapy(self.roty(x_label, y_label, self.robot_pose.theta))
            
            self.display.setColor(self.DARKGREY)
            self.display.drawText('{0:.2f}'.format(d), xl, yl)