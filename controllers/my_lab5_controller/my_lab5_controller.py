"""my_lab4_controller controller."""

# my_lab4_controller Class Definition
# File: my_lab4_controller.py
# Date: 15th Nov 2022
# Description: Simple Controller based on 2021 version (2022)
# Author: Terry Payne (trp@liv.ac.uk)
#

from controller import Supervisor
import pioneer_nav2 as pn
import math
import pose
import pioneer_proxsensors1 as pps

from pioneer_nav2 import MoveState

def run_robot(robot):
        
    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    camera = robot.getDevice('camera')
    if camera is not None:
        camera.enable(timestep)

    robot_pose = pose.Pose(0.0, 0.0, 0.0)
       
    prox_sensors = pps.PioneerProxSensors(robot, "sensorDisplay", robot_pose)
    nav = pn.PioneerNavigation(robot, robot_pose, prox_sensors)

    robot_pose = nav.get_real_pose()
    
    robot_velocity = 0.5

    # Target coordinates
    target_x = 2.75
    target_y = -3.26

    while (robot_pose.x != target_x) and (robot_pose.y != target_y):

        # Turn towards target
        nav.turn_towards_target(target_x, target_y)

        #Move forward in the direction of target having already turned to face the target
        print("Moving forward, searching for a wall...")
        while not (min(nav.prox_sensors.get_value(1), 
        nav.prox_sensors.get_value(2),
        nav.prox_sensors.get_value(3),
        nav.prox_sensors.get_value(4),
        nav.prox_sensors.get_value(5),
        nav.prox_sensors.get_value(6)) < 0.2):
            target_time = nav.forward(0.05,0.5)
            robot.step(100)
        
        #Stop when a wall is detected in front
        nav.stop()
        robot_pose = nav.get_real_pose()
        if(nav.calculate_euclidean_distance(target_x, target_y, robot_pose.x, robot_pose.y) < 0.5):
            print("TARGET REACHED, TERMINATED")
            break
        hit_point = nav.get_real_pose()
        print("Found an obstacle! Circling around now.")
        #Variable allows robot to wait a bit before scanning for the hit point
        wait = 0

        min_distance_to_target = 100

        #Move around the whole perimeter of the obstacle
        while robot.step(timestep) != -1:

            wait += 1
            robot_pose = nav.get_real_pose()
            prox_sensors.set_pose(robot_pose)
            prox_sensors.paint()  # Render sensor Display

            if wait == 40:
                hit_point = robot_pose

            cur_distance_to_target = nav.calculate_euclidean_distance(target_x, target_y, robot_pose.x, robot_pose.y) 
            if cur_distance_to_target < min_distance_to_target:
                min_distance_to_target = cur_distance_to_target
                location_of_min_dist = robot_pose

            nav.follow_wall(robot_velocity, 0.2, True)

            distance_threshold = 0.6  #adjust through trial and error
            print("Distance to hit point: " + str(nav.calculate_distance(hit_point, robot_pose)))
            if (nav.calculate_distance(hit_point, robot_pose) < distance_threshold) and (wait > 150):
                print("Back to first point of contact with wall")
                print("Now navigating towards point on wall closest to the target")
                break
        

        while robot.step(timestep) != -1:
            
            robot_pose = nav.get_real_pose()
            prox_sensors.set_pose(robot_pose)
            prox_sensors.paint()

            cur_distance_to_target = nav.calculate_euclidean_distance(target_x, target_y, robot_pose.x, robot_pose.y)
            nav.follow_wall(robot_velocity, 0.2, True)

            if (nav.calculate_distance(location_of_min_dist, robot_pose) < distance_threshold):
                print("Back to point of min distance to target")
                break

  
if __name__ == "__main__":
    # create the Supervised Robot instance.
    my_robot = Supervisor()
    run_robot(my_robot)
