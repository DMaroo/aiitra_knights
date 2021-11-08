#!/usr/bin/env python3

from __future__ import print_function

from darp_path_planner.srv import DARPPath, DARPPathResponse

import rospy
from darpinPoly import DARPinPoly
import sys

def serve_path(request):
    rospy.logdebug(f"Serving path for: mode = {request.mode} | bot = {request.bot} | step = {request.step}")
    return DARPPathResponse(poly.final_paths[request.mode][request.bot][request.step][2], poly.final_paths[request.mode][request.bot][request.step][3], rospy.Time.now())

def server():
    rospy.init_node('path_service')
    rospy.Service('darp_path', DARPPath, serve_path)
    rospy.logdebug(f"Server initialized!")
    rospy.spin()

if __name__ == '__main__':
    rows = rospy.get_param("rows", 10)
    cols = rospy.get_param("cols", 10)
    maxIter = rospy.get_param("maxIter", 80000)
    CCvariation = rospy.get_param("CCvariation", 0.01)
    randomLevel = rospy.get_param("randomLevel", 0.0001)
    dcells = rospy.get_param("dcells", 2)
    importance = rospy.get_param("importance", False)
    nep = rospy.get_param("nep", False)
    p_initial_positions = rospy.get_param("initial_positions", [1, 3, 9])
    p_portions = rospy.get_param("portions", [0.2, 0.3, 0.5])
    p_obstacles_positions = rospy.get_param("obstacles_positions", [])

    obstacles_positions = []
    initial_positions = []

    for position in p_initial_positions:
        if position < 0 or position >= rows*cols:
            print("Initial positions should be inside the Grid.")
            sys.exit(2)
        initial_positions.append((position // cols, position % cols))
        #saving position input to print for path optimization simulation results
        pos_in_long = p_initial_positions

    # finding row and column of obstalces position from grid position
    for obstacle in p_obstacles_positions:
        if obstacle < 0 or obstacle >= rows*cols:
            print("Obstacles should be inside the Grid.")
            sys.exit(3)
        obstacles_positions.append((obstacle // cols, obstacle % cols))

    portions = []
    if nep:
        for portion in p_portions:
            portions.append(portion)
    else:
        for drone in range(len(initial_positions)):
            portions.append(1/len(initial_positions))

    if len(initial_positions) != len(portions):
        print("Portions should be defined for each drone")
        sys.exit(4)

    s = sum(portions)
    if abs(s-1) >= 0.0001:
        print("Sum of portions should be equal to 1.")
        sys.exit(1)

    for position in initial_positions:
        for obstacle in obstacles_positions:
            if position[0] == obstacle[0] and position[1] == obstacle[1]:
                print("Initial positions should not be on obstacles")
                sys.exit(3)

    poly = DARPinPoly(rows, cols, maxIter, CCvariation, randomLevel, dcells, importance, nep, initial_positions, portions, obstacles_positions)

    rospy.logdebug("Path planned!")

    server()


    
