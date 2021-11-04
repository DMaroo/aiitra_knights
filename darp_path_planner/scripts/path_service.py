#!/usr/bin/env python3

from __future__ import print_function

from darp_path_planner.srv import DARPPath, DARPPathResponse

import rospy
from darpinPoly import DARPinPoly

def serve_path(request):
    rospy.logdebug(f"Serving path for: mode = {request.mode} | bot = {request.bot} | step = {request.step}")
    return DARPPathResponse(poly[request.mode][request.bot][request.step][2], poly[request.mode][request.bot][request.step][3], rospy.Time.now())

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
    initial_positions = rospy.get_param("initial_positions", [[0, 1], [0, 3], [0, 9]])
    portions = rospy.get_param("portions", [0.2, 0.3, 0.5])
    obstacles_positions = rospy.get_param("obstacles_positions", [])

    poly = DARPinPoly(rows, cols, maxIter, CCvariation, randomLevel, dcells, importance, nep, initial_positions, portions, obstacles_positions)

    rospy.logdebug("Path planned!")

    server()


    
