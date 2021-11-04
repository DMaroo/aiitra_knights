from darp import DARP
import numpy as np
from kruskal import Kruskal
from CalculateTrajectories import CalculateTrajectories
import sys
import argparse

class DARPinPoly(DARP):
    def __init__(self, nx, ny, MaxIter, CCvariation, randomLevel, dcells, importance, notEqualPortions, initial_positions, portions, obstacles_positions):
        DARP.__init__(self, nx, ny, MaxIter, CCvariation, randomLevel, dcells, importance, notEqualPortions, initial_positions, portions, obstacles_positions)

        if not self.success:
            print("DARP did not manage to find a solution for the given configuration!")
            sys.exit(0)

        FinalPaths = []

        for mode in range(4):
            #print("###### MODE "+str(mode)+" ########\n")
            MSTs = self.calculateMSTs(self.BinaryRobotRegions, self.droneNo, self.rows, self.cols, mode)
            AllRealPaths = []
            for r in range(self.droneNo):
                ct = CalculateTrajectories(self.rows, self.cols, MSTs[r])
                ct.initializeGraph(self.CalcRealBinaryReg(self.BinaryRobotRegions[r], self.rows, self.cols), True)
                ct.RemoveTheAppropriateEdges()
                ct.CalculatePathsSequence(4 * self.init_robot_pos[r][0] * self.cols + 2 * self.init_robot_pos[r][1])
                AllRealPaths.append(ct.PathSequence)

            # To print all the paths plannes
            #print(AllRealPaths)
            FinalPaths.append(AllRealPaths)
        
        self.final_paths = FinalPaths


    def CalcRealBinaryReg(self, BinaryRobotRegion, rows, cols):
        temp = np.zeros((2*rows, 2*cols))
        RealBinaryRobotRegion = np.zeros((2 * rows, 2 * cols), dtype=bool)
        for i in range(2*rows):
            for j in range(2*cols):
                temp[i, j] = BinaryRobotRegion[(int(i / 2))][(int(j / 2))]
                if temp[i, j] == 0:
                    RealBinaryRobotRegion[i, j] = False
                else:
                    RealBinaryRobotRegion[i, j] = True

        return RealBinaryRobotRegion

    def calculateMSTs(self, BinaryRobotRegions, droneNo, rows, cols, mode):
        MSTs = []
        for r in range(self.droneNo):
            k = Kruskal(rows, cols)
            k.initializeGraph(self.BinaryRobotRegions[r, :, :], True, mode)
            k.performKruskal()
            MSTs.append(k.mst)
        return MSTs


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '-grid',
        default=(10, 10),
        type=int,
        nargs=2,
        help='Dimensions of the Grid (default: (10, 10))')
    argparser.add_argument(
        '-obs_pos',
        default=[],
        nargs='*',
        type=int,
        help='Dimensions of the Grid (default: (10, 10))')
    argparser.add_argument(
        '-in_pos',
        default=[1, 3, 9],
        nargs='*',
        type=int,
        help='Initial Positions of the robots (default: (1, 3, 9))')
    argparser.add_argument(
        '-nep',
        action='store_true',
        help='Not Equal Portions shared between the Robots in the Grid (default: False)')
    argparser.add_argument(
        '-portions',
        default=[0.2, 0.3, 0.5],
        nargs='*',
        type=float,
        help='Portion for each Robot in the Grid (default: (0.2, 0.7, 0.1))')
    args = argparser.parse_args()

    rows, cols = args.grid

    obstacles_positions = []
    initial_positions = []

    # finding row and column of bot position from grid position
    for position in args.in_pos:
        if position < 0 or position >= rows*cols:
            print("Initial positions should be inside the Grid.")
            sys.exit(2)
        initial_positions.append((position // cols, position % cols))
        #saving position input to print for path optimization simulation results
        pos_in_long = args.in_pos

    # finding row and column of obstalces position from grid position
    for obstacle in args.obs_pos:
        if obstacle < 0 or obstacle >= rows*cols:
            print("Obstacles should be inside the Grid.")
            sys.exit(3)
        obstacles_positions.append((obstacle // cols, obstacle % cols))

    portions = []
    if args.nep:
        for portion in args.portions:
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

    MaxIter = 80000
    CCvariation = 0.01
    randomLevel = 0.0001
    dcells = 2
    importance = False

    # Printing initial conditions
    #print("\nInitial Conditions Defined:")
    #print("Grid Dimensions:", rows, cols)
    #print("Robot Number:", len(initial_positions))
    #print("Initial Robots' positions", initial_positions)
    #print("Portions for each Robot:", portions, "\n")

    poly = DARPinPoly(rows, cols, MaxIter, CCvariation, randomLevel, dcells, importance, args.nep, initial_positions, portions, obstacles_positions)
