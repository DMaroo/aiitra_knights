from darp import DARP
import numpy as np
from kruskal import Kruskal
from CalculateTrajectories import CalculateTrajectories
from Visualization import visualize_paths
import sys
import argparse
from turns import turns
from pprint import pprint


class DARPinPoly(DARP):
    def __init__(self, nx, ny, MaxIter, CCvariation, randomLevel, dcells, importance, notEqualPortions, initial_positions, portions, obstacles_positions, visualization):
        DARP.__init__(self, nx, ny, MaxIter, CCvariation, randomLevel, dcells, importance, notEqualPortions, initial_positions, portions, obstacles_positions, visualization)

        if not self.success:
            print("DARP did not manage to find a solution for the given configuration!")
            sys.exit(0)

        mode_to_drone_turns = dict()
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
            #print("\n\n\n")

            TypesOfLines = np.zeros((self.rows*2, self.cols*2, 2))
            for r in range(self.droneNo):
                flag = False
                for connection in AllRealPaths[r]:
                    if flag:
                        if TypesOfLines[connection[0]][connection[1]][0] == 0:
                            indxadd1 = 0
                        else:
                            indxadd1 = 1

                        if TypesOfLines[connection[2]][connection[3]][0] == 0 and flag:
                            indxadd2 = 0
                        else:
                            indxadd2 = 1
                    else:
                        if not (TypesOfLines[connection[0]][connection[1]][0] == 0):
                            indxadd1 = 0
                        else:
                            indxadd1 = 1
                        if not (TypesOfLines[connection[2]][connection[3]][0] == 0 and flag):
                            indxadd2 = 0
                        else:
                            indxadd2 = 1

                    flag = True
                    if connection[0] == connection[2]:
                        if connection[1] > connection[3]:
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 2
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 3
                        else:
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 3
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 2

                    else:
                        if (connection[0] > connection[2]):
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 1
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 4
                        else:
                            TypesOfLines[connection[0]][connection[1]][indxadd1] = 4
                            TypesOfLines[connection[2]][connection[3]][indxadd2] = 1

            subCellsAssignment = np.zeros((2*self.rows, 2*self.cols))
            for i in range(self.rows):
                for j in range(self.cols):
                    subCellsAssignment[2 * i][2 * j] = self.A[i][j]
                    subCellsAssignment[2 * i + 1][2 * j] = self.A[i][j]
                    subCellsAssignment[2 * i][2 * j + 1] = self.A[i][j]
                    subCellsAssignment[2 * i + 1][2 * j + 1] = self.A[i][j]

            drone_turns = turns(AllRealPaths)
            drone_turns.count_turns()
            mode_to_drone_turns[mode] = drone_turns

            if self.visualization:
                image = visualize_paths(AllRealPaths, subCellsAssignment, self.droneNo, self.color)
                image.visualize_paths(mode)

        #print("\nResults:\n")

        for mode, val in mode_to_drone_turns.items():
            #printing initial positions of the bots in long form as CSV
            for in_pos_input in pos_in_long:
                print(in_pos_input,end=",")
            #printing themode for which the values are being printed
            print(mode,end="")
            straightlinemoves = []
            for i in range(self.droneNo):
                straightlinemoves.append(len(FinalPaths[mode][i])-mode_to_drone_turns[mode].turns[i])
                #print("Straight line moves for robot " + str(i) + " = " + str(len(FinalPaths[mode][i])-mode_to_drone_turns[mode].turns[i]))
            #print("Straight line moves: ",end="")

            #prints straight line moves as CSV
            for num_straights in straightlinemoves:
                print(",",num_straights,sep='',end="")

            #print straight line moves as a list
            #print(straightlinemoves, end="   ")

            #print(val)   #val is an Object that stores data about the turns. : val.turns , val.avg (average turns), val.std (std dev of turns)

            #printing only number of turns
            #print("Turns: ",end="")

            #prints values of number of turns as CSV
            for num_turns in val.turns:
                print(",",num_turns,sep='',end="")

            #prints value of turns as a list
            #print(val.turns, end="   ")

            print("")


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
    argparser.add_argument(
        '-vis',
        action='store_true',
        help='Visualize results (default: False)')
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

    poly = DARPinPoly(rows, cols, MaxIter, CCvariation, randomLevel, dcells, importance, args.nep, initial_positions, portions, obstacles_positions, args.vis)
