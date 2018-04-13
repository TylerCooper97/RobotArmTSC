'''Tyler Cooper
3/21/18
Robot Arm project
I worked with Micah and Ethan to create this code
'''

import Search as aima
import numpy as np
import sys
import matplotlib.pyplot as plt
import random
from shapely.geometry import LineString, Polygon
import argparse

slomo = 0 #this is the speed at which the plot function plots the line, by default slomo is turned off. to turn it on, change the 0 to a 1

class My_Problem(aima.Problem):
    """My_Problem copies the problem class from aima and adds its own attributes to it"""
    def __init__(self, initial, goal, length, obstacles, limits):
        aima.Problem.__init__(self, initial, goal)
        self.lengths = length
        self.obstacles = obstacles
        self.limits = limits

    def actions(self, state):
        lact = [[0]*len(state) for _ in range(2*len(state))] #list of actions
        for i in range(len(state)): #take in the angle and create all possible actrions from that
            lact[2*i][i] = 5
            lact[2*i+1][i] = -5
        for i in lact: #creating the limits
            for j in range(len(i)):
                ang = state[j][-1] + i[j]
                if not self.limits[j][0] < ang < self.limits[j][1]:
                    lact.remove(i)
                    break
        return lact

    def result(self, state, action):
        newstate = [i[:] for i in state]
        for i in range(len(state)):
            newstate[i].append(newstate[i][-1]+action[i])

        thetas = [newstate[i][-1] * np.pi / 180 for i in range(len(newstate))]
        x = [0] * (len(thetas) + 1)
        y = [0] * (len(thetas) + 1)
        sumtheta = np.cumsum(thetas)
        for i in range(len(thetas)): #part of forward kinematics
            x[i + 1] = self.lengths[i] * np.cos(sumtheta[i])
            y[i + 1] = self.lengths[i] * np.sin(sumtheta[i])
        x = np.cumsum(x)
        y = np.cumsum(y)
        coordlist = [[] for _ in range(len(x) -1)]
        for i in range(len(x) - 1):
            coordlist[i] = [(x[i], y[i]), (x[i+1], y[i+1])]
        linelist = [LineString(i) for i in coordlist]
        for i in self.obstacles :
            for j in linelist:
                if i.intersects(j):
                    return state
        return newstate

    def scheduler(self, k=20, lam=0.08, limit=500):
        return lambda t: (k * np.exp(-lam * t) if t < limit else 0)

    def value(self, state):
        thetavals = [state[i][-1] * np.pi / 180 for i in range(len(state))]  # list of theta values
        curtheta = 0 #current value of theta
        x = 0
        y = 0
        for i in range(len(thetavals)): #part of forward kinematics
            curtheta += thetavals[i]
            x += self.lengths[i] * np.cos(curtheta)
            y += self.lengths[i] * np.sin(curtheta)
        return -np.sqrt((x-self.goal[0])**2 + (y-self.goal[1])**2) #minimize the distance from the end of the arm to the goal state
def restart(problem):
    """This is the random restart hillclimbing method."""
    start = aima.hill_climbing(problem)
    running = [start]
    while problem.value(running[-1]) < -0.05:
        starttheta = [[random.randint(problem.limits[i][0], problem.limits[i][1])] for i in range(len(problem.initial))]
        problem.initial = starttheta
        running.append(aima.hill_climbing(problem))
    return running

def plot(lens, limits, joints, obstacles, alg): #this is main
    """This plots the line and the obstacles"""
    Problem = My_Problem(joints, goal, lens, obstacles, limits)
    if alg == "hc":
        # hillclimb is not the hillclimbing method, it is just a placeholder for whatever method is going to be used
        hillclimb = [aima.hill_climbing(Problem)]
    elif alg == "sima":
        hillclimb = [aima.simulated_annealing(Problem)]
    else:
        hillclimb = restart(Problem)

    thetas = [hillclimb[0][i][0] * np.pi / 180 for i in range(len(hillclimb[0]))]
    x = [0] * (len(thetas) + 1)
    y = [0] * (len(thetas) + 1)
    sumtheta = np.cumsum(thetas)
    for i in range(len(thetas)):
        x[i + 1] = Problem.lengths[i] * np.cos(sumtheta[i])
        y[i + 1] = Problem.lengths[i] * np.sin(sumtheta[i])
    x = np.cumsum(x) #part of forward kinematics
    y = np.cumsum(y)

    #tarplee code
    axes = plt.gca()
    leng = np.sum(Problem.lengths)
    axes.set_xlim(-leng, leng)
    axes.set_ylim(-leng, leng)

    for i in Problem.obstacles: #plotting obstacles
        xp, yp= i.exterior.xy
        plt.plot(xp,yp,"g-")


    line,=axes.plot(x,y, 'r-')
    plt.plot(Problem.goal[0], Problem.goal[1], "bo")
    for h in range(len(hillclimb)):

        for j in range(len(hillclimb[h][0])):
            thetas = [hillclimb[h][i][j] * np.pi / 180 for i in range(len(hillclimb[h]))] #part of forward kinematics
            x = [0] * (len(thetas) + 1)
            y = [0] * (len(thetas) + 1)
            sumtheta = np.cumsum(thetas)
            for i in range(len(thetas)):
                x[i + 1] = Problem.lengths[i] * np.cos(sumtheta[i])
                y[i + 1] = Problem.lengths[i] * np.sin(sumtheta[i])
            x = np.cumsum(x)
            y = np.cumsum(y)
            line.set_xdata(x)
            line.set_ydata(y)
            plt.draw()
            if slomo == 0:
                plt.pause(.01)
            else:
                plt.pause(.1)

    print("Final angle configuration:", [i[-1] for i in hillclimb[-1]]) #prints the final angle config
    print("Final value of the state:", Problem.value(hillclimb[-1])) #prints the value of the final state

    plt.show()

if __name__ == "__main__":
    #arguments for argparse
    parser = argparse.ArgumentParser(description=" ")
    argv = sys.argv[1:]
    parser.add_argument("arm_file", help=" ")
    parser.add_argument("goal", help=" ")
    if "--joints" in argv:
        i = argv.index("--joints")
        argv2 = argv[:i] + argv[i+2:]
        if "--alg" in argv2:
            j = argv2.index("--alg")
            argv3 = argv2[:j] + argv2[j + 2:]
            parser.add_argument("ob_files", nargs=argparse.REMAINDER)
            args, joints, alg = parser.parse_args(argv3), argv[i + 1], argv2[j + 1]
        else:
            parser.add_argument("ob_files", nargs=argparse.REMAINDER)
            args, joints, alg = parser.parse_args(argv2), argv[i + 1], "lbs"
    else:
        if "--alg" in argv:
            j = argv.index("--alg")
            argv2 = argv[:j] + argv[j + 2:]
            parser.add_argument("ob_files", nargs=argparse.REMAINDER)
            args, joints, alg = parser.parse_args(argv2), [], argv[j + 1]
        else:
            parser.add_argument("ob_files", nargs=argparse.REMAINDER)
            args, joints, alg = parser.parse_args(argv), [], "lbs"
    lens = []
    limits = []
    obstacles = []
    for i in args.ob_files:
        with open(i) as file:
            coords = []
            for data in file:
                coords.append(tuple([int(i) for i in data.split(' ')]))
        obstacles.append(Polygon(coords))
    with open(args.arm_file) as file: #useing file to refer to file object
        for data in file:
            datalist = [i for i in data.split(' ')]
            lens.append(int(datalist[1]))
            limits.append((int(datalist[2]), int(datalist[3])))
    goal = [int(i) for i in args.goal.split(',')]
    if not joints:
        joints = [[random.randint(limits[i][0], limits[i][1])]for i in range(len(lens))]
    else:
        joints = [[int(i)] for i in joints.split(',')]
    print(lens, limits, goal, joints)
plot(lens, limits, joints, obstacles, alg)

