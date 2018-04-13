#RobotArmTSC.py
To run this program from the command line, the following example can be followed:
python RobotArmTSC.py <--joints> <--alg> <arm file> <goal> <obstacle file>...
where --joints is an optional command that allows you to specify the angles of the joints
and there may be an infinite number of object files. The other optional command is --alg, which allows 
you to specify which algorithm is run.There are three algorithms to choose from: hillclimbing(--alg hc), 
simulated annealing(--alg sima), and random restart hillclimbing(default). There is no --alg command 
for random restart hillclimbing becuase it is the default algorithm.

#Requirements to run
1) Search.py and utils.py from the aima github page
2) numpy, sys, random, argparse, matplotlib.pyplot, and shapely. Note: shapely is a pain to get working on
a windows machine.
3) An arm file is required to run. If you do not want obstacles, you do not need obstacle files

#Helpful tip
By default, slomo is turned off(value of 0). If you would like to slow down the speed at which the
plot moves, change the global variables value to 1.

#Algorithms implemented
Of the three algorithms implented, random restart hillclimbing is the only complete algorithm. The
other algorithms are incomplete.

#Math used
I did not need to implement any math beyond the forward kinematic equations, however I did have to
implement them in an odd way.