This code is a IK solver for a 2 degree of freedom robot arm, the important part is the funtion IK(), as it is right now this funtion takes in X and Y coordinates and spits out a pair of angles in degrees. this is still in progress and will improve overtime



TODO:
motion planner 

real time control

more commands

ardunio code

servo control



To use: 

1.set L1 and L2 (the length of your arm segments) on line 8 and 9

2.set the x and y coordinates of your desired end effector location line 32 and 33

Commands:

point: provides arm angles for the desired X,Y coordinates. use  point (Xint,Yint)
