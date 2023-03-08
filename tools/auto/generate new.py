#
# BLACK MAGIC CODE AHEAD
# BAD FORMATTING AS WELL
# made as a "works for now, will fix later (TM)"
# very verbose logging because I cannot be bothered to clean it up right now
# will clean up eventually
#
from sympy import *
import json
import toml
import os
import sys

script_dir = os.path.dirname(__file__)

# toml_file = toml.load("{}/../src/main/deploy/config.toml".format(script_dir))["auto"]

# max_speed = toml_file["max_speed"]
max_speed = 4
acceleration_time = 0.25

#data = json.load("{}/../src/main/deploy/{}".format(script_dir, toml_file["pathplanner_file"]))
file = open("{}/test.json".format(script_dir), "r")
data = json.load(file)

if(os.path.exists("{}/out.json".format(script_dir))): #remove the file if it exists
    os.remove("{}/out.json".format(script_dir))
output = open("{}/out.json".format(script_dir), "x")

def doNothing(): # in python you cannot have a try without a catch, this is here just do do nothing :)
    return


#gets the lengths of the bezier curves from the control points
def calculate(a_x, a_y, b_x, b_y, c_x, c_y, d_x, d_y):
    global max_speed
    t = symbols('t') #represents time (0 to 1)

    x_pos = [a_x,b_x,c_x,d_x]
    y_pos = [a_y,b_y,c_y,d_y]

    # derived from P = ((1-t)^3)P1 + (((1-t)^2)t)P2 + (1-t)(t^2)P3 + (t^3)P4
    #
    # https://javascript.info/bezier-curve#maths
    # accessed March 2, 2023
    x = ((1-t)**3)*x_pos[0] + ((1-t)**2)*(t)*x_pos[1] + ((1-t)**1)*(t**2)*x_pos[2] + (t**3)*x_pos[3]
    y = ((1-t)**3)*y_pos[0] + ((1-t)**2)*(t)*y_pos[1] + ((1-t)**1)*(t**2)*y_pos[2] + (t**3)*y_pos[3]

    # calculate the derivative of both directions for the arc-length of the bezier curve section
    dx = diff(x, t)
    dy = diff(y, t)

    """
           /\b
       1   |
      ___  |  sqrt(x(t)^2  +  y(t)^2) dt = length of f(x) between a and b (presuming parametric components of x(t) and y(t))  
      b-a  |
          \/a
    """
    len = integrate(sqrt((dx**2) + (dy**2)), (t, 0, 1)) # does the integral from 0 to 1, so no need to divide by anything because it will be 1 :)

    # calculate the integral to 7 digits (will be an approximation, but will be close enough enough since the FPGA timestamp only goes to 6 decimal places)
    #
    # https://github.com/wpilibsuite/allwpilib/blob/e5c4c6b1a727b6e7527ebfd13b776ee0e68645a6/wpilibc/src/main/native/cpp/Timer.cpp#L87
    appr_len = N(len, 7)
    seconds = appr_len/max_speed

    return seconds

def start():
    out = []
    for i in range(len(data["segments"])):
        seg = data["segments"][i]
        # get the lengths of the bezier curves and append the output to the out array
        time = calculate(seg[0][0], seg[0][1], seg[1][0], seg[1][1], seg[2][0], seg[2][1], seg[3][0], seg[3][1])
        # out.append(time*2 if time<(acceleration_time*2) else time+(acceleration_time))
        out.append(time)
    print("\"time\":{}".format(out))
start()