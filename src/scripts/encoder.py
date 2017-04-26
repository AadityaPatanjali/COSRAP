from RPi import GPIO
from time import sleep

M1_channel_A=17
M1_channel_B =18

M2_channel_A=22
M2_channel_B=23

M1_counter =0
M1_channel_ALastState =0
M2_counter =0
M2_channel_ALastState =0

def left_encoder(inp):

    global M1_counter
    if inp != 10:
        M1_counter += 1
   # "print "left Encoder count ",M1_counter
    return M1_counter


def right_encoder(inp):

    global M2_counter
    if inp != 10:
        M2_counter += 1
   # print "right Encoder count ",M2_counter
    return M2_counter
