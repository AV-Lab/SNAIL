#! /usr/bin/env python

import struct
import socket
from ctypes import *
from system_defines import *

"""
Make a 16-bit value from two 8-bit values
"""
def m16( a, b ):

    return ((( a << 8) | (a & 0x0FF)) & 0x0FFFF)

"""
Make a 32-bit value from four 8-bit values
"""
def m32(a,b,c,d):
    
    ret = 0;
    ret |= (a & 0xFF) << 24
    ret |= (b & 0xFF) << 16
    ret |= (c & 0xFF) << 8
    ret |= (d & 0xFF)

    return ret

"""
Make an array of 8-bit values from a 32-bit value
"""
def m8_from_32 (a):
    ret = [0]*4;
    
    ret[0] = int((a & 0xFF000000) >> 24)
    ret[1] = int((a & 0x00FF0000) >> 16)
    ret[2] = int((a & 0x0000FF00) >> 8)
    ret[3] = int((a & 0x000000FF))
    return ret; 

"""
For non IEEE754 processors this function manually converts a 32-bit integer representation
of a floating point value to float representation
"""
def get32bitfloat(value_to_convert):

    SIGN_BIT        = (31)
    SIG_MASK        = (0x7FFFFF)  # bits 22-0
    SIG_LEADING_ONE = (0x800000)  # bit 23 added to significand
    BIAS            = (127)

    sign_bit = (value_to_convert >> SIGN_BIT);
    exponent_field = (((value_to_convert >> 23) & 0xFF) - BIAS);
    significand = value_to_convert & SIG_MASK;
    significand = significand | SIG_LEADING_ONE;

    float32_value = float( (((float(significand)) / pow(2.0, 23.0)) * pow(2.0, float(exponent_field))) );

    #  If the sign bit is a 1, the value is negative.
    if (sign_bit):
        float32_value = float32_value * -1;

    
    return (float32_value);

"""
For IEEE754 processors this function converts a 32-bit floating point number to
a 32-bit integer representation
"""
def convert_float_to_u32(value):
    return struct.unpack('=I', struct.pack('=f', value))[0]

"""
For IEEE754 processors this function converts a 32-bit integer representation
of a floating point value to float representation
"""
def convert_u32_to_float(bits):
    return struct.unpack('=f', struct.pack('=I', bits))[0]

"""
Used to apply a deadband to a signal
"""
def deadband(signal_in, signal_deadband):
    
    return_value = 0.0;

    if (signal_deadband < 0.0):
        return_value = (0.0);
    elif (signal_in > signal_deadband):
        return_value = (signal_in - signal_deadband);
    elif (signal_in < -signal_deadband):
        return_value = (signal_in + signal_deadband);
    else:
        return_value = 0.0;
        
    return return_value;

"""
Used to rate limit a given signal. Note that one must choose the max_rate depending on the
update frequency
"""
def slew_limit_f(signal_in, max_rate, signal_out):
    requested_rate = signal_in - signal_out;

    #
    # Max_rate must be non-negative so that signal_out is modified correctly. 
    #
    if (max_rate >= 0):
        #
        #  Order of comparisons is important in the following if,else statements.  
        #  Go through possible scenarios in order from largest requested_rate to
        #  smallest requested_rate so that first if statement that is true is 
        #  carried out.
        #   
        if (requested_rate > max_rate):
            signal_out += max_rate;
        elif (requested_rate >= -max_rate):
            signal_out = signal_in;
        else:
            signal_out += -max_rate;
    return signal_out

"""
Used to convert a byte array (string) into an array of 32-bit values
"""
def convert_byte_data_to_U32(data):

    rx_dat = [];
    k = 0;
    
    #
    # Convert the string into a byte array
    #
    
    for x in range(0,len(data)):
        rx_dat.append(ord(data[x]));
        
    number_of_u32s = (len(rx_dat)/4)

    #
    # Convert the byte array into an array of 32bit values
    #
    converted = [0]*number_of_u32s;
    for x in range(0,number_of_u32s):
        converted[x] = int((((rx_dat[k]   << 24) & 0xFF000000)) |
                        (((rx_dat[k+1] << 16) & 0x00FF0000)) |
                        (((rx_dat[k+2] << 8)  & 0x0000FF00)) |
                          (rx_dat[k+3] & 0x000000FF));

        k+=4;
        
    return converted;

"""
Used to determine if the input signal is within a valid range
"""
def in_range(signal_in,lower_bound,upper_bound):
    if (signal_in >= lower_bound) and (signal_in <= upper_bound):
        return True
    else:
        return False

"""
Used to determine if the input signal approximately equal to a value within
a maximum delta. Primarily useful for floating point operations where == is undesireable
"""    
def approx_equal(signal1,signal2,max_delta):
    if (abs(signal1 - signal2)< max_delta):
        return True;
    else:
        return False;

"""
Used to convert an IP address string in dotted quad format to an integer
"""  
def dottedQuadToNum(ip):
    "convert decimal dotted quad string to long integer"
    return struct.unpack('=L',socket.inet_aton(ip))[0]

"""
Used to convert an IP address in integer format to a dotted quad string
""" 
def numToDottedQuad(n):
    "convert long int to dotted quad string"
    return socket.inet_ntoa(struct.pack('=L',n))

def clamp_value_f(value,lower_limit,upper_limit):
    
    if (value < lower_limit):
        value = lower_limit;
    elif (value > upper_limit):
        value = upper_limit;
    
    return value;

def sign_f(signal_in):
    
    ret = 0
    
    if (signal_in > 0.0):
        ret = 1;
    elif(signal_in < 0.0):
        ret = -1
        
    return ret;

def print_data(title,data):
    
    temp = [];
    
    for i in range(0,len(data)):
        temp.append(hex(data[i]))
        
    print title, temp






