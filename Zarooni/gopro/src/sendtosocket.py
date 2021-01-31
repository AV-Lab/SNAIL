#!/usr/bin/env python3
# import rospy
import socket
from threading import Thread
import inputs
import struct
import time

# def send(conn):
#     while True:
#         conn.send('a')

# host1 = "10.5.5.9"
# port1 = 10000

# sock1 = socket.socket()
# sock1.connect((host1, port1))

# Thread(target=send, args=[sock1]).start()

send_socket = socket.socket() # Create the socket
send_socket.bind(("10.5.5.100",0))
send_socket.connect(('url', 0))  # Connect to the server
# send_socket.send(data)  # Send the data to the server
