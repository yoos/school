#!/usr/bin/env python

import subprocess
import socket

procList = []   # List of compute process PIDs
result = []   # List of perfect numbers found so far.


# Create server socket.
ss = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
ss.bind((socket.gethostname(), 50117))
ss.listen(5)   # Queue up to 5 requests.


# My send function
def msend(sock, msg):
	totalsent = 0
	while totalsent < len(msg):
		sent = sock.send(msg[totalsent:])
		if sent == 0:
			raise RuntimeError("socket connection broken")
		totalsent = totalsent + sent

# My receive function
def mreceive(sock, msglen):
	msg = ''
	while len(msg) < msglen:
		chunk = sock.recv(msglen-len(msg))
		if chunk == '':
			raise RuntimeError("socket connection broken")
		msg = msg + chunk
	return msg


# Spawn new compute process
def spawnCompute ():
	pid = subprocess.Popen(["./compute"])
	return pid

# Receive performance info and range request from compute process
def computeCallback ():


# Send back range to compute process


# Receive request from report.py for perfect numbers calculated so far

# Send back information to report.py


# If asked to die, stop spawning new compute processes and die.

if __name__ == "__main__":

	# Accept connections and dole out number ranges.
	while 1:
		(cs, addr) = ss.accept()
		#now do something with the clientsocket
		#in this case, we'll pretend this is a threaded server
		ct = client_thread(clientsocket)
		ct.run()


