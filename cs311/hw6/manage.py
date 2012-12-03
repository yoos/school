#!/usr/bin/env python

import subprocess
import socket
import select

procList = []   # List of compute process PIDs
perfNums = []   # List of perfect numbers found so far.


# Create server socket.
ss = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#ss.setblocking(0)
ss.bind((socket.gethostname(), 50117))
ss.listen(5)   # Queue up to 5 requests.


# My send function
def mysend(sock, msg):
	totalsent = 0
	while totalsent < len(msg):
		sent = sock.send(msg[totalsent:])
		if sent == 0:
			raise RuntimeError("socket connection broken")
		totalsent = totalsent + sent

# My receive function
def myreceive(sock, msglen):
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
	print "arst"


# Send back range to compute process


# Receive request from report.py for perfect numbers calculated so far

# Send back information to report.py


# If asked to die, stop spawning new compute processes and die.



if __name__ == "__main__":

	# Spawn compute processes.
	for i in range(1):
		procList.append(spawnCompute())

	# Accept connections and dole out number ranges.
	n = 0

	running = True
	while running:
		# Is ss ready to be read?
		(sread, swrite, sexc) = select.select([ss], [], [])

		for sock in sread:
			if sock == ss:
				print "[M] accepting.."
				(conn, addr) = ss.accept()

				conn.send("[M] hello "+str(i))

				res = conn.recv(1024)
				print "[M] I got:", res   #myreceive(conn, 10)
				if res == '':
					conn.close
				else:
					perfNums.append(res)

				print "[M]", str(n)+"th loop"
				n += 1


	# Kill compute processes.
	for i in range(4):
		print "[M] Killing process", i
		procList[i].wait()

	conn.close()
	ss.close()



