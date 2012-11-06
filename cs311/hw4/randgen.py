import random
import sys

for i in range(int(sys.argv[1])):
	print ''.join([random.choice("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz") for j in range(random.randint(1,256))])

