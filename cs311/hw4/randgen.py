import random
import sys

if (sys.argv[1] == 'u'):
	for i in range(int(sys.argv[2])):
		print ''.join([random.choice("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz") for j in range(random.randint(1,256))])
else:
	count = 0
	while (count < 1000):
		word = ''.join([random.choice("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz") for j in range(random.randint(1,30))])
		for i in range(random.randint(1,20)):
			print word
			count += 1
			if (count == 1000):
				break

