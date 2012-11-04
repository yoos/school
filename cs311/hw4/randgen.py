import random

for i in range(1000):
	print ''.join([random.choice("abcdefghijklmnopqrstuvwxyz") for j in range(random.randint(1,20))])


