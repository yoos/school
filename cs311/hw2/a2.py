#!/usr/bin/env python2

import subprocess as sp
import shlex

def problem1():
	lnRoot = '/usr/local/classes/eecs/fall2012/cs311'
	dirsToCreate = ['assignments', 'examples', 'exams', 'lecture_notes', 'submissions']

	p = sp.Popen(['ls'], stdout=sp.PIPE)
	dirList = shlex.split(p.communicate()[0])   # Directory listing

	# Function to find an item in a list
	def find(item, lst):
		for each in lst:
			if each == item:
				return True
		return False

	# Create directories.
	for dirName in dirsToCreate:
		if not find(dirName, dirList):
			sp.Popen(shlex.split('mkdir ' + dirName))
			print dirName, 'created.'
		else:
			print dirName, 'already exists.'

	# Create symbolic links
	if not find('website', dirList):
		sp.Popen(shlex.split('ln -s ' + lnRoot + '/public_html website'))
		print 'website created.'
	else:
		print 'website already exists.'
	if not find('handin', dirList):
		sp.Popen(shlex.split('ln -s ' + lnRoot + '/handin handin'))
		print 'handin created.'
	else:
		print 'handin already exists.'


def problem2():
	num = '7316717653133062491922511967442657474235534919493496983520312774506326239578318016984801869478851843858615607891129494954595017379583319528532088055111254069874715852386305071569329096329522744304355766896648950445244523161731856403098711121722383113622298934233803081353362766142828064444866452387493035890729629049156044077239071381051585930796086670172427121883998797908792274921901699720888093776657273330010533678812202354218097512545405947522435258490771167055601360483958644670632441572215539753697817977846174064955149290862569321978468622482839722413756570560574902614079729686524145351004748216637048440319989000889524345065854122758866688116427171479924442928230863465674813919123162824586178664583591245665294765456828489128831426076900422421902267105562632111110937054421750694165896040807198403850962455444362981230987879927244284909188845801561660979191338754992005240636899125607176060588611646710940507754100225698315520005593572972571636269561882670428252483600823257530420752963450'

	print max([int(num[i]) * int(num[i+1]) * int(num[i+2]) * int(num[i+3]) * int(num[i+4]) for i in range(len(num)-4)])


def getList(fileName):
	try:
		f = open(fileName, 'r')
		lst = sorted([item.replace('\"', '') for item in f.read().split(',')])
		f.close()
		return lst
	except IOError:
		print 'WARNING: I can\'t open ' + fileName + '!'
		return []


def problem3():
	nameList = getList('names.txt')

	print sum(sum((i+1) * (ord(letter)-ord('A')+1) for letter in nameList[i]) for i in range(len(nameList)))


def problem4():
	wordList = getList('words.txt')
	triList = [n*(n+1)/2 for n in range(50)]   # This will produce enough triangle numbers to check against a string of 47 Z's.

	print [val in triList for val in [sum(ord(letter)-ord('A')+1 for letter in word) for word in wordList]].count(True)


if __name__ == '__main__':
	while 1:
		print '\nYou may run one of four functions corresponding to the problem numbers of Assignment 2.'
		choice = raw_input('Run which problem? (q to quit): ')
		print ''

		if   choice == '1': problem1()
		elif choice == '2': problem2()
		elif choice == '3': problem3()
		elif choice == '4': problem4()
		elif choice == 'q': exit(0)
		else: print 'Invalid problem number.'

