#!/usr/bin/env python

import sys
import csv
import multiprocessing

### CONSTANTS ###
NUM_PROC = 10

if len(sys.argv) < 5:
    print "Not enough arguments"
    exit(1)

train_data_filename  = sys.argv[1]
train_label_filename = sys.argv[2]
test_data_filename   = sys.argv[3]
test_label_filename  = sys.argv[4]

# Read in files. Note that csv.reader will read in each line as a list of words.
train_data  = [line    for line in list(csv.reader(open(train_data_filename, 'rU'),  delimiter=',', quotechar='"'))]
train_label = [line[0] for line in list(csv.reader(open(train_label_filename, 'rU'), delimiter=',', quotechar='"'))]
test_data   = [line    for line in list(csv.reader(open(test_data_filename, 'rU'),   delimiter=',', quotechar='"'))]
test_label  = [line[0] for line in list(csv.reader(open(test_label_filename, 'rU'),  delimiter=',', quotechar='"'))]
vocab = train_data[0]   # For convenience.

# Remove header line.
train_data = train_data[1:]
test_data  = test_data[1:]

# Count some stuff.
num_vocab = len(vocab)
num_train_data = len(train_data) - 1
num_test_data  = len(test_data)  - 1

# Count number of positive/negative reviews.
train_pos = 0
test_pos = 0

for i in range(1, num_train_data):
    if train_label[i] == "pos":
        train_pos += 1
for i in range(1, num_test_data):
    if test_label[i] == "pos":
        test_pos += 1

train_neg = num_train_data - train_pos   # For convenience.
test_neg  = num_test_data  - test_pos


print "Training Bayes classifier."

# Allocate memory for parameters, initializing each parameters with Dirichlet
# priors. Elements in the innermost list in the nested list below are ordered
# according to (word-in-review, positive-review) = [(F, F), (F, T), (T, F), (T,
# T)].
parameter_list = [[1.0/num_vocab] * 4 + [i] for i in range(num_vocab)]

# Synchronized counter used to track progress.
progress = multiprocessing.Value('i', 1)
lock = multiprocessing.Lock()

# Train the classifier.
def train(parameter):
    vocab_num = parameter[-1]

    # Start obfuscation so multiprocessing will work.
    for i in range(num_train_data):
        offset = 2*int(train_data[i][vocab_num])   # If the word is in the review (i.e., feature = 1), we need to update elements 2 and 3.
        r = 1   # Update the appropriate elements.
        if train_label[i] == "pos": r = 0

        parameter[offset+0] = (i*parameter[offset+0]+r)    /(i+1)
        parameter[offset+1] = (i*parameter[offset+1]+(1-r))/(i+1)

    parameter.pop()   # Remove the index element.

    print "Progress:", str(progress.value)+"/"+str(num_vocab), parameter
    with lock:
        progress.value += 1
    return parameter

p = multiprocessing.Pool(NUM_PROC)
parameter_list = p.map(train, parameter_list)
p.close()
p.join()

