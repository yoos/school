#!/usr/bin/env python

import sys
import csv
import multiprocessing
from math import log, e

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
num_train_data = len(train_data)
num_test_data  = len(test_data)

# Count number of positive/negative reviews.
num_train_pos = 0
num_test_pos  = 0

for i in range(1, num_train_data):
    if train_label[i] == "pos":
        num_train_pos += 1
for i in range(1, num_test_data):
    if test_label[i] == "pos":
        num_test_pos += 1

num_train_neg = num_train_data - num_train_pos   # For convenience.
num_test_neg  = num_test_data  - num_test_pos


print "Training Bayes classifier."

# Allocate memory for parameters. Elements in the innermost list in the nested
# list below are ordered according to (word-in-review, positive-review) = [(F,
# F), (F, T), (T, F), (T, T)].
#
# Note that each parameter is initialized as a Dirichlet Prior. This means that
# each value of the parameter is equally likely in the absence of data (i.e.,
# we're assuming prior knowledge). As data is collected, the prior term is
# dominated.
#
# TODO: I only need half of these entries, but first check that my current numbers make sense.
parameter_list = [[1.0/num_vocab] * 4 + [i] for i in range(num_vocab)]
prob_review_pos = float(num_train_pos)/num_train_data

# Synchronized counter used to track progress.
progress = multiprocessing.Value('i', 1)
lock = multiprocessing.Lock()

# Train the classifier.
def train(parameter):
    vocab_num = parameter[-1]

    # Start obfuscation so multiprocessing will work.
    in_review_count = 0
    for i in range(num_train_data):
        if train_data[i][vocab_num] == '1':   # If the word is in the review (i.e., feature = 1)..
            in_review_count += 1
            num_elements = in_review_count
            offset = 2   # We need to update elements 2 and 3.
        else:
            num_elements = (i+1) - in_review_count
            offset = 0

        r = 1   # Update the appropriate elements.
        if train_label[i] == "pos": r = 0

        parameter[offset+0] = ((num_elements-1) * parameter[offset+0]+r)     / num_elements
        parameter[offset+1] = ((num_elements-1) * parameter[offset+1]+(1-r)) / num_elements

    parameter.pop()   # Remove the index element.

    sys.stdout.flush()
    sys.stdout.write("Progress: %5d/%5d\r" % (progress.value, num_vocab))
    with lock:
        progress.value += 1
    return parameter

p = multiprocessing.Pool(NUM_PROC)
parameter_list = p.map(train, parameter_list)
p.close()
p.join()

print "\nTraining done.\n"


print "Classifying testing data."

# Allocate memory for testing data classifications.
review_list = [["unknown", i] for i in range(num_test_data)]

# Reset counter.
progress.value = 1

# Classify.
def classify(review):
    review_idx = review[-1]

    # Use logs so we're not comparing zeros.
    score_pos = log(prob_review_pos)
    score_neg = log(1-prob_review_pos)

    # Consider each word.
    for i in range(num_vocab):
        if test_data[review_idx][i] == '1':   # If the word is in the review (i.e., feature = 1)..
            score_pos += log(parameter_list[i][3] + e) - 1.0
            score_neg += log(parameter_list[i][2] + e) - 1.0

    review.pop()   # Remove the index element.

    if score_pos > score_neg:
        review[0] = "pos"
    else:
        review[0] = "neg"

    sys.stdout.flush()
    sys.stdout.write("Progress: %3d/%3d\r" % (progress.value, num_test_data))
    with lock:
        progress.value += 1
    return review

p = multiprocessing.Pool(NUM_PROC)
review_list = p.map(classify, review_list)
p.close()
p.join()

print "\nClassification done.\n"

# Calculate accuracy.
pos_accuracy = 0.0
neg_accuracy = 0.0
for i in range(num_test_data):
    if review_list[i][0] == test_label[i] == "pos":
        pos_accuracy += 1.0
    elif review_list[i][0] == test_label[i] == "neg":
        neg_accuracy += 1.0
pos_accuracy /= num_test_pos
neg_accuracy /= num_test_neg

print "Positive accuracy:", pos_accuracy
print "Negative accuracy:", neg_accuracy


exit(0)

