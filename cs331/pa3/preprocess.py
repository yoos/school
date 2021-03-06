#!/usr/bin/env python

import sys
import csv
import string
import multiprocessing

### CONSTANTS ###
NUM_PROC = 10

# Read in files. Note that csv.reader will read in each line as a list of words.
raw_vocab = [word for line in list(csv.reader(open('files/raw.vocabulary.txt', 'rU'), delimiter=',', quotechar='"')) for word in line]   # 47525 words
stoplist  = [word for line in list(csv.reader(open('files/stoplist.txt', 'rU'),       delimiter=',', quotechar='"')) for word in line]
raw_train_unflat = [line for line in list(csv.reader(open('files/raw.train.txt', 'rU'), delimiter=' ', quotechar='"'))]
raw_train        = [word for line in raw_train_unflat for word in line]
raw_test_unflat  = [line for line in list(csv.reader(open('files/raw.test.txt', 'rU'), delimiter=' ', quotechar='"'))]
raw_test         = [word for line in raw_test_unflat for word in line]

# Remove stopwords from raw_vocab
print len(raw_vocab), "in raw vocabulary."
raw_vocab = [word for word in raw_vocab if word not in stoplist]   # 47026 words
print len(raw_vocab), "in vocabulary after removing stopwords."

# Convert list to dict.
raw_vocab_dict = dict([(word, 0) for word in raw_vocab])

# Count frequency of usage.
for word in raw_train:
    try: raw_vocab_dict[word] += 1
    except KeyError: pass
for word in raw_test:
    try: raw_vocab_dict[word] += 1
    except KeyError: pass

# Remove uncommon (i.e., used <= 3 times) words.
for word in raw_vocab:
    if raw_vocab_dict[word] <= 3:
        del raw_vocab_dict[word]

# Make clean vocab list in alphabetical order (raw_vocab is already in alphabetical order).
clean_vocab = [word for word in raw_vocab if word in raw_vocab_dict]

# Count some stuff.
num_clean_vocab   = len(clean_vocab)
num_train_reviews = len(raw_train_unflat)
num_test_reviews  = len(raw_test_unflat)

print num_clean_vocab, "in vocabulary after removing uncommon words.\n"


print "Featurizing training data."

# Allocate meomory.
feature_list = [[0] * num_clean_vocab + [i] for i in range(num_train_reviews)]   # Keep index element at end of each feature list.

# Synchronized counter used to track progress.
progress = multiprocessing.Value('i', 1)
lock = multiprocessing.Lock()

# Featurize training data.
def featurize(feature):
    review_num = feature[-1]

    for word in raw_train_unflat[review_num]:
        if word in clean_vocab:
            feature[clean_vocab.index(word)] = 1

    feature.pop()   # Remove index element.

    with lock:
        sys.stdout.flush()
        sys.stdout.write("Progress: %4d/%4d\r" % (progress.value, num_train_reviews))
        progress.value += 1
    return feature

p = multiprocessing.Pool(NUM_PROC)
feature_list = p.map(featurize, feature_list)
p.close()
p.join()

# Write to file.
out_file = open('training.txt', 'wb')
writer = csv.writer(out_file, delimiter=',', quotechar='"')
writer.writerow(clean_vocab)
writer.writerows(feature_list)
out_file.close()

print "\nDone featurizing training data.\n"


print "Featurizing testing data."

# Allocate meomory.
feature_list = [[0] * num_clean_vocab + [i] for i in range(num_test_reviews)]   # Keep index element at end of each feature list.

# Reset counter.
progress.value = 1

# Featurize testing data.
def featurize(feature):
    review_num = feature[-1]

    for word in raw_test_unflat[review_num]:
        if word in clean_vocab:
            feature[clean_vocab.index(word)] = 1

    feature.pop()   # Remove index element.

    with lock:
        sys.stdout.flush()
        sys.stdout.write("Progress: %3d/%3d\r" % (progress.value, num_test_reviews))
        progress.value += 1
    return feature

p = multiprocessing.Pool(NUM_PROC)
feature_list = p.map(featurize, feature_list)
p.close()
p.join()

print "\nDone featurizing testing data.\n"


# Write to file.
out_file = open('testing.txt', 'wb')
writer = csv.writer(out_file, delimiter=',', quotechar='"')
writer.writerow(clean_vocab)
writer.writerows(feature_list)
out_file.close()

# vim: expandtab

