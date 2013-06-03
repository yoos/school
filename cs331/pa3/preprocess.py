#!/usr/bin/env python

import csv
import string

# Read in files. Note that csv.reader will read in each line as a list of words.
raw_vocab = [word for line in list(csv.reader(open('files/raw.vocabulary.txt', 'rU'), delimiter=',', quotechar='"')) for word in line]   # 47525 words
stoplist  = [word for line in list(csv.reader(open('files/stoplist.txt', 'rU'),       delimiter=',', quotechar='"')) for word in line]
raw_train = [word for line in list(csv.reader(open('files/raw.train.txt', 'rU'),      delimiter=' ', quotechar='"')) for word in line]
raw_test  = [word for line in list(csv.reader(open('files/raw.test.txt', 'rU'),       delimiter=' ', quotechar='"')) for word in line]

# Remove stopwords from raw_vocab
raw_vocab = [word for word in raw_vocab if word not in stoplist]   # 47026 words

# Convert list to dict.
raw_vocab_dict = dict([(word, 0) for word in raw_vocab])

# Count frequency of usage.
for word in raw_train and raw_test:
    try:
        raw_vocab_dict[word] += 1
    except KeyError:
        pass

# Remove uncommon (i.e., used <= 3 times) words. len(raw_vocab_dict) should be 5864 after this.
for word in raw_vocab:
    if raw_vocab_dict[word] <= 3:
        del raw_vocab_dict[word]



# vim: expandtab

