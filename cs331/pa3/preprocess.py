#!/usr/bin/env python

import csv
import string

# Read in files. Note that csv.reader will read in each line as a list of words.
raw_vocab = [word for line in list(csv.reader(open('files/raw.vocabulary.txt', 'rU'), delimiter=',', quotechar='"')) for word in line]   # 47525 words
stoplist  = [word for line in list(csv.reader(open('files/stoplist.txt', 'rU'),       delimiter=',', quotechar='"')) for word in line]
raw_train_unflat = [line for line in list(csv.reader(open('files/raw.train.txt', 'rU'),      delimiter=' ', quotechar='"'))]
raw_train = [word for word in line]
raw_test  = [word for line in list(csv.reader(open('files/raw.test.txt', 'rU'),       delimiter=' ', quotechar='"')) for word in line]

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

# Featurize training data.
num_clean_vocab   = len(clean_vocab)
num_train_reviews = len(raw_train_unflat)

print num_clean_vocab, "in vocabulary after removing uncommon words."

# Allocate meomory.
feature_list = [[0] * num_clean_vocab] * num_train_reviews


out_file = open('training.txt', 'wb')
writer = csv.writer(out_file, delimiter=',', quotechar='"')
writer.writerow(clean_vocab)
writer.writerows(feature_list)
out_file.close()

# vim: expandtab

