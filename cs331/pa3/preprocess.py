#!/usr/bin/env python

import csv
import string

# Read in files. Note that csv.reader will read in each line as a list of words.
raw_vocab = [word[0] for word in list(csv.reader(open('files/raw.vocabulary.txt', 'rU'), delimiter=',', quotechar='"')) if len(word) > 0]   # 47525 words
stoplist  = [word[0] for word in list(csv.reader(open('files/stoplist.txt', 'rU'),       delimiter=',', quotechar='"')) if len(word) > 0]

# Remove stopwords from raw_vocab
raw_vocab = [word for word in raw_vocab if word not in stoplist]   # 47026 words

# Convert list to dict.
raw_vocab_dict = dict([(word, 0) for word in raw_vocab])



# vim: expandtab

