#!/usr/bin/env python

import sys
import os

# Check if an argument (timestamp X) is provided
if len(sys.argv) != 2:
    print("Usage: {} <timestamp>".format(sys.argv[0]))
    sys.exit(1)

# Extract the timestamp from the command line argument
timestamp = sys.argv[1]

# Define the file names based on the timestamp
misbehaving_reporter_file = "misbehaving_reporter_{}.csv".format(timestamp)
received_reporter_file = "received_reporter_{}.csv".format(timestamp)

# Check if the files exist
if not (os.path.isfile(misbehaving_reporter_file) and os.path.isfile(received_reporter_file)):
    print("One or both of the specified files do not exist.")
    sys.exit(1)

# Execute the grep commands
with open(misbehaving_reporter_file, 'r') as misbehaving_file, open(received_reporter_file, 'r') as received_file:
    total_flagged = sum(1 for line in misbehaving_file)
    true_positives = sum(1 for line in misbehaving_file if line.startswith('0'))
    total_attacker_messages = sum(1 for line in received_file if line.startswith('0'))
    total_messages = sum(1 for line in received_file)

print("Total flagged: {}".format(total_flagged))
print("True Positives: {}".format(true_positives))
print("Total attacker messages: {}".format(total_attacker_messages))
print("Total messages: {}".format(total_messages))
