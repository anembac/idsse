#!/usr/bin/env python

import csv
import sys

messages = {}
misbehaving = []
behaving = []
timings = []
TP = 0
TN = 0
FP = 0
FN = 0
total_attacking = 0
total_flagged = 0
total_messages = 0

def read_csv(filename):
    """Read the contents of a .csv file"""    
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            timings.append(int(row.get("idsTime"))/1000000)
            total_messages += 1
            if row.get("attacking") == "1":
                total_attacking += 1
                if row.get("flagged") == "1":
                    total_flagged += 1
                    TP += 1
                else:
                    FN += 1
            else:
                if row.get("flagged") == "1":
                    total_flagged += 1
                    FP += 1
                else:
                    TN += 1


def load_data(timestamp):
    """Load data from a dictionary"""    
    # Define the file names based on the timestamp
    received_reporter_file = "received_reporter_{}.csv".format(timestamp)
    read_csv(received_reporter_file)

def main(args):
    """A main method responsible for handling Intrusion Detection on network level"""
    timestamp = args[1]
    load_data(timestamp) # populate messages
    timings.sort()
    print(f"Best detection latency: {timings[0]}")
    print(f"Worst detection latency: {timings[-1]}")
    print(f"Average detection latency: {sum(timings)/len(timings)}")
    print("Total flagged: {}".format(total_flagged))
    recall = TP/(TP+FN)
    precision = TP/(TP+FP)
    accuracy = (TP+TN)/(TP+FP+TN+FN)
    print(f"TP: {TP}")
    print(f"TN: {TN}")
    print(f"FP: {FP}")
    print(f"FN: {FN}")
    print(f"Recall       {recall}")
    print(f"Precision    {precision}")
    print(f"Accuracy     {accuracy}")
    print(f"F1Score      {(2*recall*precision)/(recall+precision)}")
    print("Total attacker messages: {}".format(total_attacking))
    print("Total messages: {}".format(total_messages))


if __name__ == '__main__':
    main(sys.argv)