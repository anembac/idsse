"""A class responsible for handling the message clusters receieved from vehicles
 and analyzing them to notice misbehavior"""

import math
import csv
import os
import sys

LOWER_BOUND = 0
UPPER_BOUND = 75
#A structure for adding all messages, the structure works as the follwing
# {msg1_fingerprint : [msg1_car1, msg1_car2, msg1_car3], msg2_fingerprint : [msg2_car1, msg2_car2]}
messages = {}
misbehaving = []

def misbehaving_msgs():
    """Function responsible for going through all messages and adding misbehaving ones to a list"""
    for key in messages:
        if detect_misbehavior(messages[key]):
            misbehaving.append(key)

def detect_misbehavior(msg_set):
    """Function for determining if a message is suspicious"""
    for msg in msg_set:
        dist = distance([float(msg.get("receiveXPosCoords")), float(msg.get("receiveYPosCoords"))],
                         [float(msg.get("xposCoords")),float(msg.get("yposCoords"))])
        transfer_time = int(msg.get("receiveTime")) - int(msg.get("genDeltaTime"))
        print(dist)
        print(transfer_time)
        if LOWER_BOUND <= dist/transfer_time <= UPPER_BOUND: # transfer time is too short/inconsistent to be used this way
            continue
        return True
    return False



def distance (pos1, pos2):
    """Returns the distance between two positions"""
    return math.sqrt(pow((pos2[0] - pos1[0]),2) + pow((pos2[1]-pos1[1]),2))

def collect_messages(collection):
    """Add things here"""
    for msg in collection:
        fingerprint = msg.get("fingerprint")
        if messages.get(fingerprint):
            messages[fingerprint].append(msg)
        else:
            messages.update({fingerprint : [msg]})
        msg.get("fingerprint")

def read_csv(filename):    
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            messages.setdefault(row["fingerprint"],[]).append(row)

def load_data(directory):    
    for f in os.listdir(directory):
        if f.endswith('.csv'):
            file_path = os.path.join(directory, f)
            read_csv(file_path)
    
def main(args):
    """A main method responsible for handling Intrusion Detection on network level"""
    directory = args[1]
    load_data(directory) # populates messages
    misbehaving_msgs()
    print(misbehaving)

if __name__ == '__main__':
    main(sys.argv)
