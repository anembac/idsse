"""A class responsible for handling the message clusters receieved from vehicles
 and analyzing them to notice misbehavior"""

import math
import csv
import os
import sys
import time
LOWER_BOUND = 0
UPPER_BOUND = 75

#A structure for adding all messages, the structure works as the follwing
# {msg1_fingerprint : [msg1_car1, msg1_car2, msg1_car3], msg2_fingerprint : [msg2_car1, msg2_car2]}
messages = {}
misbehaving = []
regularmessage = []

def misbehaving_msgs():
    """Function responsible for going through all messages and adding misbehaving ones to a list"""
    for key in messages:
        t_1 = time.time()
        bad_msg  = detect_misbehavior(messages[key])
        t_diff = time.time() - t_1
        total_time = t_diff + calc_collection_time(messages[key]) + calc_car_ids(messages[key])
        if bad_msg:
            misbehaving.append((key,total_time))
        else:
            regularmessage.append((key,total_time))

def calc_collection_time(msg_set):
    highest_time = 0
    lowest_time = 9999999999999
    for msg in msg_set:
        t_receive = int(msg.get("receiveTime"))
        if  t_receive < lowest_time:
            lowest_time = t_receive
        elif t_receive > highest_time:
            highest_time = t_receive

def calc_car_ids(msg_set):
    car_ids_time = 99999999999999 #We need it to take first value at least...
    for msg in msg_set:
        if msg.get("idsTime") < car_ids_time:
            car_ids_time = msg.get("idsTime")
    return car_ids_time


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
