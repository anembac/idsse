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
    for key, value in messages:
        t_1 = time.time()
        misbehavior  = detect_misbehavior(value)
        t_diff = time.time() - t_1
        total_time = (t_diff +
                      calc_collection_time(value) +
                      calc_car_ids(value) +
                      2 * calc_transmission_time(value)) #Needs two of these since it's to and from
        if misbehavior:
            misbehaving.append((key,total_time))
        else:
            regularmessage.append((key,total_time))

def calc_collection_time(msg_set):
    """"Calculating the time it took to collect all the messages"""
    highest_time = 0
    lowest_time = 9999999999999
    for msg in msg_set:
        t_receive = int(msg.get("receiveTime"))
        lowest_time  =  min(lowest_time,  t_receive)
        highest_time =  max(highest_time, t_receive)
    #print((highest_time-lowest_time)/1000)
    return (highest_time-lowest_time)/1000 #So the time is in seconds

def calc_car_ids(msg_set):
    """Calculate the best time it took for one of the cars to use CAR IDS"""
    car_ids_time = 99999999999999 #We need it to take first value at least...
    for msg in msg_set:
        idst = int(msg.get("idsTime"))
        car_ids_time = min(car_ids_time,idst)
    #print(car_ids_time/1000)
    return car_ids_time/1000 #So the time is in seconds

def calc_transmission_time(msg_set):
    """TEST"""
    transmission_time = 99999999999999
    for msg in msg_set:
        cmp_t_time = int(msg.get("receiveTime"))-int(msg.get("genDeltaTime"))
        transmission_time = min(transmission_time,cmp_t_time)
    #print(transmission_time/1000)
    return transmission_time/1000 #So the time is in seconds


def detect_misbehavior(msg_set):
    """Function for determining if a message is suspicious"""
    for msg in msg_set:
        dist = distance([float(msg.get("receiveXPosCoords")), float(msg.get("receiveYPosCoords"))],
                         [float(msg.get("xposCoords")),float(msg.get("yposCoords"))])
        transfer_time = int(msg.get("receiveTime")) - int(msg.get("genDeltaTime"))
        if LOWER_BOUND <= dist/transfer_time <= UPPER_BOUND: #transfer time too short for this
            continue
        return True
    return False



def distance (pos1, pos2):
    """Returns the distance between two positions"""
    return math.sqrt(pow((pos2[0] - pos1[0]),2) + pow((pos2[1]-pos1[1]),2))

###This code is not used anymore I think
#def collect_messages(collection):
#    """Add things here"""
#    for msg in collection:
#        fingerprint = msg.get("fingerprint")
#        if messages.get(fingerprint):
#            messages[fingerprint].append(msg)
#        else:
#            messages.update({fingerprint : [msg]})
#        msg.get("fingerprint")


def read_csv(filename):
    """Read the cotents of a .csv file"""    
    with open(filename, 'r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            messages.setdefault(row["fingerprint"],[]).append(row)

def load_data(directory):
    """Load data from a dictionary"""    
    for f in os.listdir(directory):
        if f.endswith('.csv'):
            file_path = os.path.join(directory, f)
            read_csv(file_path)

def main(args):
    """A main method responsible for handling Intrusion Detection on network level"""
    directory = args[1]
    load_data(directory) # populates messages
    misbehaving_msgs()
    # print("Misbehaving: {}".format(misbehaving))
    # print("Behaving: {}".format(regularmessage))

if __name__ == '__main__':
    main(sys.argv)
