"""A class responsible for handling the message clusters receieved from vehicles
 and analyzing them to notice misbehavior"""

import math

LOWER_BOUND = 0
UPPER_BOUND = 100
#A structure for adding all messages, the structure works as the follwing
# {msg1_fingerprint : [msg1_car1, msg1_car2, msg1_car3], msg2_fingerprint : [msg2_car1, msg2_car2]}
messages = {}
misbehaving = []

def misbehaving_msgs():
    """Function responsible for going through all messages and adding misbehaving ones to a list"""
    for key, value in messages:
        if detect_misbehavior(value):
            misbehaving.append(key)

def detect_misbehavior(msg_set):
    """Function for determining if a message is suspicious"""
    for msg in msg_set:
        dist = distance(msg.get("receiver_position"), msg.get("sender_position"))
        transfer_time = msg.get("msg_arrival_time") - msg.get("timestamp")
        if LOWER_BOUND <= dist/transfer_time <= UPPER_BOUND:
            continue
        return True
    return False



def distance (pos1, pos2):
    """Returns the distance between two posistion"""
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

def main():
    """A main method responsible for handling Intrusion Detection on network level"""


if __name__ == '__main__':
    main()
