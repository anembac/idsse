"""A class for handling comparison and the IDS"""
import math

PLAUSABILITY_RANGE = 500
MAX_SPEED = 30
MAX_ACC = 5  #Maximum Acceleration
MAX_DEC = -5 #Maximum Deacceleration

#A stack containing all messages related to a car
#Will this look like this, might just be multiple functions that are getters for sensor values
mycar_sensors = {}
#This should probably be a dictionary of stacks, the id would be the car-id, since these are unique
# this might however change some of the code slightly so it's not done yet.
msg_stack = []

def car_ids (msg_latest):
    """Returns whether a msg from a car is deemed as misbehaving"""
    if len(msg_stack) > 0:
        msg_prev = msg_stack.peek()
        misbehaved = cmp_all(msg_prev, msg_latest)
    else:
        #Only one message exist, can't compare no misbehavior
        return False
    msg_stack.append(msg_latest)
    return misbehaved

#Compares the old and new message, if a check fails it will return true, otherwise false
def cmp_all(old, new):
    """Compares the old msg from a car with the latest one and return whether it seems consistent"""
    t_diff = new.timestamp - old.timestamp
    if(all([range_plausability(old.pos),                               #Range Plausability
           position_plausability(),                                    #Position Plausability
           speed_plausability(new),                                    #Speed Plausability
           position_consistency(old, new, t_diff, MAX_ACC, MAX_DEC),   #Posisiton Consistency
           speed_consistency(old.speed-new.speed, t_diff),             #Speed Consistency
           pos_speed_consistency(),                                    #Position speed consistency
           pos_heading_consistency()])):                               #Position heading consistancy
        return False
    return True


def range_plausability(car_position):
    """Whether the car is within allowed sending range"""
    return distance(mycar_sensors.get('position'), car_position) < PLAUSABILITY_RANGE


#Might not be relevant in the current implementatin, this makes sure that the car is actually
# on the road or similar and not driving in weird places.
def position_plausability():
    """No implementation yet"""
    return True

def speed_plausability(msg):
    """Return a bool for whether the speed is higher than maximum allowed speed """
    return msg.speed <= MAX_SPEED

#The distance between car positions should be reasonable when taking into account their speed
# checks whether the distance is within an upper and lower bound from speed, acceleration and
# deacceleration.
def position_consistency(old, new, time, max_acc, max_dec):
    """Returns a bool whether a car's position is consistant with it's speed"""
    v_diff = new.speed - old.speed

    if v_diff < 0:
        max_dec , max_acc = max_acc, max_dec

    t_acc = v_diff / max_acc
    t_remaining = time - t_acc

    dist = old.speed * t_acc + max_acc * t_acc * t_acc

    t_acc = -(max_dec) * t_remaining /(-(max_dec) + max_acc)
    t_dec = t_remaining - t_acc

    v_tmp = old.speed + max_dec * t_dec

    bound_upper =  dist + new.speed * t_remaining
    bound_lower = (dist + (old.speed * t_dec + max_dec * t_dec * t_dec) +
                   (v_tmp * t_acc + max_acc * t_acc * t_acc))

    if v_diff < 0:
        bound_lower, bound_upper = bound_upper, bound_lower

    return bound_lower < distance(old.pos, new.pos) < bound_upper

#Speed diff is what you get when putting (old.speed - new.speed), this should be kept under
# the MAX_ACCELERATION and over the MAX_DEACCELERATION.
def speed_consistency(speed_diff, time):
    """Returns the"""
    return MAX_DEC * time < speed_diff < MAX_ACC * time

def pos_speed_consistency():
    """Not yet implemented"""
    return True

def pos_heading_consistency():
    """Not yet implemented"""
    return True

def distance (pos1, pos2):
    """Returns the distance between two posistion"""
    return math.sqrt(pow((pos2.x - pos1.x),2) + pow((pos2.y-pos1.y),2))


#Add a main method, it should be running and taking messages and each message should be compared in
# the car_ids method
