"""A class for handling comparison and the IDS"""
import math

PLAUSABILITY_RANGE = 500
MAX_SPEED = 30
MAX_ACC = 5  #Maximum Acceleration
MAX_DEC = -15 #Maximum Deacceleration

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
    v_diff = new.get("speed") - old.get("speed")

    if v_diff < 0:
        max_dec , max_acc = max_acc, max_dec

    t_acc = ((-max_dec) * time + v_diff) / (-(max_dec) + max_acc)
    t_dec = time - t_acc

    v_tmp_acc = old.get("speed") + max_acc * t_acc
    v_tmp_dec = old.get("speed") + max_dec * t_dec

    bound_upper = ((old.get("speed") * t_acc + max_acc * pow(t_acc, 2)) +
                    (v_tmp_acc * t_dec + max_dec * pow(t_dec, 2)))
    bound_lower = ((old.get("speed") * t_dec + max_dec * pow(t_dec,2)) +
                    (v_tmp_dec * t_acc + max_acc * pow(t_acc, 2)))

    if v_diff < 0:
        bound_lower, bound_upper = bound_upper, bound_lower


    dist = distance(old.get("pos"), new.get("pos"))

    print(f"Lower: {bound_lower}, Upper: {bound_upper}, Dist: {dist}")

    #Might want to add a GPS uncertainty factor
    return bound_lower < dist < bound_upper

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
    return math.sqrt(pow((pos2[0] - pos1[0]),2) + pow((pos2[1]-pos1[1]),2))

#Add a main method, it should be running and taking messages and each message should be compared in
# the car_ids method
def main():
    """A main method where currently testing resides"""
    old = {"speed":15, "pos":(10,0)}
    new = {"speed":20, "pos":(30,25)}

    #Test 1
    test1 = position_consistency(old, new, 2, MAX_ACC, MAX_DEC) #15 -> 20
    #Test2
    test2 = position_consistency(new, old, 2, MAX_ACC, MAX_DEC) #20 -> 15

    print(f"Test 1: {test1}, Test 2: {test2}")

if __name__ == '__main__':
    main()
