"""A code example of how the deciding of routes could be handled, probably want this in C++"""

#Defines for route lenght and max speed on them
SHORT_ROUTE = 598.34
LONG_ROUTE = 698.26
MAX_SPEED = 20

#Might want to remove some of them since cars at the end of each route is not interesting
edges_short = ["main1"]
edges_long = ["side1", "side2", "side3"]

#Default route is short route, so this function returns a boolean of whether the car should
# continue of the route or switch to the side road due to congestion. Latest is a list of all the
# latest messages, one from each car where the car is the key. 
def continue_on_route (latest, short_speed, long_speed):
    """Return boolean if it should continue on the route"""
    for key in latest:
        if key.get("edge") in edges_short and key.get("speed") < short_speed:
            short_speed = key.get("speed")
        if key.get("edge") in edges_long and key.get("speed") < long_speed:
            long_speed = key.get("speed")

    return short_speed * SHORT_ROUTE < long_speed * LONG_ROUTE
