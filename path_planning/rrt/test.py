from rrt import *
from geometry_msgs.msg import Point

p1 = Point()
p1.x = 0
p1.y = 0

p2 = Point()
p2.x = 20
p2.y = 20


print("Testing rrt")
run(p1,p2)
print("test successful")



