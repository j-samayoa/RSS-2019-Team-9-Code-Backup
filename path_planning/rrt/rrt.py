import rospy
import numpy as np
from geometry_msgs.msg import Point
import random
import math
from math import atan2,sin,cos

x_dim = 87
y_dim = 65
count = 0
RADIUS =5
delta = 2
node_limit = 1000



class Node():
    """
    Node class, will represent pixels in the image
    Each node has:
    -a "point obeject", a tuple representing x and y values      
    -a parent node, which it will connect to
    """
    def __init__(self, point, parent):
        self.point = point
        self.parent = parent

    def dist(self,other):
        """
        Euclidean distance to another node

        Returns:
            Float representing distance
        """
        #print(self.point)
        #print(self.point.y)
        #print(type(self.point.y))
        #print("OTHER")
        #print(other.point)
        return math.sqrt((self.point.x - other.point.x)**2+(self.point.y - other.point.y)**2)

    def collision(self,other,collision_dist):
        """
        A node collides with another node if the distance between them
        is less than collision_dist

        Returns:
            Boolean (True if collision, False otherwise)
        """
        return self.dist(other) <= collision_dist

    def step_from_to(self,other):
        """
        Steps from one node to another, creates a new node object if necessary

        Returns:
            A node object??? point??? node for now
        """

        if self.dist(other) < delta:
            return other
        else:#what does this do?
            theta = atan2(other.point.y-self.point.y,other.point.x-self.point.x)
            p = Point()
            p.x = self.point.x+delta*cos(theta)
            p.y = self.point.y+delta*sin(theta)
            return Node(p,None)

def node_collides(node):
    """
    Check if a node collides with its encoronment
    ??????
    """
    #raytracing will go here
    pass

def get_rand_node():
    p = Point()
    x = random.random()*x_dim-60.0
    y = random.random()*y_dim-16.9
    #collision here
    #if doesn't collide, return Point
    p.x = x
    p.y = y
    return Node(p,None)


def run(start,end):
    """
    runs rrt
    """

    global count

    print("Entering rrt run")
    start_node = Node(start,None)
    end_node = Node(end,None)
    state = "Build"
    nodes = [start_node]
    result  = []
    done = False

    while not done:
        if state == "Finished":
            current = end_node #???
            print("Done")
            while current.parent !=None:
                #draw lines here, final path
                result.append(current.point)
                current = current.parent

                

            done = True
        elif state == "Build":
            count = count+1
            print("Building tree.....")
            if count < node_limit:
                new_found = False
                while not new_found:
                    new_n =  get_rand_node()
                    parent = nodes[0]
                    for n in nodes:
                        if new_n.dist(n) <= new_n.dist(parent):
                            new_n = new_n.step_from_to(n)
                            #test collision
                            if True: #collision later
                                parent = n
                                new_found = True

                new_n.parent = parent
                nodes.append(new_n)
                #draw line here from parent point to new point

                if end_node.collision(new_n,RADIUS):
                    state = "Finished"
                    end_node = nodes[-1]

            else:
                print("Exceeded number of nodes")
    return result


#use point class from geometry_msgs, contains x, y, and z values
