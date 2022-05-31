#!/usr/bin/env python3
import numpy as np
import rospy
import tf
import math
from heapq import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
import matplotlib.pyplot as plt 

map = np.array([
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1], #9
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1], #8
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1], #7
       [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1], #6
       [0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1], #5
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0,1], #4
       [0,0,1,1,0,0,1,1,1,1,1,1,0,0,0,0,0,0,1], #3
       [0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,1], #2
       [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1,1], #1
       [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,1], #0
       [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,1], #-1
       [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,1], #-2
       [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,1], #-3
       [0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1], #-4
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1], #-5
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1], #-6
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,1], #-7
       [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0,1], #-8
       [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,1], #-9 
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1,1]]) #-10

def euclid_dis(pt1,pt2):
    dis = math.sqrt(((pt2[0]-pt1[0])**2 + (pt2[1]-pt1[1])**2))
    return dis

def get_error(path,next_Ind,robot_posx,robot_posy,robot_orient):
    next_x=path[next_Ind][0]-9+0.9
    next_y=10.5-path[next_Ind][1]-1
    diff_x = next_x-robot_posx
    diff_y = next_y-robot_posy
    angle=math.atan2(diff_y,diff_x)
    #print(angle)
    error = angle - robot_orient
    return error
def get_next_x(path,next_Ind):
    next_x=path[next_Ind][0]-9+0.9
    return next_x

def get_next_y(path,next_Ind):
    next_y = 10.5-path[next_Ind][1]-1
    return next_y
class main():
    def get_param(self):
        x_param , y_param = rospy.search_param('goalx') , rospy.search_param('goaly')
        goalx , goaly = rospy.get_param(x_param) , rospy.get_param(y_param)
        goalx_val , goaly_val = int(goalx) , int(goaly)
        return (goalx_val,goaly_val)

    orientation = None
    position = None
    
    def _path_plan(self,path_fin,args_for_path_plan):
        if (complete == False):
            tw = Twist()
            trans , rot =path_fin.pose.pose.position , path_fin.pose.pose.orientation
            robot_orient=tf.transformations.euler_from_quaternion([rot.x,rot.y,rot.z,rot.w])[2]
            robot_posx , robot_posy  =trans.x , trans.y

            pub = args_for_path_plan['pub']
            path=args_for_path_plan['a_star_path']
            next_Ind=args_for_path_plan['next_Ind']
            

            if round(abs(robot_posx - goalx),1) <= 0.6 :
                if round(abs(goaly  -robot_posy),1) <= 0.8:

                    args_for_path_plan["complete"]=True
                    args_for_path_plan["rot"]=False
                    reached()
                    
            if round(abs(robot_posx - goalx),1) > 0.6  :
                next_x=get_next_x(path,next_Ind)
                next_y=get_next_y(path,next_Ind)

                diff_x = next_x-robot_posx
                diff_y = next_y-robot_posy

                error = get_error(path,next_Ind,robot_posx,robot_posy,robot_orient)

            if round(abs(goaly  -robot_posy),1) > 0.8 :
                next_x=get_next_x(path,next_Ind)
                next_y=get_next_y(path,next_Ind)

                diff_x = next_x-robot_posx
                diff_y = next_y-robot_posy

                error = get_error(path,next_Ind,robot_posx,robot_posy,robot_orient)

            if args_for_path_plan['rot']:
                if math.fabs(error) <= 0.1:
                    args_for_path_plan['rot']=False
                elif math.fabs(error) > 0.1:
                    if error < 0:
                        error += 6
                    elif error > 6:
                        error -= 6
                    if error > 6:
                        tw.angular.z = -0.05
                        pub.publish(tw)        
                    elif error <= 6:
                        tw.angular.z = 0.5
                        pub.publish(tw)
              
            elif args_for_path_plan['rot'] == False:
                error=math.sqrt((diff_x)**2+(diff_y)**2)

                if error <= 0.5:
                    args_for_path_plan['rot']=True
                    if next_Ind+1<len(path):
                        args_for_path_plan['next_Ind']+=1
                    elif next_Ind+1<len(path) :
                        args_for_path_plan['complete']=True
                elif error> 0.5:
                    tw.linear.x = 0.5
                    pub.publish(tw)

def astar_planning(array, start_loc, goal_loc,rot):
    if rot != False:
        l1,trav,l2 = set(),{},[]
        goal = euclid_dis(start_loc, goal_loc)

        g_func,f_func = {start_loc:0},{start_loc: goal}
        heappush(l2, (f_func[start_loc], start_loc))

        while len(l2) > 0:
            current = heappop(l2)[1]

            if current == goal_loc:
                old_path = []
                final_path = []

                while current in trav:
                    old_path.append(current)
                    current = trav[current]
                old_path.reverse()
                counter = 0

                while counter < len(old_path):
                    y = math.floor(old_path[counter][1])
                    x = math.floor(old_path[counter][0])
                    final_path.append((x,y))
                    counter +=1
                print((final_path))

                map_path = map
                for coord in final_path:
                    map_path[int(coord[1]),int(coord[0])] = 100
                map2 = np.array2string(map_path, precision=0, separator='',suppress_small=True).replace('100',' *').replace('0',' ').replace('[','').replace(']','')
                print(map2)
                print(" ################################################# ")
                print(map_path)
                return final_path

            l1.add(current)
            x,y = current
            l = []
            R = [0,1,-1]
        
            for a in R:
                for b in R:
                    new = (a,b)
                    if (new[0],new[1]) != (0,0):
                        l.append(new)

            for m in l:
                a = m[0]
                b = m[1]
                near_cells = x + int(a), y + int(b)
                new_g_func = g_func[current] + euclid_dis(current, near_cells)
                if(0 <= near_cells[0] < array.shape[1]):
                    x1 = True
                else:
                    x1 = False
                if(0 <= near_cells[1] < array.shape[0]):
                    x2 = True
                else:
                    x2 = False
                x3 = array[near_cells[1]][near_cells[0]]
                if x1:
                    if x2:
                        if x3:
                            continue
                elif (x1 == 0):
                    continue
                elif x1:
                    if (x2 == 0):
                        continue

                if near_cells in l1:
                    if new_g_func >= g_func.get(near_cells, 0):
                        continue

                new = [a[1]for a in l2]
                if  new_g_func < g_func.get(near_cells, 0) :
                    trav[near_cells] = current
                    g_func[near_cells] = new_g_func
                    f_func[near_cells] = new_g_func + euclid_dis(near_cells, goal_loc)
                    heappush(l2, (f_func[near_cells], near_cells))

                if   near_cells not in new:
                    trav[near_cells] = current
                    g_func[near_cells] = new_g_func
                    f_func[near_cells] = new_g_func + euclid_dis(near_cells, goal_loc)
                    heappush(l2, (f_func[near_cells], near_cells))

def reached():
    complete =  True
    rot = False
    print("Goal reached")
    plt_map= np.transpose(map)
    plt.plot(np.where(plt_map == 1)[0],19 - np.where(plt_map == 1)[1], 'bo')
    plt_path = np.array(a_star_path)
    plt.plot(plt_path[:, 0], 19 - plt_path[:, 1], 'rx')
    plt.show()

    rospy.spin()

    
if __name__ == '__main__':
    rospy.init_node("a_path_planning", anonymous=False)
    robot=main()
    goalx = rospy.get_param("/goalx")
    goaly = rospy.get_param("/goaly")
    start_loc,goal_loc = (1,12),(int(goalx+8.5),int(10-goaly))
    args_for_path_plan = {}

    complete = False
    rot = True
    
    next_Ind = 0
    astar_path = {}

    a_star_path = astar_planning(map, start_loc,goal_loc,rot)
    pub  = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

    # print(map)
    
    args_for_path_plan['rot'] = rot
    args_for_path_plan['pub'] = pub
    args_for_path_plan['a_star_path'] = a_star_path
    args_for_path_plan['next_Ind'] = next_Ind
    args_for_path_plan['complete'] = complete



    robot_pos_pub = rospy.Subscriber("/base_pose_ground_truth", Odometry,robot._path_plan,args_for_path_plan)
    while not rospy.is_shutdown():
        if complete:
            if rospy.has_param("/goalx") and rospy.has_param("/goaly"):
                    goalx = rospy.get_param("/goalx")
                    goaly = rospy.get_param("/goaly")
                    rospy.delete_param("/goalx") , rospy.delete_param("/goaly")
                    new_initial = goal_loc
                    goal_loc = (round(goalx+9),round(10-goaly))
                    a_star_path = astar_planning(map, new_initial, goal_loc,rot)
                    #print(a_star_path)
                    rot = True
                    complete = False
                    next_Ind = 0
 
        rate = rospy.Rate(10)
        rospy.sleep(1)