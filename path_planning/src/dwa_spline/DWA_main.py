import pygame 
import random
from webcolors import name_to_rgb
import math
import copy
import time
from scipy.interpolate import BSpline, make_interp_spline,CubicSpline
import numpy as np
import statistics
from occupancy2coords import *
from rrt_planner import rrt_planner


coordinates,grid = convert_ocgrid2coord()
norm,w,h,lim = normal(coordinates)
coordinates = transform_coordinates(coordinates,w,h)


def main(start,goal,method):
    pygame.init()

    # Units are in metres and radians
    obs_radius = 0.3
    robot_radius = 0.1

    robot_width = 2*robot_radius
    safedistance = robot_radius 

    max_vel = 0.5
    max_accel = 0.5
    
    '''Add dynamic obstacles if needed'''
    obs_vel_range = 0.15   # use the obs_move function

    display_bound = (-5.0,-5.0,5.0,5.0)
    # display_bound = (lim[1],lim[0],lim[3],lim[2])


    # Params for display
    render_scale = 2

    width = 1500
    height = 1500

    size = (width,height)

    # Screen center at (x,y) = (0,0)
    u0 = width/2
    v0 = height/2
   


    #pixels per metre for graphics
    k = 20
 

    # Initialise Pygame display screen
    screen = pygame.display.set_mode(size)
    # This makes the normal mouse pointer invisible in graphics window
    pygame.mouse.set_visible(0)
   


    '''START'''
    #Initialize states: x,y,theta
    
    x,y,theta = start[0],start[1],0


    '''Global Planner'''
    global_path = rrt_planner((x,y),goal,coordinates)
    Num_points = len(global_path)
  
    goal = global_path[1]  #first point
    goal_idx = 1
   
    location_history= []  #history of x and y

    # Initial velocity of left and right wheel
    vL = 0.0
    vR = 0.0

    #Params
    dt = 0.1  
    steps_ahead_to_plan = 20
    tau = dt * steps_ahead_to_plan

    #Obstacles
    obs = []  # contain x,y and visibility mask(2 values)
    # num_obstalces = 20
    
 
    num_obstacles = len(coordinates)

    for coord in coordinates:
        (ox,oy,vx,vy) = (coord[0],coord[1],random.gauss(0.0, obs_vel_range),random.gauss(0.0, obs_vel_range))
        ob = [ox,oy,vx,vy]
        obs.append(ob)

    

    def move_obs(obs,display_bound,dt):
        for i in range(len(obs)):
            obs[i][0] += obs[i][2] * dt
            if (obs[i][0] < display_bound[0]):
                obs[i][2] = -obs[i][2]
            if (obs[i][0] > display_bound[2]):
                 obs[i][2] = -obs[i][2]

            obs[i][1] += obs[i][3] * dt
            if (obs[i][1] < display_bound[1]):
                obs[i][3] = -obs[i][3]
            if (obs[i][1] > display_bound[3]):
                 obs[i][3] = -obs[i][3]


    def predict_next(vL,vR, x,y, theta, dt):
        
        if (round(vL,3) == round(vR,3)):
            # if moving in a straight line
            x_new = x + vL * math.cos(theta)*dt
            y_new = y * vL * math.sin(theta)*dt
            theta_new = theta 
            motion = (0,vL*dt) # 0 indicates pure translation, as it is

        elif (round(vL,3) == - round(vR,3)):
            # if pure rotation
            x_new = x 
            y_new = y
            theta_new = theta + ((vR-vL)*dt/robot_width)  # width to convert to radians
            motion = (1,0)  # 1 indicates pure rotation

        else: 
            R = robot_width/2.0 * ((vR+vL)/(vR-vL))
            d_theta = ((vR-vL)/robot_width) * dt
            x_new = x + R * (math.sin(d_theta + theta) - math.sin(theta))
            y_new = y - R * (math.cos(d_theta + theta) - math.cos(theta))
            theta_new = theta + d_theta

            # centre of circle - cx,cy
            (cx, cy) = (x - R * math.sin(theta), y + R * math.cos (theta))

            # Turn this into Rect
            Rabs = abs(R)
            ((tlx, tly), (Rx, Ry)) = ((int(u0 + k * (cx - Rabs)), int(v0 - k * (cy + Rabs))), (int(k * (2 * Rabs)), int(k * (2 * Rabs))))
            if (R > 0):
                    start_angle = theta - math.pi/2.0
            else:
                    start_angle = theta + math.pi/2.0
            stop_angle = start_angle + d_theta
            motion = (2, ((tlx, tly), (Rx, Ry)), start_angle, stop_angle) # 2 indicates general motion

        return (x_new, y_new, theta_new, motion)
    

    def dist_to_closest_obstacle(x,y,obs):
        dist2closest_obs = 10000000.0
        for i,ob in enumerate(obs):
            if ob != goal:
                dx = ob[0] - x
                dy = ob[1] - y
                d = math.sqrt(dx**2+dy**2)
                dist = d - (obs_radius+robot_radius)
                if dist < dist2closest_obs:
                    dist2closest_obs = dist
        return dist2closest_obs
    

    def dist_to_globe(x,y,obs):
        dist2globe = 1e6
        for i,ob in enumerate(obs):
            dx = ob[0] - x
            dy = ob[1] - y
            d = math.sqrt(dx**2+dy**2)
            dist = d - (obs_radius+robot_radius)
            if dist < dist2globe:
                dist2globe = dist
        return dist2globe
    

    def mean_dist_to_obstacles(x,y,obs):
        total = 0
        num =len(obs)
        for i,ob in enumerate(obs):
            if ob != goal:
                dx = ob[0] - x
                dy = ob[1] - y
                d = math.sqrt(dx**2+dy**2)
                dist = d - (obs_radius+robot_radius)
                total+=dist
        mean = total/num     
        return mean
    
  
    
    def generate_path(x_start, y_start, x_end, y_end, theta_end, num_points=100):
        if method == 'Cubic':
            path = []

            # Define control points
            control_points = np.array([
                [x_start, y_start],
                [(x_start + x_end) / 2, (y_start + y_end) / 2],
                [x_end, y_end]
            ])

            # Calculate the knot vector for the spline
            t_control = np.linspace(0, 1, control_points.shape[0])
            t_spline = np.linspace(0, 1, num_points)

            # Create the cubic spline
            spline_x = CubicSpline(t_control, control_points[:, 0], bc_type='natural')
            spline_y = CubicSpline(t_control, control_points[:, 1], bc_type='natural')

        
            curve_points_x = spline_x(t_spline)
            curve_points_y = spline_y(t_spline)
            curve_points = np.vstack((curve_points_x, curve_points_y)).T

            
            dtheta = theta_end
            theta_values = np.linspace(0, dtheta, num_points)

            # Combine the curve points and theta values
            for i in range(num_points):
                x, y = curve_points[i]
                theta = theta_values[i]
                path.append((x, y, theta))

            return path
        
        elif method == 'bspline':
            path = []

            # Define control points
            control_points = np.array([
                [x_start, y_start],
                [(x_start + x_end) / 2, (y_start + y_end) / 2],
                [x_end, y_end]
            ])

            # Degree of the B-spline curve
            degree = 2

            # Create the B-spline representation of the curve for x and y coordinates separately
            t_control = np.linspace(0, 1, control_points.shape[0])
            spline_x = make_interp_spline(t_control, control_points[:, 0], k=degree)
            spline_y = make_interp_spline(t_control, control_points[:, 1], k=degree)

            # Evaluate the B-spline at t_spline values
            t_spline = np.linspace(0, 1, num_points)
            x_values = spline_x(t_spline)
            y_values = spline_y(t_spline)

            # Calculate the linear interpolation of the angle
            dtheta = theta_end
            theta_values = np.linspace(0, dtheta, num_points)

            # Combine the x, y, and theta values
            for i in range(num_points):
                x, y, theta = x_values[i], y_values[i], theta_values[i]
                path.append((x, y, theta))

            return path
             

    # draw the obstalces and goal
    def draw_objects(obstacles, goals):
        for ob in obstacles:
            pygame.draw.rect(screen, tuple(name_to_rgb('black')), (int(u0 + k * ob[0]), int(v0 - k * ob[1]),1,1), width=0)
        # for goal in goals:
        pygame.draw.circle(screen,tuple(name_to_rgb('Navy')) , (int(u0 + k * goal[0]), int(v0 - k * goal[1])), int(k * 0.1* render_scale), 0)



    while (True):
        event_list = pygame.event.get()
        
        # Save trail for displaying
        location_history.append((x,y))


        # Planning

        best_benefit = -1e8
        forward_weight = 12
        obs_weight = 1000
        mean_weight =  4

       

        angle_weight = 5
        history_weight = 4
        pull_to_obstacle_weight = 5
        

        obs_copied = copy.deepcopy(obs)
        goal_copied = copy.deepcopy(goal)

        vL_list = (vL - max_accel*dt, vL, vL+ max_accel*dt)
        vR_list = (vR - max_accel*dt, vR, vR+ max_accel*dt)

        paths_to_draw = []
        new_pos_to_draw = []
        splined_paths = []

        for vl in vL_list:
            for vr in vR_list:
                if (vl <= max_vel and vr <= max_vel and vl >= -max_vel and vr >= -max_vel):

                    x_predict, y_predict, theta_predict, motion = predict_next(vl,vr,x,y,theta,tau)
                    path = generate_path(x, y, x_predict, y_predict, theta_predict)
                    paths_to_draw.append(motion)
                    new_pos_to_draw.append((x_predict,y_predict))

                    distance2obs = dist_to_closest_obstacle(x_predict,y_predict,obs)

                    prev_distance_to_goal = math.sqrt((x - goal[0])**2+(y-goal[1])**2)
                    updated_distance_to_goal = math.sqrt((x_predict-goal[0])**2+(y_predict-goal[1])**2)
                    distance_forward = prev_distance_to_goal - updated_distance_to_goal
                    distance_benefit = forward_weight* distance_forward

                    goal_angle = math.atan2(goal[1] - y, goal[0] - x)
                    heading_diff = theta_predict - goal_angle
                    heading_diff = (heading_diff + math.pi) % (2 * math.pi) - math.pi

                    heading_term =  angle_weight * (1 - abs(heading_diff) / math.pi)

                    min_distance_to_visited = min([math.sqrt((x - vx)**2 + (y - vy)**2) for vx, vy in location_history])
                    history_term = history_weight * (1 / (min_distance_to_visited + 0.1))

                    '''MEAN TERM'''
                    mean_term = mean_dist_to_obstacles(x_predict,y_predict,obs)
                    mean_term *= mean_weight

                    if distance2obs < safedistance:
                        obstacle_cost = obs_weight *(safedistance - distance2obs)
                    else:
                        obstacle_cost = 0.0

                    # Pull to farthest obstacle term
                    dist_to_farthest_obs = max([dist_to_closest_obstacle(x_predict,y_predict,obs_copied)])
                    pull_to_obstacle_term = pull_to_obstacle_weight * (1 - dist_to_closest_obstacle(x_predict,y_predict,obs_copied) / dist_to_farthest_obs)


                    '''Another term'''
                    d2globe = dist_to_globe(x,y,obs)
                    d2 = (3-d2globe) *100
                    benefit = distance_benefit - obstacle_cost
                    

                    if benefit > best_benefit:
                        vl_chosen = vl
                        vr_chosen = vr
                        best_benefit = benefit
                    

        vL = vl_chosen
        vR = vr_chosen
       


        obs = copy.deepcopy(obs_copied)
        goal = copy.deepcopy(goal_copied)


        '''Pygame rendering'''
        
        # Planning is finished; now do graphics
        screen.fill(tuple(name_to_rgb('white')))
        for loc in location_history:
                pygame.draw.circle(screen,tuple(name_to_rgb('green')), (int(u0 + k * loc[0]), int(v0 - k * loc[1])), 3, 3)
        draw_objects(obs,goal)

        
        # Draw robot
        u = u0 + k * x
        v = v0 - k * y
        pygame.draw.circle(screen,tuple(name_to_rgb('white')) , (int(u), int(v)), int(k * robot_radius* render_scale))
        # Draw wheels as little blobs so you can see robot orientation
        # left wheel centre 
        wlx = x - (robot_width/2.0) * math.sin(theta)
        wly = y + (robot_width/2.0) * math.cos(theta)
        ulx = u0 + k * wlx
        vlx = v0 - k * wly
        WHEELBLOB = 0.04
        pygame.draw.circle(screen,tuple(name_to_rgb('blue')), (int(ulx), int(vlx)), int(k * WHEELBLOB),)
        # right wheel centre 
        wrx = x + (robot_width/2.0) * math.sin(theta)
        wry = y - (robot_width/2.0) * math.cos(theta)
        urx = u0 + k * wrx
        vrx = v0 - k * wry
        pygame.draw.circle(screen, tuple(name_to_rgb('blue')), (int(urx), int(vrx)), int(k * WHEELBLOB))


        if (1):
                for path in paths_to_draw:
                        #if path[0] = 1:    # Pure rotation: nothing to draw
                        if path[0] == 0:    # Straight line
                                straightpath = path[1]
                                linestart = (u0 + k * x, v0 - k * y)
                                lineend = (u0 + k * (x + straightpath * math.cos(theta)), v0 - k * (y + straightpath * math.sin(theta)))
                                pygame.draw.line(screen, (0, 200, 0), linestart, lineend, 1)
                        if path[0] == 2:    # General case: circular arc
                                if (path[3] > path[2]):
                                        startangle = path[2]
                                        stopangle = path[3]
                                else:
                                        startangle = path[3]
                                        stopangle = path[2]
                               
                                if (startangle < 0):
                                        startangle += 2*math.pi
                                        stopangle += 2*math.pi
                                if (path[1][1][0] > 0 and path[1][0][0] > 0 and path[1][1][1] > 1):
                                        #print (path[1], startangle, stopangle)
                                        pygame.draw.arc(screen, (0, 200, 0), path[1], startangle, stopangle, 1)
        
        # Update display
        pygame.display.flip()

        (x, y, theta, tmppath) = predict_next(vL, vR, x, y, theta, dt)

        
        #If the robot reach an intermediate point. move to next and end at the final point in the global plath
        disttotarget = math.sqrt((x - goal[0])**2 + (y - goal[1])**2)
        if (disttotarget < (obs_radius + robot_radius)):
            print("Points Visited: ",goal)
            if goal_idx < Num_points-1:
                goal_idx+=1
                goal = global_path[goal_idx]
            else:
                break
      
                 
        # sim rendering interval
        time.sleep(0.00001)


if __name__ == '__main__':

    start  = (4,15)
    goal = (17,20)
    main(start,goal,method='cubic')

