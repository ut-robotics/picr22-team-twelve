import image_processor
import camera
import motion
import cv2
import time
from enum import Enum # for state machine enumerate
import asyncio #for referee commands over websockets
import websockets #before use: pip3.9 install websockets
import json #for parsing referee commands into python library
from threading import Thread

#STATE MACHINE
class State(Enum):
    FIND_BALL = 0
    MOVE_CENTER_BALL = 1
    FIND_BASKET = 2
    THROW_BALL = 3
    STOP = 4
    TEST_THROWER = 5
    FIND_FURTHEST_BASKET = 6
    DRIVE_TO_OP_BASKET = 7

class BasketColor(Enum):
    BLUE=0
    MAGENTA=1

def run_websocket(command_list):
    asyncio.new_event_loop().run_until_complete(listen_referee(command_list))

# method for listening to referee commands
async def listen_referee(command_list):
    ip_addr='192.168.3.28:8222'
    async with websockets.connect('ws://'+ip_addr) as websocket:
        while True:
            print("while")
            command = await websocket.recv()
            command_list.append(command) #adds to the end of the list

# function to get the latest referee command
def get_referee_commands(command_list, robot_id):
    print("GET")
    # if there are no commands in the list, return STOP and basketcolor blue
    if len(command_list)==0:
        print("no commands in list")
        # TODO: how to not change the state?
        return State.STOP, BasketColor.BLUE

    # parse referee commands into python library (https://www.w3schools.com/python/python_json.asp)
    # pop from the list, but then problems with empty list changing state to stop. Instead just view the last command over and over again
    #referee=json.loads(command_list.pop())
    referee=json.loads(command_list[-1])
    # get the index commands meant for my robot id
    my_index = referee["targets"].index(robot_id)
    # if -1 then it wasn't my command, return
    if my_index == -1:
        print("not my command")
        return State.STOP, BasketColor.Blue
    
    command=referee["signal"][my_index]
    if command=="start":
        current_state = State.FIND_BALL
        color = referee["baskets"][my_index]
        if color == "magenta": current_target = BasketColor.MAGENTA
        else: current_target = BasketColor.BLUE
        print("return start:" current_state, current_target)
        return current_state, current_target
    elif command=="stop":
        print("return stop")
        return State.STOP, BasketColor.BLUE

# function to print the state when changing state
def state_printer(state, last_state, new_state):
    if last_state == state:
        new_state = False
    else:
        last_state = state
        new_state = True
    return last_state, new_state

# method for normalizing object x or y coordinate
# Normalize destination in the [-1;1] range and and then multiply it with motor max speed.
# (then the driving is proportional)
# The destination coordinates are the difference between the ball location and desired location.
def norm_co(desired_location, coordinate, max_range):
    proportional = (desired_location - coordinate)/max_range
    return proportional

def main_loop():
    # state to show camera image
    debug = False
    # variable to store target basket color, currently blue for testing (if want magenta, change b to False)
    basket_color = BasketColor.BLUE

    #motion_sim = motion.TurtleRobot()
    #motion_sim2 = motion.TurtleOmniRobot()

    omni_motion = motion.OmniMotionRobot()
    
    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    #camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100)
    processor = image_processor.ImageProcessor(cam, debug=debug)

    processor.start() # ilma kaamerata hangub siin

    #motion_sim.open()
    #motion_sim2.open()

    # open the serial connection
    omni_motion.open()
    
    # state if to listen to referee commands, when needed, change to True
    referee_active = True
    command_list=[]
    robot_id="twelve"

    # listen for referee commands
    if referee_active:
        print("start referee")
        thread = Thread(target=run_websocket, args=(command_list,))
        thread.start()
        state=State.STOP
    # list for referee commands and robot_id
    else:
        # default state to start with
        state=State.FIND_BALL

    # variables to control driving to furthest basket when ball has not been found for max_find_time
    finder_timer=time.time()
    max_find_time=5
    furthest_basket = BasketColor.BLUE
    basket_to_drive=None
    basket_blue_d=0
    basket_magn_d=0
    start_opbasket_drive=time.time()


    # if want to test the thrower, comment in
    #state = State.TEST_THROWER
    testing_thrower_speed=1900

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    
    # time counter for testing
    # zero_time = time.time()
        
    # Ball desired coordinates are in the middle of the frame width and 400 from the upper edge of the frame
    # (THE ZERO COORDINATES OF THE FRAME ARE IN THE UPPER LEFT CORNER.)
    ball_desired_x = cam.rgb_width/2
    ball_desired_y = 400

    # Speed range for motors is 48 - 2047, we use 100 for max motor speed at the moment.
    max_motor_speed = 80
    # rotation speed for find ball state
    find_rotation_speed = max_motor_speed/6
    # orbiting speed for centering the basket
    orbit_speed = max_motor_speed/1.5

    # start time variable for thrower state
    throw_start=0
    # true false variable to keep track if when throwing, the ball has left the camera frame
    ball_out_of_frame=False
    # when throwing the ball, the speed which the robot moves forward
    throw_move_speed=max_motor_speed/3
    # maximum and minimum speed to throw the ball
    throw_motor_speed_max=1750
    throw_motor_speed_min=650
    thrower_speed_range=throw_motor_speed_max-throw_motor_speed_min
    # maximum basket distance in mm
    max_basket_depth=4000

    # for printing logs (log when change state)
    new_state=True
    last_state=State.FIND_BALL
    
    try:        
        while True:
	
            # get the referee command
            # when connection lost, retry connection
            if referee_active:
                try:
                    # listen for referee commands
                    print(command_list)
                    state, basket_color = get_referee_commands(command_list, robot_id)
                    print("GOT: ", state, basket_color)
                except:
                    print("EXCEPTION")

            # method for printing the state only when it changes
            if new_state==True: print(state)
            last_state, new_state =  state_printer(state, last_state, new_state)
            
            # Proccess the camera frame and get data
            aligned=False
            if (state==State.THROW_BALL or state==State.FIND_FURTHEST_BASKET or state==State.DRIVE_TO_OP_BASKET): aligned=True
            # has argument aligned_depth that enables depth frame to color frame alignment
            processedData = processor.process_frame(aligned_depth=aligned)
	
            # get the basket data to which we want to throw into, depends which color basket is set
            basket_to_throw = processedData.basket_b
            if basket_color==BasketColor.MAGENTA: basket_to_throw=processedData.basket_m
            
            # get the furthest basket data for the drive to the opposing basket state when ball has not been found in the find ball state
            if furthest_basket == BasketColor.BLUE: basket_to_drive=processedData.basket_b
            elif furthest_basket == BasketColor.MAGENTA: basket_to_drive=processedData.basket_m
            
            # Here is the state machine for the driving logic.
            #   Omni-motion movement:
            #   side speed is x_speed
            #   forward speed is y_speed
            #   rotation if you want to turn
            
            # STATE TO FIND THE BALL
            if state == State.FIND_BALL:

                # if we have a ball in view, center it
                if processedData.balls_exist==True:
                    state=State.MOVE_CENTER_BALL
                    continue
                # if no ball has been found for given maximum time find the furthest basket
                elif (time.time()-finder_timer)>=max_find_time:
                    state=State.FIND_FURTHEST_BASKET
                print("Throw timer: ", time.time()-finder_timer)

                # if no ball found, rotate
                omni_motion.move(0, 0, find_rotation_speed)

            # STATE TO MOVE TO AND CENTER THE BALL
            elif state == State.MOVE_CENTER_BALL:
                # if there are no balls, then find one
                #if len(processedData.balls)<1:
                if processedData.balls_exist==False:
                    state=State.FIND_BALL
                    finder_timer=time.time()
                    continue
                
                # if ball is close enough, circle it and find a basket (the distance is from the 0 coordinate - closer is larger value)
                if processedData.biggest_ball.distance>380:
                    state=State.FIND_BASKET
                    continue
                # else simultaniously drive to and center the ball
                else:
                    # Using the coordinates of the biggest ball calculate the side speed, forward speed and rotation for the robot.
                    x_speed = norm_co(ball_desired_x, processedData.biggest_ball.x, cam.rgb_width) * max_motor_speed
                    y_speed = norm_co(ball_desired_y, processedData.biggest_ball.y, cam.rgb_height) * max_motor_speed
                    #print(-1*x_speed, -1*y_speed, x_speed)
                    omni_motion.move(-1*x_speed, -1*y_speed, x_speed)
            
            # state to find the basket when ball is found - circle around the ball until basket is found, then move on to throwing
            elif state==State.FIND_BASKET:
                # if there are no balls, then find one
                if processedData.balls_exist==False:
                    state=State.FIND_BALL
                    finder_timer=time.time()
                    continue
                    
                #x-speed (side speed) is the proportional speed of the normalised difference between ball x coordinate and basket x coordinate
                x_speed_prop = -1*norm_co(processedData.biggest_ball.x, 0, cam.rgb_width)*orbit_speed
                if basket_to_throw.exists:
                    x_speed_prop = -1*norm_co(basket_to_throw.x, processedData.biggest_ball.x, cam.rgb_width)*orbit_speed*1.5
                
                # y-speed (forward speed) is calculated based on the distance of the ball
                y_speed_prop = -1*norm_co(ball_desired_y, processedData.biggest_ball.distance, cam.rgb_height)
                # rotational speed is the difference between the desired x-location of the ball and actual, normalized and proportional
                rot_speed_prop = norm_co(ball_desired_x, processedData.biggest_ball.x, cam.rgb_width)
                print("X_speed: ", x_speed_prop, "Y_speed: ", y_speed_prop, "rot: ", rot_speed_prop)
                # if the basket and ball are in the center of the frame and ball is close enough move on to throwing
                if -0.8<x_speed_prop<0.8 and -0.1<rot_speed_prop<0.1 and -0.1<y_speed_prop<0.1 and basket_to_throw.exists:
                    state = State.THROW_BALL
                    continue
                # center the basket and the ball with orbiting, get the ball to the desired distance
                omni_motion.move(x_speed_prop, orbit_speed*y_speed_prop, orbit_speed/1.5*rot_speed_prop) #added *1.5

            # THROW THE BALL STATE:
            elif state==State.THROW_BALL:

                # enters the if statement once to start the throw timer when the ball is out of frame
                if ball_out_of_frame==False and (processedData.balls_exist==False or processedData.biggest_ball.y>450):
                    ball_out_of_frame=True
                    throw_start=time.time()
                    print("Starting the throw timer")

                # if the ball is out of frame then the throw duration should be 1.2s
                throw_duration=time.time()-throw_start
                if throw_duration>2 and ball_out_of_frame==True:
                    state=State.FIND_BALL
                    finder_timer=time.time()
                    # reset variables
                    throw_start=0
                    ball_out_of_frame=False
                    print("ball throwing time up")
                    continue
		
                #when the ball is not in view, calculate proportional speed for the thrower and forward speed based on basket
                if ball_out_of_frame==True:
                    #print("ball out of frame, calculate side-speed prop to basket x location")
                    x_speed_prop = norm_co(ball_desired_x, basket_to_throw.x, cam.rgb_width)
                    rot_speed_prop = x_speed_prop

                # when the ball is in view, drive towards it, x-speed based on ball and basket x-coordinate difference
                else:
                    #print("ball in view, calculate side-speed prop. to basket and ball x location difference")
                    # x speed aka side speed is proportional to the distance of the ball from the basket
                    x_speed_prop = norm_co(processedData.biggest_ball.x, basket_to_throw.x, cam.rgb_width)
                    rot_speed_prop = norm_co(ball_desired_x, processedData.biggest_ball.x, cam.rgb_width)
		    
                # y speed aka forward speed is proportional to the basket distance in the frame considering y coordinate -destination is 100pixels from the bottom edge
                y_speed_prop=norm_co((cam.rgb_height-100), basket_to_throw.y, (cam.rgb_height-100))
                # calculate the thrower motor speed based on the basket distance gotten from depth camera
                # take depth frame basket x, y depth, better to take matrix of all the nearest and the medium of that
                basket_depth = basket_to_throw.distance
                #print("BASKET_DEPTH:", basket_depth)
                basket_dist_norm = basket_depth/max_basket_depth
                thrower_speed_prop=basket_dist_norm*thrower_speed_range+throw_motor_speed_min
		
                omni_motion.move(-1*x_speed_prop*throw_move_speed, -1*y_speed_prop*throw_move_speed, rot_speed_prop*throw_move_speed*2, thrower_speed_prop) #added *2
                
            elif state==State.STOP:
                omni_motion.move(0, 0, 0, 0)

            # to test the thrower
            elif state==State.TEST_THROWER:
                omni_motion.move(0, 0, 0, testing_thrower_speed)
                print("testing thrower")
                processedData=processor.process_frame(aligned_depth=False)
                basket_depth = basket_to_throw.distance
                print(basket_depth)

            elif state==State.FIND_FURTHEST_BASKET:
                # rotate and find both the baskets, when found, drive to the furthest
                if processedData.basket_b.exists:
                    blue_basket_depth = processedData.basket_b.distance
                    if blue_basket_depth>basket_blue_d: basket_blue_d=blue_basket_depth
                elif processedData.basket_m.exists:
                    magn_basket_depth = processedData.basket_m.distance
                    if magn_basket_depth>basket_magn_d: basket_magn_d=magn_basket_depth

                # compare and see which is furthest
                if basket_blue_d>=basket_magn_d: furthest_basket=BasketColor.BLUE
                elif basket_magn_d>basket_blue_d: furthest_basket=BasketColor.MAGENTA
                
                print(furthest_basket, basket_blue_d, basket_magn_d)
                if basket_blue_d>0 and basket_magn_d>0 and basket_to_drive.exists:
                    state=State.DRIVE_TO_OP_BASKET
                    start_opbasket_drive=time.time()
                    continue
                omni_motion.move(0, 0, find_rotation_speed/2)

            # State to drive to the furthest basket when in the find_ball state the ball has not been found for some time.
            elif state==State.DRIVE_TO_OP_BASKET:
                basket_depth = basket_to_drive.distance
                if (time.time()-start_opbasket_drive)>2.2: # or processedData.balls_exist==True: # if basket is closer than 25cm
                    state=State.FIND_BALL
                    basket_blue_d=0
                    basket_magn_d=0
                    finder_timer=time.time()
                    continue
                # drive straight towards the furthest basket while correcting movement based on the baskets x, y coordinates
                x_speed_prop = norm_co(ball_desired_x, basket_to_drive.x, cam.rgb_width)
                y_speed_prop = norm_co((cam.rgb_height-100), basket_to_drive.y, (cam.rgb_height-100))
                omni_motion.move(-1*x_speed_prop*max_motor_speed/2, -1*y_speed_prop*max_motor_speed, x_speed_prop*max_motor_speed/2)
                

            """# Mainboard and communication testing function.
            # Move two wheels for 4s. Starting from program start time (zero_time) and duration 0 to 4.
            omni_motion.test_moving(zero_time, 0, 4, 0, -15, 15)"""

            frame_cnt +=1

            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                print("ball_count: {}".format(len(processedData.balls)))
                #if (frame_cnt > 1000):
                #    break

            if debug:
                debug_frame = processedData.debug_frame
                cv2.imshow('debug', debug_frame)
                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break


    except KeyboardInterrupt:
        print("closing....")
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        #motion_sim.close()
        #motion_sim2.close()
        omni_motion.close()

main_loop()
    
