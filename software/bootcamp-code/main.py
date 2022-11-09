import image_processor
import camera
import motion
import cv2
import time
from enum import Enum # for state machine enumerate
import asyncio #for referee commands over websockets
import websockets #before use: pip3.9 install websockets
import json #for parsing referee commands into python library

"""async def listen_referee(command_list):
    async with websockets.connect('ws://localhost:8008') as websocket:
        command = await websocket.recv()
        command_list.append(command)
"""
# STATE MACHINE: 
class State(Enum):
    FIND_BALL = 0
    MOVE_CENTER_BALL = 1
    FIND_BASKET = 2
    THROW_BALL = 3
    STOP = 4

# function to print the state when changing state
def state_printer(state, last_state, new_state):
    if last_state == state:
        new_state = False
    else:
        last_state = state
        new_state = True
    return last_state, new_state

def main_loop():
    debug = False
    
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
    
    command_list=[]
    # listen for referee commands
    #asyncio.get_event_loop().run_until_complete(listen_referee(command_list))
    # parse referee commands into python library (https://www.w3schools.com/python/python_json.asp)
    #referee=json.loads(command_list[0])
    #if referee["signal"] == "start": state=State.FIND_BALL
    #else: state=State.STOP
    state=State.FIND_BALL

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    
    # time counter for testing
    # zero_time = time.time()
        
    # Ball desired coordinates are in the middle of the frame width and 83% of frame height
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
    throw_move_speed=max_motor_speed/2
    # the maximum distance of the ball (y zero coordinate is in the upper edge of the frame)
    basket_max_distance=cam.rgb_height-100
    # maximum speed to throw the ball
    throw_motor_speed_max=2000
    # variable to store target basket color, currently blue for testing
    basket_color="basket_b"

    # for printing logs (log when change state)
    new_state=True
    last_state=State.FIND_BALL
    
    try:        
        while True:

            while True:
                omni_motion.move(0, 0, 0, 800)
                print("testing thrower")
                print(processor.process_frame(aligned_depth=False).basket_b.distance)
		#distance 18 - speed 900
		# speed 1000 - dist 17, 18
            # method for printing the state only when it changes
            if new_state==True: print(state)
            last_state, new_state =  state_printer(state, last_state, new_state)
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)
            
            # Here is the state machine for the driving logic.
                #Omni-motion movement:
                # side speed is x_speed
                # forward speed is y_speed
                # rotation if you want to turn
            
            # STATE TO FIND THE BALL
            if state == State.FIND_BALL:
                # if we have a ball in view, center it
                if len(processedData.balls)>0:
                    state=State.MOVE_CENTER_BALL
                    continue
                # if no ball found, rotate
                
                omni_motion.move(0, 0, find_rotation_speed)

            # STATE TO MOVE TO AND CENTER THE BALL
            elif state == State.MOVE_CENTER_BALL:
                # if there are no balls, then find one
                if len(processedData.balls)<1:
                    state=State.FIND_BALL
                    continue
                    
                # The biggest ball is the last one in the detected balls list.
                # Using the coordinates of the biggest ball calculate the side speed, forward speed and rotation for the robot.
                
                # The destination coordinates are the difference between the ball location and desired location.
                dest_x = ball_desired_x - processedData.balls[-1].x
                dest_y = ball_desired_y - processedData.balls[-1].y
              
                # if ball is close enough, circle it and find a basket (the distance is from the 0 coordinate - closer is larger value)
                if processedData.balls[-1].distance>380:
                    state=State.FIND_BASKET
                    continue
                # else simultaniously drive to and center the ball
                else:
                    # Normalize destination in the [-1;1] range and and then multiply it with motor max speed.
                    # (then the driving is proportional)
                    x_speed = (dest_x/cam.rgb_width) * max_motor_speed
                    y_speed = (dest_y/cam.rgb_height) * max_motor_speed
                    omni_motion.move(-1*x_speed, -1*y_speed, x_speed)
            
            # state to find the basket when ball is found - circle around the ball until basket is found, then move on to throwing
            elif state==State.FIND_BASKET:
                # if there are no balls, then find one
                if len(processedData.balls)<1:
                    state=State.FIND_BALL
                    continue
                    
                #x-speed (side speed) is the proportional speed of the normalised difference between ball x coordinate and basket x coordinate
                x_speed_prop = (processedData.balls[-1].x - 0) / cam.rgb_width
                if processedData.basket_b.exists:
                    x_speed_prop=(processedData.balls[-1].x-processedData.basket_b.x)/cam.rgb_width
                
                # y-speed (forward speed) is calculated based on the distance of the ball 
                y_speed_prop=(processedData.balls[-1].distance-400)/(cam.rgb_height-400)
                # rotational speed is the difference between the desired x-location of the ball and actual, normalized and proportional
                rot_speed_prop= (ball_desired_x - processedData.balls[-1].x)/cam.rgb_width
                #print("X_speed: ", x_speed_prop, "Y_speed: ", y_speed_prop, "rot: ", rot_speed_prop)
                # if the basket and ball are in the center of the frame and ball is close enough move on to throwing
                if -0.1<x_speed_prop<0.1 and -0.1<rot_speed_prop<0.1 and -0.1<y_speed_prop<0.1:
                    state = State.THROW_BALL
                    continue
                # center the basket and the ball with orbiting, get the ball to the desired distance
                omni_motion.move(orbit_speed*x_speed_prop, orbit_speed*y_speed_prop, orbit_speed*rot_speed_prop)


            # drive ontop of the ball and throw it.
            elif state==State.THROW_BALL:              
                # enters the if statement once to start the throw timer when the ball is out of frame
                if ball_out_of_frame==False:
                    if (len(processedData.balls))<1:
                        ball_out_of_frame=True
                        throw_start=time.time()
                        print("no balls found")
                    elif processedData.balls[-1].y<400:
                        ball_out_of_frame=True
                        throw_start=time.time()
                        print("BIGgest ball dist<400")

                # if the ball is out of frame then the throw duration should be bigger than 1.2
                throw_duration=time.time()-throw_start
                if throw_duration>4 and ball_out_of_frame==True:
                    state=State.FIND_BALL
                    throw_start=0
                    ball_out_of_frame=False
                    print("ball out and time out")
                    continue


                if ball_out_of_frame==True:
		    print("ball not in view drive")
                    #when the ball is not in view, calculate proportional speed for the thrower and forward speed based on basket
                    y_speed_prop=((cam.rgb_height-100)-processedData.basket_b.y)/cam.rgb_height
                    x_speed_prop=(cam.rgb_width/2 - processedData.basket_b.x)/cam.rgb_width
		    # normalize the basket distance from the robot to the 0-1 range
                    basket_dist_norm = (basket_max_distance-processedData.basket_b.y)/basket_max_distance
                    omni_motion.move(-1*x_speed_prop*throw_move_speed, -1*y_speed_prop*throw_move_speed/4, 0, throw_motor_speed_max*basket_dist_norm)


                # when the ball is in view, drive towards it
                else:
                    print("ball in view drive")
                    # enters the if statement once to start the throw timer when the ball has just gone out of frame
                    if (len(processedData.balls))<1:
                        ball_out_of_frame=True
                        throw_start=time.time()
                        continue
                    elif processedData.balls[-1].distance<(cam.rgb_height-70): # when there are other balls in view but the biggest is very close
                        ball_out_of_frame=True
                        throw_start=time.time()
                        continue
                    #TODO: For the thrower motor speeds, I suggest mapping the mainboard-speed to throwing distance. 
                    # Based on that you can either estimate a function or linearly interpolate the speeds.
                    
                    # normalize the basket distance from the robot to the 0-1 range
                    basket_dist_norm = (basket_max_distance-processedData.basket_b.y)/basket_max_distance
                    # x speed is proportional to the distance of the ball from the basket
                    x_speed_prop = (processedData.balls[-1].x-processedData.basket_b.x)/cam.rgb_width
                    #the max distance of the basket is 100 pixels from out of frame
                                                            
                    omni_motion.move(x_speed_prop*throw_move_speed, -1*basket_dist_norm*throw_move_speed, 0, throw_motor_speed_max*basket_dist_norm)
                
            
            elif state==State.STOP:
                omni_motion.move(0, 0, 0)
                

            # Mainboard and communication testing function.
            # Move two wheels for 4s. Starting from program start time (zero_time) and duration 0 to 4.
            # omni_motion.test_moving(zero_time, 0, 4, 0, -15, 15)

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
    
