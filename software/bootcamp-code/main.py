import image_processor
import camera
import motion
import cv2
import time
from enum import Enum

# STATE MACHINE: 
class State(Enum):
    FIND_BALL = 0
    MOVE_CENTER_BALL = 1
    FIND_BASKET = 2
    THROW_BALL = 3
    STOP = 4
    
class BasketColor(Enum):
    MAGENTA = 1
    BLUE = 2

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
    ball_desired_y = cam.rgb_height/1.2
    center_frame=ball_desired_x
    # Speed range for motors is 48 - 2047, we use 100 for max motor speed at the moment.
    max_motor_speed = 100
    # rotation speed for find ball state
    find_rotation_speed = max_motor_speed/15
    # variable to store target basket color, currently blue for testing
    basketColor = BasketColor.BLUE
    # orbiting speed for centering the basket
    orbit_speed = max_motor_speed/15
    # start time variable for thrower state
    throw_start=MAX_INT
    # true false variable to keep track if when throwing, the ball has left the camera frame
    ball_out_of_frame=False
    # when throwing the ball, the speed which the robot moves forward
    throw_forward_speed=max_motor_speed/8
    # the maximum distance of the ball (y zero coordinate is in the upper edge of the frame)
    ball_max_distance=cam.rgb_height
    # maximum speed to throw the ball
    throw_motor_speed_max=max_motor_speed/2
    
    try:        
        while True:
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)
            
            # Here is the state machine for the driving logic.
            
            # STATE TO FIND THE BALL
            if state == State.FIND_BALL:
                print(state)
                # if we have a ball in view, center it
                if len(processedData.balls)>0:
                    state=State.MOVE_CENTER_BALL
                    continue
                # if no ball found, rotate
                
                omni_motion.move(0, 0, find_rotation_speed)

            # STATE TO MOVE TO AND CENTER THE BALL
            elif state == State.MOVE_CENTER_BALL:
                print(state)
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
                if processedData.balls[-1].distance>500:
                    state=State.FIND_BASKET
                    continue
                # else simultaniously drive to and center the ball
                else:
                    # Normalize destination in the [-1;1] range and and then multiply it with motor max speed.
                    # (then the driving is proportional)
                    x_speed = (dest_x/cam.rgb_width) * max_motor_speed
                    y_speed = (dest_y/cam.rgb_height) * max_motor_speed
                    # Driving towards a ball:
                        # side speed is x_speed
                        # forward speed is y_speed
                        # rotation if you want to turn
                    # print("X: ", x_speed, "Y: ", y_speed)
                    omni_motion.move(-1*x_speed, -1*y_speed, x_speed)
            
            # state to find the basket when ball is found - circle around the ball until basket is found, then move on to throwing
            elif state==State.FIND_BASKET:
                print(state)
                # if there are no balls, then find one
                if len(processedData.balls)<1:
                    state=State.FIND_BALL
                    continue
                    
                # find blue basket
                if basketColor==basketColor.BLUE:
                    if len(processedData.basket_b>0): # or processedData.basket_m>0)
                        # center the basket with orbiting
                        # the normalized destination x range is -0.05 to 0.05, if the x location is out of that range - orbit
                        if -0.05 > ((center_frame - processedData.basket_b[-1].x) / cam.rgb_width) > 0.05:
                            omni_motion.move(orbit_speed, 0, orbit_speed)
                        else:
                            state=State.THROW_BALL                           
                    
                # otherwise orbit the ball until basket is found
                # y(forward speed) = 0; x and rotation have speed as it turns and moves sideways at the same time when orbiting
                else:
                   omni_motion.move(orbit_speed, 0, orbit_speed)
            

            # drive ontop of the ball and throw it.
            elif state==State.THROW_BALL:
                print(state)
                
                # enters the if statement once to start the throw timer when the ball is out of frame
                if ball_out_of_frame==False and len(processedData.balls)<1:
                    ball_out_of_frame=True
                    throw_start=time.time()
                
                # if the ball is out of frame then the throw duration should be bigger than 1.2
                throw_duratin=time.time()-throw_start
                if throw_duration>1.2 and ball_out_of_frame==True:
                    state=State.FIND_BALL
                    throw_start=MAX_INT
                    ball_out_of_frame=False
                    continue
                
                elif ball_out_of_frame==True:
                    omni_motion.move(0, -throw_forward_speed, 0, throw_motor_speed)
                
                #when the ball is in view, calculate proportional speed for the thrower
                else:
                    #?????? For the thrower motor speeds, I suggest mapping the mainboard-speed to throwing distance. 
                    # Based on that you can either estimate a function or linearly interpolate the speeds. - Don't understand
                    
                    # normalize the ball distance from the robot to the 0-1 range
                    ball_dist_norm = (ball_max_distance-processedData.balls[-1].distance)/ball_max_distance
                    # proportional speed to the distance of the ball
                    throw_motor_speed = ball_dist_norm*throw_motor_speed_max
                    omni_motion.move(0, -throw_forward_speed, 0, throw_motor_speed)
                
            
            elif state==State.STOP:
                print(state)
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
    
