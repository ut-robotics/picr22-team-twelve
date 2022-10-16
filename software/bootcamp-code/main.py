import image_processor
import camera
import motion
import cv2
import time
from enum import Enum

# STATE MACHINE: 3 states - FIND_BALL = 0, CENTER_BALL = 1, DRIVE_TO_BALL = 2
class State(Enum):
    FIND_BALL = 0
    CENTER_BALL = 1
    DRIVE_TO_BALL = 2

def main_loop():
    debug = True
    
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
    try:
        # time counter for testing
        zero_time = time.time()
        # Ball desired coordinates are in the middle of the frame.
        ball_desired_x = cam.rgb_width/2
        ball_desired_y = cam.rgb_height/2
        
        while True:
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)

            # TODO This is where you add the driving behaviour of your robot. It should be able to filter out
            # objects of interest and calculate the required motion for reaching the object.

            # STATE TO FIND THE BALL 0
            if state == State.FIND_BALL:
                if len(processedData.balls)>0:
                    state=State.CENTER_BALL
                    continue
                rotation = 15
                omni_motion.move(0, 0, rotation)

            # STATE TO DRIVE TO BALL is 1
            elif state == State.CENTER_BALL:
                # The biggest ball is the first one in the detected balls list.
                # Find the coordinates of the largest ball.
                # (THE ZERO COORDINATES OF THE FRAME ARE IN THE UPPER LEFT CORNER.)
                # Using these coordinates calculate the side speed, forward speed and rotation for the robot.
                
                # The destination coordinates are the difference between the ball location and desired location.
                dest_x = ball_desired_x - processedData.balls[0].x
                dest_y = ball_desired_y - processedData.balls[0].y
                # Normalize destination in the [-1;1] range and and then multiply it with motor max speed.
                # (then the driving is proportional)
                # Speed range for motors is 48 - 2047, we use 100 for max motor speed at the moment.
                x_speed = (dest_x/cam.rgb_width) * 100
                y_speed = (dest_y/cam.rgb_height) * 100
                #rotation = 0
                # Drive towards a ball:
                    # side speed is x_speed
                    # forward speed is y_speed
                    # rotation if you want to turn
                #print("X: ", x_speed, "Y: ", y_speed)
                omni_motion.move(x_speed, y_speed, 0)
                

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
    
