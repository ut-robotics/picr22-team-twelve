import image_processor
import camera
import motion
import cv2
import time

# function to move wheel/wheels for given time
def move_wheel_s(omnim_robot_instance, start_time, end_time, time_passed, speed1=0, speed2=0, speed3=0):
    if time_passed<start_time:
        return
    elif time_passed>end_time:
        return
    else:
        omnim_robot_instance.send_commands(speed1, speed2, speed3, 0)

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

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    try:
        start_time = time.time()
        state = 0
        while True:
            # STATE MACHINE: 3 states - FIND_BALL = 0, DRIVE_TO_BALL = 1, ALREADY_HAVE_A_BALL = 2

            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)

            # TODO This is where you add the driving behaviour of your robot. It should be able to filter out
            # objects of interest and calculate the required motion for reaching the objects
            print()

            if state>1:
                continue
            elif len(processedData.balls)>0:
                state=1
            else:
                state = 0

            # STATE TO FIND THE BALL
            if state == 0:
                largest_ball_xco = 0
                largest_ball_yco = 0
                rotation = 60
                omni_motion.move(largest_ball_xco, largest_ball_yco, rotation)

            # STATE TO DRIVE TO BALL
            elif state == 1:
                # the biggest ball is the first one in the detected balls list
                # the coordinates of the largest ball, THE ZERO COORDINATES OF THE FRAME ARE IN THE UPPER LEFT CORNER
                # Using these coordinates calculate the side speed, forward speed and rotation for the robot
                largest_ball_Object = processedData.balls[0]
                # We want the ball to end up in the middle of the frame
                ball_desired_x = cam.rgb_width/2
                ball_desired_y = cam.rgb_height/2
                largest_ball_xco = largest_ball_Object.x
                largest_ball_yco = largest_ball_Object.y
                # the destination coordinates are the difference between the ball location and desired location
                dest_x = ball_desired_x - largest_ball_xco
                dest_y = ball_desired_y - largest_ball_yco
                # normalize destination in the [-1;1] range and and then * it with motor max speed
                # (then the driving is proportional)
                # speed range for motors is 48 - 2047, we use 100 for max motor speed rn
                x_speed = (dest_x/cam.rgb_width) * 100
                y_speed = (dest_y/cam.rgb_height) * 100
                rotation = 0
                # Drive towards a ball
                # side speed is x_speed
                # forward speed is y_speed
                # rotation if you want to turn
                #print("X: ", x_speed, "Y: ", y_speed)
                omni_motion.move(x_speed, y_speed, rotation)



            # for mainboard testing driving function
            # find time passed since the start of program
            # time_passed = time.time() - start_time
            # move two wheels for 4s
            # move_wheel_s(omni_motion, 0, 4, time_passed, 0, -15, 15)

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
    
