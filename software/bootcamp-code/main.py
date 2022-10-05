import image_processor
import camera
import motion
import cv2
import time
import serial.tools.list_ports

def main_loop():
    debug = True
    
    motion_sim = motion.TurtleRobot()
    motion_sim2 = motion.TurtleOmniRobot()

    omni_motion = motion.OmniMotionRobot()
    
    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100)
    
    processor = image_processor.ImageProcessor(cam, debug=debug)

    processor.start()
    motion_sim.open()
    motion_sim2.open()

    # open the serial connection
    omni_motion.open()

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    try:
        start_time = time.time()
        # unnessecary now:       	sent=0
        
        while True:
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)

            # This is where you add the driving behaviour of your robot. It should be able to filter out
            # objects of interest and calculate the required motion for reaching the objects

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
            
            #find time passed since the start of program
            time_passed = time.time()-start_time
            
            move_wheel_s(omni_motion, 0, 2, time_passed, 20)
            move_wheel_s(omni_motion, 2, 4, time_passed, -20)
            move_wheel_s(omni_motion, 4, 6, time_passed)
            """
            if time_passed < 2:
                if sent!=1:
                    sent=1
                    print("sent1")
                omni_motion.send_commands(20, 0, 0, 0)
            elif 4 > time_passed > 2:
                if sent!=2:
                    sent=2
                    print("sent2")
                omni_motion.send_commands(-20, 0, 0, 0)
            elif 6 > time_passed > 4:
                omni_motion.send_commands(0, 0, 0, 0)
             """

    except KeyboardInterrupt:
        print("closing....")
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        motion_sim.close()
        motion_sim2.close()
        omni_motion.close()

main_loop()

# function to move wheel/wheels for given time
def move_wheel_s(omnim_robot_instance, start_time, end_time, time_passed, speed1=0, speed2=0, speed3=0):
    if time_passed<start_time:
        return
    elif time_passed>end_time:
        return
    else:
        omnim_robot_instance.send_commands(speed1, speed2, speed3, 0)
    
    
    
