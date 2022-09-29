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

    # incase permission issues: sudo chmod 666 /dev/tty0
    # idProduct=5740 to filter the list given by  serial.tools.list_ports.comports(include_links=False)
    # serial connection port
    port = '/dev/ttyACM0'
    # get all the available seiral ports
    ports = serial.tools.list_ports.comports()
    for p in ports:
        print(p.device)
        print(p.hwid)
        if p.hwid.__contains__("5740"):
            port=p.device
            print("Serial port m22ratud")

    omni_motion.open(port)

    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    try:
        start_time = time.time()
        sent=0
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
            current_time = time.time()-start_time
            if current_time < 2:
                if sent!=1:
                    sent=1
                    print("sent1")
                omni_motion.send_commands(20, 0, 0, 0)
            elif current_time > 2:
                if sent!=2:
                    sent=2
                    print("sent2")
                omni_motion.send_commands(-20, 0, 0, 0)
            elif 6 > current_time > 4:
                omni_motion.send_commands(0, 0, 0, 0)

    except KeyboardInterrupt:
        print("closing....")
    finally:
        cv2.destroyAllWindows()
        processor.stop()
        motion_sim.close()
        motion_sim2.close()
        omni_motion.close(port)

main_loop()
