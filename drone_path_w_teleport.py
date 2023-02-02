import setup_path 
import airsim

import sys
import time
import argparse

import numpy as np
import os
import tempfile
import pprint
import cv2

cnt = 0

class SurveyNavigator:
    def __init__(self, args):
        self.boxsize = args.size
        self.stripewidth = args.stripewidth
        self.altitude = args.altitude
        self.velocity = args.speed
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.pose = self.client.simGetVehiclePose()

    def takeImage(self):
        global cnt
        for i in range(6):
            print("rotation" + str(i))    

            self.client.moveByVelocityZAsync(0, 0, 0, 1, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 60 + i*60))
            #time.sleep(3)

            print('take images')
            # get camera images from the drone
            responses = self.client.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
                airsim.ImageRequest("1", airsim.ImageType.DepthPerspective, True), #depth in perspective projection
                airsim.ImageRequest("1", airsim.ImageType.Scene), #scene vision image in png format
                airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])  #scene vision image in uncompressed RGBA array
            print('Retrieved images: %d' % len(responses))

            tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
            print ("Saving images to %s" % tmp_dir)
            try:
                os.makedirs(tmp_dir)
            except OSError:
                if not os.path.isdir(tmp_dir):
                    raise

            for idx, response in enumerate(responses):

                filename = os.path.join(tmp_dir, f"{cnt}_{i}_{idx}")

                if response.pixels_as_float:
                    print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
                    airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
                elif response.compress: #png format
                    print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
                    airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
                else: #uncompressed array
                    print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
                    img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8) # get numpy array
                    img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
                    cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png
        cnt += 1

    def start(self):
        print("arming the drone...")
        self.client.armDisarm(True)

        landed = self.client.getMultirotorState().landed_state
        if landed == airsim.LandedState.Landed:
            print("taking off...")
            self.client.takeoffAsync().join()

        landed = self.client.getMultirotorState().landed_state
        if landed == airsim.LandedState.Landed:
            print("takeoff failed - check Unreal message log for details")
            return
        
        start = time.time()
        # AirSim uses NED coordinates so negative axis is up.
        x = -self.boxsize
        y = -self.boxsize
        z = -self.altitude

        print(self.pose.position)

        # Change altitude to Z !!
        print("climbing to altitude: " + str(self.altitude))
        self.pose.position.z_val = z
        self.client.simSetVehiclePose(self.pose, True)
        print(self.pose.position)
        #time.sleep(2)

        # Teleport to start point !!
        print("teleport to first corner of survey box")
        self.pose.position.x_val = x
        self.pose.position.y_val = y
        self.client.simSetVehiclePose(self.pose, True)
        print(self.pose.position)
        #time.sleep(2)
        self.takeImage()

        # Teleport to all crawling point !!
        while self.pose.position.x_val < self.boxsize:
            while self.pose.position.y_val < self.boxsize:
                self.pose.position.y_val += 10
                self.client.simSetVehiclePose(self.pose, True)
                print(self.pose.position)
                #time.sleep(1)
                self.takeImage()

            self.pose.position.x_val += 10
            self.client.simSetVehiclePose(self.pose, True)
            print(self.pose.position)
            #time.sleep(1)
            self.takeImage()

            while self.pose.position.y_val > -self.boxsize:
                self.pose.position.y_val -= 10
                self.client.simSetVehiclePose(self.pose, True)
                print(self.pose.position)
                #time.sleep(1)
                self.takeImage()

            self.pose.position.x_val += 10
            self.client.simSetVehiclePose(self.pose, True)
            print(self.pose.position)
            #time.sleep(1)
            self.takeImage()


        print("End !!!")
        end = time.time()
        print(f"{end - start: .5f} sec")

        # after hovering we need to re-enabled api control for next leg of the trip
        self.client.enableApiControl(True)

        if z < -5:
            print("descending")
            self.client.moveToPositionAsync(0, 0, -5, 2).join()

        print("landing...")
        self.client.landAsync().join()

        print("disarming.")
        self.client.armDisarm(False)

if __name__ == "__main__":
    args = sys.argv
    args.pop(0)
    arg_parser = argparse.ArgumentParser("Usage: survey boxsize stripewidth altitude")
    arg_parser.add_argument("--size", type=float, help="size of the box to survey", default=100)
    arg_parser.add_argument("--stripewidth", type=float, help="stripe width of survey (in meters)", default=10)
    arg_parser.add_argument("--altitude", type=float, help="altitude of survey (in positive meters)", default=30)
    arg_parser.add_argument("--speed", type=float, help="speed of survey (in meters/second)", default=30)
    args = arg_parser.parse_args(args)
    nav = SurveyNavigator(args)
    nav.start()