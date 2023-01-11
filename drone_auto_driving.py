import setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2
import sys
import time

print("""This script is designed to fly on the streets of the Neighborhood environment
and assumes the unreal position of the drone is [160, -1500, 120].""")

client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

print("arming the drone...")
client.armDisarm(True)

state = client.getMultirotorState()
if state.landed_state == airsim.LandedState.Landed:
    print("taking off...")
    client.takeoffAsync().join()
else:
    client.hoverAsync().join()

time.sleep(1)

state = client.getMultirotorState()
if state.landed_state == airsim.LandedState.Landed:
    print("take off failed...")
    sys.exit(1)

# AirSim uses NED coordinates so negative axis is up.
# z of -5 is 5 meters above the original launch point.
z = -1
print("make sure we are hovering at {} meters...".format(-z))
client.moveToZAsync(z, 1).join()

# see https://github.com/Microsoft/AirSim/wiki/moveOnPath-demo


for i in range(6):
    print("rotation" + str(i))    

    client.moveByVelocityZAsync(0, 0, 0, 1, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 60 + i*60))
    time.sleep(3)

    print('take images')
    # get camera images from the drone
    responses = client.simGetImages([
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

        filename = os.path.join(tmp_dir, f"{i}_{idx}")

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


# drone will over-shoot so we bring it back to the start point before landing.
client.moveToPositionAsync(0,0,z,1).join()
print("landing...")
client.landAsync().join()
print("disarming...")
client.armDisarm(False)
client.enableApiControl(False)
print("done.")