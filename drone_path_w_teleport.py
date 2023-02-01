import setup_path 
import airsim

import sys
import time
import argparse

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

        # Teleport to all crawling point !!
        while self.pose.position.x_val < self.boxsize:
            while self.pose.position.y_val < self.boxsize:
                self.pose.position.y_val += 10
                self.client.simSetVehiclePose(self.pose, True)
                print(self.pose.position)
                #time.sleep(1)

            self.pose.position.x_val += 10
            self.client.simSetVehiclePose(self.pose, True)
            print(self.pose.position)
            #time.sleep(1)

            while self.pose.position.y_val > -self.boxsize:
                self.pose.position.y_val -= 10
                self.client.simSetVehiclePose(self.pose, True)
                print(self.pose.position)
                #time.sleep(1)

            self.pose.position.x_val += 10
            self.client.simSetVehiclePose(self.pose, True)
            print(self.pose.position)
            #time.sleep(1)


        print("End !!!")

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