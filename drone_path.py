import setup_path 
import airsim

import sys
import time
import argparse

class SurveyNavigator:
    def __init__(self, args):
        self.boxsize = args.size # map 전체 크기
        self.stripewidth = args.stripewidth # 얼마나 멀리 운전해야하는지
        self.altitude = args.altitude
        self.velocity = args.speed
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

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
        x = -self.boxsize # map의 모서리 x 좌표
        z = -self.altitude

        print("climbing to altitude: " + str(self.altitude))
        self.client.moveToPositionAsync(0, 0, z, self.velocity).join() # 고도 이동

        print("flying to first corner of survey box")
        self.client.moveToPositionAsync(x, -self.boxsize, z, self.velocity).join() # 가장 모서리로 이동
        
        # let it settle there a bit.
        self.client.hoverAsync().join()
        time.sleep(2)

        # after hovering we need to re-enabled api control for next leg of the trip
        self.client.enableApiControl(True)

        # now compute the survey path required to fill the box 
        path = []
        distance = 0
        while x < self.boxsize:
            distance += self.boxsize 
            path.append(airsim.Vector3r(x, self.boxsize, z))
            x += self.stripewidth            
            distance += self.stripewidth 
            path.append(airsim.Vector3r(x, self.boxsize, z))
            distance += self.boxsize 
            path.append(airsim.Vector3r(x, -self.boxsize, z)) 
            x += self.stripewidth  
            distance += self.stripewidth 
            path.append(airsim.Vector3r(x, -self.boxsize, z))
            distance += self.boxsize 
        
        print("starting survey, estimated distance is " + str(distance))
        trip_time = distance / self.velocity
        print("estimated survey time is " + str(trip_time))
        try:
            result = self.client.moveOnPathAsync(path, self.velocity, trip_time, airsim.DrivetrainType.ForwardOnly, 
                airsim.YawMode(False,0), self.velocity + (self.velocity/2), 1).join()
        except:
            errorType, value, traceback = sys.exc_info()
            print("moveOnPath threw exception: " + str(value))
            pass

        print("flying back home")
        self.client.moveToPositionAsync(0, 0, z, self.velocity).join()
        
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
    arg_parser.add_argument("--size", type=float, help="size of the box to survey", default=50) # 맵 크기 입력
    arg_parser.add_argument("--stripewidth", type=float, help="stripe width of survey (in meters)", default=10)
    arg_parser.add_argument("--altitude", type=float, help="altitude of survey (in positive meters)", default=15) # 높이 입력
    arg_parser.add_argument("--speed", type=float, help="speed of survey (in meters/second)", default=50) # 속도 입력
    args = arg_parser.parse_args(args)
    nav = SurveyNavigator(args)
    nav.start()