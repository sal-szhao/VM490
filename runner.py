#!/usr/bin/env python


from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import json
import argparse

import numpy as np

import matplotlib.pyplot as plt
import seaborn as sns

# we need to import python modules from the $SUMO_HOME/tools directory
# the install instructions for SUMO should guide you through all of 
# getting this set up
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary 
import traci
import traci.constants as tc

##########################
# function to generate a routefile with a random traffic flow
# input
#  - probs : a dictionary of the probabilities of a car taking a given 
#           route appearing at each time step
#  - speed : the initial speed of cars when they spawn
#  - N     : the number of times steps for which to generate the routefile
#  - accel : the max acceleration of the cars
#  - decdel: the max deceleration of the cars
#
# output
#  - the routefile written to `routes.rou.xml`

def generate_routefile(probs, speed, N, accel, deccel):
    # make tests reproducible by uncommenting the below
    # you can pick whatever seed you want 
    # random.seed(42)

    # unpack demand per second for different routes
    pWE = probs["WE"]
    pWN = probs["WN"]
    pWS = probs["WS"]
    pEW = probs["EW"]
    pEN = probs["EN"]
    pES = probs["ES"]
    pNS = probs["NS"]
    pNE = probs["NE"]
    pNW = probs["NW"]
    pSN = probs["SN"]
    pSE = probs["SE"]
    pSW = probs["SW"]

    with open("routes.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeCar" accel="%f" decel="%f" sigma="0.0" length="5" minGap="2.5" speedDev="0" maxSpeed="%f" guiShape="passenger"/>


        <route id="WE" edges="wc ce" />
        <route id="WN" edges="wc cn" />
        <route id="WS" edges="wc cs" />

        <route id="EW" edges="ec cw" />
        <route id="EN" edges="ec cn" />
        <route id="ES" edges="ec cs" />

        <route id="NS" edges="nc cs" />
        <route id="NE" edges="nc ce" />
        <route id="NW" edges="nc cw" />

        <route id="SN" edges="sc cn" />
        <route id="SE" edges="sc ce" />
        <route id="SW" edges="sc cw" />

        """ % (accel, deccel, speed),file=routes)
        vehNr = 0

        # loop through the desired number of timesteps
        for i in range(N):
            '''
            # randomly sample each route probability to see
            # if a car appears and if so write it to the xml file
            if random.uniform(0, 1) < pWE:
                print('    <vehicle id="WE_%i" type="typeCar" route="WE" depart="%i" departSpeed="%f"/>' % (
                    vehNr, i, speed), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pWN:
                print('    <vehicle id="WN_%i" type="typeCar" route="WN" depart="%i" departSpeed="%f"/>' % (
                    vehNr, i,  speed), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pWS:
                print('    <vehicle id="WS_%i" type="typeCar" route="WS" depart="%i" departSpeed="%f"/>' % (
                    vehNr, i,  speed), file=routes)
                vehNr += 1


            if random.uniform(0, 1) < pEW:
                print('    <vehicle id="EW_%i" type="typeCar" route="EW" depart="%i" departSpeed="%f"/>' % (
                    vehNr, i,  speed), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pEN:
                print('    <vehicle id="EN_%i" type="typeCar" route="EN" depart="%i" departSpeed="%f"/>' % (
                    vehNr, i,  speed), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pES:
                print('    <vehicle id="ES_%i" type="typeCar" route="ES" depart="%i" departSpeed="%f"/>' % (
                    vehNr, i,  speed), file=routes)
                vehNr += 1


            if random.uniform(0, 1) < pNS:
                print('    <vehicle id="NS_%i" type="typeCar" route="NS" depart="%i" color="1,0,0" departSpeed="%f"/>' % (
                    vehNr, i,  speed), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pNE:
                print('    <vehicle id="NE_%i" type="typeCar" route="NE" depart="%i" color="1,0,0" departSpeed="%f"/>' % (
                    vehNr, i,  speed), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pNW:
                print('    <vehicle id="NW_%i" type="typeCar" route="NW" depart="%i" color="1,0,0" departSpeed="%f"/>' % (
                    vehNr, i,  speed), file=routes)
                vehNr += 1


            if random.uniform(0, 1) < pSN:
                print('    <vehicle id="SN_%i" type="typeCar" route="SN" depart="%i" color="1,0,0" departSpeed="%f"/>' % (
                    vehNr, i,  speed), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pSE:
                print('    <vehicle id="SE_%i" type="typeCar" route="SE" depart="%i" color="1,0,0" departSpeed="%f"/>' % (
                    vehNr, i,  speed), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pSW:
                print('    <vehicle id="SW_%i" type="typeCar" route="SW" depart="%i" color="1,0,0" departSpeed="%f"/>' % (
                    vehNr, i,  speed), file=routes)
                vehNr += 1
            '''
            sc_pre=chr(65+int(vehNr/10))
            # Generate one vehicle in one direction one by one.
            # randomly sample each route probability to see
            # if a car appears and if so write it to the xml file
            if i % 24 == 0:
                print('    <vehicle id="%c%i" type="typeCar" route="WE" depart="%i" departSpeed="%f"/>' % (
                    sc_pre,vehNr, i, speed), file=routes)
                vehNr += 1
            elif i % 24 == 8:
                print('    <vehicle id="%c%i" type="typeCar" route="WN" depart="%i" departSpeed="%f"/>' % (
                    sc_pre,vehNr, i,  speed), file=routes)
                vehNr += 1
            elif i % 24 == 16:
                print('    <vehicle id="%c%i" type="typeCar" route="WS" depart="%i" departSpeed="%f"/>' % (
                    sc_pre,vehNr, i,  speed), file=routes)
                vehNr += 1


            elif i % 24 == 2:
                print('    <vehicle id="%c%i" type="typeCar" route="EW" depart="%i" departSpeed="%f"/>' % (
                    sc_pre,vehNr, i,  speed), file=routes)
                vehNr += 1
            elif i % 24 == 10:
                print('    <vehicle id="%c%i" type="typeCar" route="EN" depart="%i" departSpeed="%f"/>' % (
                    sc_pre,vehNr, i,  speed), file=routes)
                vehNr += 1
            elif i % 24 == 18:
                print('    <vehicle id="%c%i" type="typeCar" route="ES" depart="%i" departSpeed="%f"/>' % (
                    sc_pre,vehNr, i,  speed), file=routes)
                vehNr += 1


            elif i % 24 == 20:
                print('    <vehicle id="%c%i" type="typeCar" route="NS" depart="%i" color="1,0,0" departSpeed="%f"/>' % (
                    sc_pre,vehNr, i,  speed), file=routes)
                vehNr += 1
            elif i % 24 == 4:
                print('    <vehicle id="%c%i" type="typeCar" route="NE" depart="%i" color="1,0,0" departSpeed="%f"/>' % (
                    sc_pre,vehNr, i,  speed), file=routes)
                vehNr += 1
            elif i % 24 == 12:
                print('    <vehicle id="%c%i" type="typeCar" route="NW" depart="%i" color="1,0,0" departSpeed="%f"/>' % (
                    sc_pre,vehNr, i,  speed), file=routes)
                vehNr += 1


            elif i % 24 == 6:
                print('    <vehicle id="%c%i" type="typeCar" route="SN" depart="%i" color="1,0,0" departSpeed="%f"/>' % (
                    sc_pre,vehNr, i,  speed), file=routes)
                vehNr += 1
            elif i % 24 == 22:
                print('    <vehicle id="%c%i" type="typeCar" route="SE" depart="%i" color="1,0,0" departSpeed="%f"/>' % (
                    sc_pre,vehNr, i,  speed), file=routes)
                vehNr += 1
            elif i % 24 == 14:
                print('    <vehicle id="%c%i" type="typeCar" route="SW" depart="%i" color="1,0,0" departSpeed="%f"/>' % (
                    sc_pre,vehNr, i,  speed), file=routes)
                vehNr += 1

        print("</routes>", file=routes)

########################
# a class that implements the orbit based
# first come first serve intersection control 
# algorithm
class FirstComeFirstServe(object):

    def __init__(self, radius, pad, topSpeed):

        # the radius of traffic that the algorithm controls
        self.radius = radius

        # the list of times that each intersection point 
        # between orbits was last occupied
        self.pointTimes = None
        # store the initial to reset after simulation runs
        self.initPointTimes = None

        # a dictionary of the points crossed and linear 
        # distance to those points (as tuples) for each orbit
        self.pathTable = None

        # the desired time between cars passing through the same point
        self.pad = pad
        # top speed of cars on the road
        self.topSpeed = topSpeed
        # list to store cars processed by the system
        self.processedIDs = []
        # list to store cars that have passed through the intersection
        self.exitedIDs = []
        # store the roads entering the intersection
        self.inRoads = []
        # the length of the roads - MUST agree with the actual length
        # from the network file
        self.roadDist = None

        # store the turning direction of the last car to 
        # pass through each point
        self.pointLast = None
        # store the initial to reset after simulation runs
        self.initPointLast = None

        # the distance from the end of the road at which the 
        # intersection starts - MUST agree with the network file
        self.intersectionStart = None
        # dictionary of the max speeds allowed on each orbit
        # MUST agree with the network file
        self.intersectionSpeedTable = None
        # dictionary of the linear distances of each orbit
        # MUST agree with the network file
        self.intersectionDistTable = None
        # dictionary of what type of turn each route makes
        self.turnList = None

    def setInroads(self, inRoads):
        self.inRoads = inRoads

    def setRoadDist(self, roadDist):
        self.roadDist = roadDist

    def setPointTimes(self, pointTimes):
        self.pointTimes = pointTimes
        self.initPointTimes = list(pointTimes)

    def setPointLast(self, pointLast):
        self.pointLast = pointLast
        self.initPointLast = list(pointLast)

    def setPathTable(self, pathTable):
        self.pathTable = pathTable

    def setIntersectionStart(self, intersectionStart):
        self.intersectionStart = intersectionStart

    def setIntersectionSpeedTable(self, intersectionSpeedTable):
        self.intersectionSpeedTable = intersectionSpeedTable

    def setIntersectionDistTable(self, intersectionDistTable):
        self.intersectionDistTable = intersectionDistTable

    def setTurnList(self, turnList):
        self.turnList = turnList

    ##########################
    # function to control the cars for the next time step
    # input
    #  - cars: the dictionary of cars from the junction subscription
    #          includes the lane position, road id, route id, and speed for each car.
    #  - step: the current simulation time step
    #
    # output
    #  - speeds: a dictionary of cars and the speeds they should be set to
    #  - modes : a dictionary of cars and the driving function speed modes they should be set to
    def controlNextStep(self, cars, step):
        if not cars:
            return {}, {}

        speeds = {}
        modes = {}
        # loop through the cars we are currently controlling 
        for car in cars:

            # if the car has passed into the intersection and we have not processed it as exited 
            # then set it back to full speed and normal car following mode and mark it as exited
            if cars[car][tc.VAR_ROAD_ID] not in self.inRoads and car not in self.exitedIDs:
                speeds[car] = self.topSpeed
                self.exitedIDs += [car]
                modes[car] = 31
            # if the car is on the roads going into the intersection and has not been processed 
            # find its correct speed and mark it as processed
            if cars[car][tc.VAR_ROAD_ID] in self.inRoads and car not in self.processedIDs:
                self.processedIDs += [car]
                modes[car] = 6
                # calculate the distance from the car to the intersection
                dist = self.roadDist - cars[car][tc.VAR_LANEPOSITION] - self.intersectionStart
                path = self.pathTable[cars[car][tc.VAR_ROUTE_ID]]
                intersectionSpeed = self.intersectionSpeedTable[cars[car][tc.VAR_ROUTE_ID]]


                # loop through all of the points in the car's orbit and their linear 
                # distances to find the necessary delay for the car
                maxDelay = 0
                for point, intersectionDist in path:
                    # calculate the time to the point always going the speed limit
                    timeToPoint = dist / self.topSpeed + intersectionDist / intersectionSpeed
                    extraDelay = 0

                    #################################
                    # How to add extra padding for certain combinations of turns going through a point
                    # e.g if the last car turned left and the next car is turning right
                    # have the next car wait an extra 2 seconds
                    # "L" - left turn, "R" - right turn, "S" - straight 
                    # using the expression below as a template put the first turn of the combination in the 
                    # comparison on the left and the second in the comparison on the right and the extra 
                    # amount of padding you want as `extraDelay`
                    # the example above would become 
                    #
                    # if self.pointLast[point] == "L" and self.turnList[cars[car][tc.VAR_ROUTE_ID]] == "R":
                    #     extraDelay = 2

                    # find the required delay given the time to point and the last time it was occupied
                    # and take the running max
                    maxDelay = max(self.pointTimes[point] - step - timeToPoint + extraDelay, maxDelay)

                # calculate the speed such that when the car arrives at the intersection it has 
                # implemented the needed delay
                delaySpeed = dist / (dist / self.topSpeed + maxDelay)

                # if there is a car in front of the given car in the same lane calculate the 
                # max speed such that when the car in front clears the intersection the current 
                # car has a 7.5 meter safety gap (can set to value other than 7.5)
                gapSpeed = self.topSpeed
                if self.pointTimes[path[0][0]] - step > 0:
                    gapSpeed = (dist - 7.5) / (self.pointTimes[path[0][0]] - step)

                # take the min of the two speeds as this will then be safe
                speed = min(delaySpeed, gapSpeed)

                speeds[car] = speed
                pad = self.pad

                # loop through each point on the car's orbit and update its occupancy time
                for point, intersectionDist in path:

                    # if the car must accelerate after it gets to the intersection find the time to the 
                    # point taking into account acceleration otherwise assume constant speed
                    if speed < intersectionSpeed:
                        intersectionTime = (-speed + (speed**2 + 2*intersectionDist*ACCEL)**0.5)/ ACCEL
                    else:
                        intersectionTime = intersectionDist / intersectionSpeed
                    timeToPoint = dist / speed + intersectionTime + pad + step

                    # update point data
                    self.pointTimes[point] = timeToPoint
                    self.pointLast[point] = self.turnList[cars[car][tc.VAR_ROUTE_ID]]

        return speeds, modes

    ####################
    # method to reset the instance after a run of the simulation
    def reset(self):
        self.pointTimes = list(self.initPointTimes)
        self.pointLast = list(self.initPointLast)
        self.processedIDs = []
        self.exitedIDs = []


########################
# a class that implements the latency-resilient tracking
# based on the first come first serve intersection control 
# algorithm
class LRTracking(object):

    def __init__(self, radius, pad, topSpeed):

        # the radius of traffic that the algorithm controls
        self.radius = radius

        # the list of times that each intersection point 
        # between orbits was last occupied
        self.pointTimes = None
        # store the initial to reset after simulation runs
        self.initPointTimes = None

        # a dictionary of the points crossed and linear 
        # distance to those points (as tuples) for each orbit
        self.pathTable = None

        # the desired time between cars passing through the same point
        self.pad = pad
        # top speed of cars on the road
        self.topSpeed = topSpeed
        # list to store cars processed by the system
        self.processedIDs = []
        # list to store cars that have passed through the intersection
        self.exitedIDs = []
        # store the roads entering the intersection
        self.inRoads = []
        # the length of the roads - MUST agree with the actual length
        # from the network file
        self.roadDist = None

        # store the turning direction of the last car to 
        # pass through each point
        self.pointLast = None
        # store the initial to reset after simulation runs
        self.initPointLast = None

        # the distance from the end of the road at which the 
        # intersection starts - MUST agree with the network file
        self.intersectionStart = None
        # dictionary of the max speeds allowed on each orbit
        # MUST agree with the network file
        self.intersectionSpeedTable = None
        # dictionary of the linear distances of each orbit
        # MUST agree with the network file
        self.intersectionDistTable = None
        # dictionary of what type of turn each route makes
        self.turnList = None

        # dictionary of time cars are driven
        self.driveTime = {}
        # dictionary of delay time for the cars
        self.delayTime = {} 
        # reference speed of the car
        self.refSpeed = self.topSpeed * 0.8

    def setInroads(self, inRoads):
        self.inRoads = inRoads

    def setRoadDist(self, roadDist):
        self.roadDist = roadDist

    def setPointTimes(self, pointTimes):
        self.pointTimes = pointTimes
        self.initPointTimes = list(pointTimes)

    def setPointLast(self, pointLast):
        self.pointLast = pointLast
        self.initPointLast = list(pointLast)

    def setPathTable(self, pathTable):
        self.pathTable = pathTable

    def setIntersectionStart(self, intersectionStart):
        self.intersectionStart = intersectionStart

    def setIntersectionSpeedTable(self, intersectionSpeedTable):
        self.intersectionSpeedTable = intersectionSpeedTable

    def setIntersectionDistTable(self, intersectionDistTable):
        self.intersectionDistTable = intersectionDistTable

    def setTurnList(self, turnList):
        self.turnList = turnList


    ##########################
    # function to control the cars for the next time step
    # input
    #  - cars: the dictionary of cars from the junction subscription
    #          includes the lane position, road id, route id, and speed for each car.
    #  - step: the current simulation time step
    #
    # output
    #  - speeds: a dictionary of cars and the speeds they should be set to
    #  - modes : a dictionary of cars and the driving function speed modes they should be set to
    def controlNextStep(self, cars, step):
        if not cars:
            init_steps = {}  ## used for recording the initial time step when the car is generated.
            return {}, {}

        speeds = {}
        modes = {}
        index=0
        ref_drive_time={}
        # loop through the cars we are currently controlling 
        for car in cars:
            # Update the reference distance for each car each time step
            # Here we take the reference speed as 0.8 * topSpeed
            if car not in self.driveTime.keys():
                self.driveTime[car] = 0
                self.delayTime[car] = 0
                continue
            else:
                self.driveTime[car] += STEP_SIZE
            ## Alwaays track one car
            # allCars = traci.vehicle.getIDList()
            if car == list(cars.keys())[0]:
                # print(traci.vehicle.getLeader(allCars[0]))
                # print("current-gap: " + str(traci.vehicle.getMinGap(allCars[0])))
                # print("current-speed: " + str(traci.vehicle.getSpeed(car)))
                # print("ref: " + str(self.refDist[car]))
                # print("dist: " + str(traci.vehicle.getDistance(car)))
                # print("drive time: " + str(self.driveTime[car]))
                # print("delay time: " + str(self.delayTime[car]))
                pass

            # Always consider the delaySpeed of the cars at all timeSteps.
            curr_speed = traci.vehicle.getSpeed(car)

            # Redefine the delaySpeed, the delaySpeed should depend on reference speed and position.
            # Total time spent should be STEP_SIZE * STEP
            # refPos = self.refSpeed * (self.driveTime[car] - self.delayTime[car])
            if index!=0 and ref_drive_time[index-1]>1.5 and self.driveTime[car]!=0:
                ref_drive_time[index]=ref_drive_time[index-1]-1.5
            else:
                ref_drive_time[index]=self.driveTime[car]
            refPos = self.refSpeed * ref_drive_time[index]
            refAccl = -(traci.vehicle.getDistance(car) - refPos) / (STEP_SIZE ** 2) - 2 * (curr_speed - self.refSpeed) / STEP_SIZE

            # refAccel should not exceed the maximum accleration.
            if refAccl > ACCEL:
                refAccl = ACCEL
            elif refAccl < -DECCEL:
                refAccl = -DECCEL

            delaySpeed = curr_speed + refAccl * STEP_SIZE
            speeds[car] = delaySpeed
            if index>0:
                print(index)
                print("front car time:"+ str(ref_drive_time[index-1]))
                print("now car drive time: " + str(self.driveTime[car]))
                print("accl: "+ str(refAccl))
                print("now speed:" + str(curr_speed))
                print("top_speed:" + str(self.topSpeed))
                print("first term 1: " + str(traci.vehicle.getDistance(car)))
                print("first term 2: " + str(refPos))
                print("second term: " + str(2 * (curr_speed - self.refSpeed) / STEP_SIZE))
                print("tracking error: " + str(traci.vehicle.getDistance(car) - refPos))
                print(" ")
            #     dist / (dist / self.topSpeed + maxDelay)
            index+=1
            # if the car has passed into the intersection and we have not processed it as exited 
            # then set it back to full speed and normal car following mode and mark it as exited
            if cars[car][tc.VAR_ROAD_ID] not in self.inRoads and car not in self.exitedIDs:
                speeds[car] = self.refSpeed
                self.exitedIDs += [car]
                modes[car] = 31
            # if the car is on the roads going into the intersection and has not been processed 
            # find its correct speed and mark it as processed
            if cars[car][tc.VAR_ROAD_ID] in self.inRoads and car not in self.processedIDs:
                self.processedIDs += [car]
                modes[car] = 6
                # calculate the distance from the car to the intersection
                dist = self.roadDist - cars[car][tc.VAR_LANEPOSITION] - self.intersectionStart
                path = self.pathTable[cars[car][tc.VAR_ROUTE_ID]]
                intersectionSpeed = self.intersectionSpeedTable[cars[car][tc.VAR_ROUTE_ID]]

                # loop through all of the points in the car's orbit and their linear 
                # distances to find the necessary delay for the car
                maxDelay = 0
                for point, intersectionDist in path:
                    # calculate the time to the point always going the speed limit
                    timeToPoint = dist / self.refSpeed + intersectionDist / intersectionSpeed
                    extraDelay = 0

                    #################################
                    # How to add extra padding for certain combinations of turns going through a point
                    # e.g if the last car turned left and the next car is turning right
                    # have the next car wait an extra 2 seconds
                    # "L" - left turn, "R" - right turn, "S" - straight 
                    # using the expression below as a template put the first turn of the combination in the 
                    # comparison on the left and the second in the comparison on the right and the extra 
                    # amount of padding you want as `extraDelay`
                    # the example above would become 
                    #
                    # if self.pointLast[point] == "L" and self.turnList[cars[car][tc.VAR_ROUTE_ID]] == "R":
                    #     extraDelay = 2

                    # find the required delay given the time to point and the last time it was occupied
                    # and take the running max
                    maxDelay = max(self.pointTimes[point] - step - timeToPoint + extraDelay, maxDelay)
                    self.delayTime[car] = maxDelay


                # if there is a car in front of the given car in the same lane calculate the 
                # max speed such that when the car in front clears the intersection the current 
                # car has a 7.5 meter safety gap (can set to value other than 7.5)
                d = 2
                beta = 0.2
                safety_gap = d + beta * traci.vehicle.getSpeed(car)

                gapSpeed = self.refSpeed
                if self.pointTimes[path[0][0]] - step > 0:
                    gapSpeed = (dist - safety_gap) / (self.pointTimes[path[0][0]] - step)

                # take the min of the two speeds as this will then be safe
                #speeds[car] = min(speeds[car], gapSpeed)

                pad = self.pad

                # loop through each point on the car's orbit and update its occupancy time
                for point, intersectionDist in path:

                    # if the car must accelerate after it gets to the intersection find the time to the 
                    # point taking into account acceleration otherwise assume constant speed
                    if speeds[car] < intersectionSpeed:
                        intersectionTime = (-speeds[car] + (speeds[car]**2 + 2*intersectionDist*ACCEL)**0.5) / ACCEL
                    else:
                        intersectionTime = intersectionDist / intersectionSpeed
                    timeToPoint = dist / speeds[car] + intersectionTime + pad + step
                    #print("first" + str(dist/speed))
                    #print(intersectionTime)

                    # update point data
                    self.pointTimes[point] = timeToPoint
                    self.pointLast[point] = self.turnList[cars[car][tc.VAR_ROUTE_ID]]
                
            # Add random acceleration to the car.
            speeds[car] += random.uniform(-0.05, 0.05)
        return speeds, modes

    ####################
    # method to reset the instance after a run of the simulation
    def reset(self):
        self.pointTimes = list(self.initPointTimes)
        self.pointLast = list(self.initPointLast)
        self.processedIDs = []
        self.exitedIDs = []


########################
# a class that implements queuing intersection control algorithm
# NB: Does NOT work with turning traffic
class Queuing(object):

    def __init__(self, radius, topSpeed):

        # the radius of traffic that the algorithm controls
        self.radius = radius
        # top speed of cars on the road
        self.topSpeed = topSpeed
        # stores the east and west roads into the intersection
        self.EWRoads = []
        # stores the north and south roads into the intersection
        self.NSRoads = []
        # the length of the roads - MUST agree with the actual length
        # from the network file
        self.roadDist = None
        # the distance from the end of the road to the position where 
        # intersection starts - MUST agree with the network file
        self.intersectionStart = None

        # are we currently letting through a queue of cars
        self.clearing = False
        # the queue we are currently letting through
        self.clearQueue = []
        # the queue of cars approaching the intersection from the east and west
        self.EWQueue = []
        # the queue of cars approaching the intersection from the north and south
        self.NSQueue = []

    def setEWRoads(self, EWRoads):
        self.EWRoads = EWRoads

    def setNSRoads(self, NSRoads):
        self.NSRoads = NSRoads

    def setRoadDist(self, roadDist):
        self.roadDist = roadDist 

    def setIntersectionStart(self, intersectionStart):
        self.intersectionStart = intersectionStart


    ##########################
    # function to control the cars for the next time step
    # input
    #  - cars: the dictionary of cars from the junction subscription
    #          includes the lane position, road id, route id, and speed for each car.
    #  - step: the current simulation time step
    #
    # output
    #  - speeds: a dictionary of cars and the speeds they should be set to
    #  - modes : a dictionary of cars and the driving function speed modes they should be set to
    def controlNextStep(self, cars, step):
        if not cars:
            return {}, {}

        speeds = {}

        self.EWQueue = []
        self.NSQueue = []

        # loop through the cars we are controlling
        for car in cars:
            # if the car is approaching from the east or west and not currently in the 
            # queue we are letting through add it to the east/west queue
            if cars[car][tc.VAR_ROAD_ID] in self.EWRoads and car not in self.clearQueue:
                self.EWQueue += [car]

            # if the car is approaching from the north or south and not currently in the 
            # queue we are letting through add it to the north/south queue
            if cars[car][tc.VAR_ROAD_ID] in self.NSRoads and car not in self.clearQueue:
                self.NSQueue += [car]
            # calculate the distance from the car to the intersection
            dist = self.roadDist - cars[car][tc.VAR_LANEPOSITION] - self.intersectionStart
            # if the car is within breaking distance and in either of the waiting queues have it stop
            if dist < (self.topSpeed / DECCEL) * self.topSpeed and (car in self.EWQueue or car in self.NSQueue):
                speeds[car] = 0

        # if we are currently letting a queue of cars through the intersection 
        # have the cars in that clearing queue accelerate to top speed
        if self.clearing:
            done = True
            for car in cars:
                if car in self.clearQueue and (cars[car][tc.VAR_ROAD_ID] in self.EWRoads or cars[car][tc.VAR_ROAD_ID] in self.NSRoads):
                    speeds[car] = self.topSpeed
                    done = False
            # if all the cars in the clearing queue have passed through the intersection 
            # then we are done clearing and reset the queue
            if done:
                self.clearQueue = []
                self.clearing = False

        # if we are not currently clearing a queue then pick the 
        # longest of the waiting queues and start clearing that one
        if not self.clearing:
            if len(self.EWQueue) > len(self.NSQueue):
                self.clearQueue = self.EWQueue
                self.EWQueue = []
            else:
                self.clearQueue = self.NSQueue
                self.NSQueue = []
            self.clearing = True
        return speeds, {}

    ####################
    # method to reset the instance after a run of the simulation
    def reset(self):
        self.clearing = False
        self.clearQueue = []


########################
# dummy class for the intersection control algorithm
# NB: Use as a placeholder when testing SUMO defaults
class Unsupervised(object):
    def __init__(self):
        self.radius = 0

    def controlNextStep(self, cars, step):
        return {}, {}

    def reset(self):
        pass


##########################
# function to drive a car based on a custom car following model
# input
#  - car        : the id of the car that is being driven
#  - targetSpeed: the current set speed of the car
#  - mode       : the speed mode of the car following model
#                 NB: these are meant to have similar meanings to
#                     the same numbers in SUMO
#
# output
#  -  the car is driven according to the car following model
def drive(car, targetSpeed, mode):

    # make sure the target speed does not accede the allowed speed for that road
    targetSpeed = min(targetSpeed, traci.vehicle.getAllowedSpeed(car))

    if mode == 6:
        # depending on if the car is faster or slower than the target speed 
        # either accelerate or decelerate
        currentSpeed = traci.vehicle.getSpeed(car)
        speed = currentSpeed
        if targetSpeed > currentSpeed:
            speed = min(STEP_SIZE * ACCEL + currentSpeed, targetSpeed)
        if targetSpeed < currentSpeed:
            speed = max(currentSpeed - STEP_SIZE * DECCEL, targetSpeed)
        traci.vehicle.setSpeed(car, speed)

    if mode == 31:
        leader = traci.vehicle.getLeader(car)

        # if there is a car in front we use the car following model 
        # use the car following model from Xi Xiong's platooning code
        # NB: this does not take into account acceleration and deceleration 
        # values and can therefore over break and over accelerate
        if leader:
            leaderId = leader[0]
            leaderPos = traci.vehicle.getPosition(leaderId)
            leaderSpeed = traci.vehicle.getSpeed(leaderId)
            carPos = traci.vehicle.getPosition(car)
            carSpeed = traci.vehicle.getSpeed(car)

            posDiff = ((leaderPos[0] - carPos[0])**2 + (leaderPos[1] - carPos[1])**2) ** 0.5

            #car following model
            gapSpeed = leaderSpeed + (posDiff-15)*0.5

            # still make sure that we do not go over the target speed
            speed = min(targetSpeed, gapSpeed)

            traci.vehicle.setSpeed(car, speed)

        # if there is no car in front we just accelerate or decelerate to the target speed
        else:
            currentSpeed = traci.vehicle.getSpeed(car)
            speed = currentSpeed
            if targetSpeed > currentSpeed:
                speed = min(STEP_SIZE * 0.8 + currentSpeed, targetSpeed)
            if targetSpeed < currentSpeed:
                speed = max(currentSpeed - STEP_SIZE * 4.5, targetSpeed)
            traci.vehicle.setSpeed(car, speed)



##########################
# function to run a simulation
# input
#  - algo     : the intersection control algorithm to use in the simulation
#  - dataName : (optional) a prefix to add to the name of the file saved
#  - routefile: (not passed as an argument) the traffic flow will be read from `routes.rou.xml`
#
# output
#  -  the full trip info will be written to `DATA_FOLDER+ / +dataName + tripinfo.xml`
#     and the log data will be written to `log.txt`
def run(algo, dataName=""):
    # start SUMO
    traci.start([sumoBinary, "-c", CONF_PATH,
                                         "--tripinfo-output", DATA_FOLDER+ "/"+dataName+"tripinfo.xml", "--duration-log.statistics","--log", "log.txt"])

    step = 0
    # subscribe to get data from the intersection up to the radius of the control algorithm
    traci.junction.subscribeContext("c", tc.CMD_GET_VEHICLE_VARIABLE, algo.radius, [tc.VAR_LANEPOSITION, tc.VAR_ROAD_ID, tc.VAR_ROUTE_ID, tc.VAR_SPEED])
    if params["CUSTOM_FOLLOW"]:
        initCars = {}
    # main loop while there are active cars
    while traci.simulation.getMinExpectedNumber() > 0:

        # if we are using a custom car following model when new cars spawn
        # we turn off the default car following model and add them to the cars we are tracking
        if params["CUSTOM_FOLLOW"]:
            allCars = traci.vehicle.getIDList()

            for car in allCars:
                
                ## Add random speed (move to drive).
                #traci.vehicle.setSpeed(car, traci.vehicle.getSpeed(car) + random.uniform(-0.5, 0.5))

                ## Parameters fpr safety constraint, maybe added to cmd arguments later (move to drive).
                d = 2
                beta = 0.05
                traci.vehicle.setMinGap(car, d + beta * traci.vehicle.getSpeed(car))
                ## print("Supposed min gap: " + str(d + beta * traci.vehicle.getSpeed(car)))

                if car not in initCars:
                    traci.vehicle.setSpeedMode(car, 0)
                    initCars[car] = {"targetSpeed": SPEED, "speedMode": 31}

        # get the cars to control from the subscription
        cars = traci.junction.getContextSubscriptionResults("c")

        # run the intersection controller for this time step
        control, modes = algo.controlNextStep(cars, step)

        # if we are not using a custom car following model then 
        # use SUMO to update the speeds and speed modes
        if not params["CUSTOM_FOLLOW"]:
            
            for car in modes:
                traci.vehicle.setSpeedMode(car, modes[car])
            
            for car in control:
                d = 2
                beta = 0.05
                traci.vehicle.setMinGap(car, d + beta * traci.vehicle.getSpeed(car))
                traci.vehicle.setSpeed(car, control[car])

        # if we are using a custom car following model then update the 
        # target speeds and modes of the cars and then apply our custom 
        # driving function to each of the cars
        if params["CUSTOM_FOLLOW"]:
            for car in modes:
                initCars[car]["speedMode"] = modes[car]
            for car in control:
                initCars[car]["targetSpeed"] = control[car]

            for car in allCars:
                drive(car, initCars[car]["targetSpeed"], initCars[car]["speedMode"])

        # step the simulation forward in time
        traci.simulationStep()
        step += STEP_SIZE

    traci.close()
    sys.stdout.flush()
    algo.reset()

# parser for command line args
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    optParser.add_option("--algo", type=str, help="control algorithm for the intersectino", default="LR")
    optParser.add_option('--cf', help="boolean to use the custom car following model", default=False)

    options, args = optParser.parse_args()
    return options

##########################
# function to run repeated simulations across a sweep of traffic conditions 
# with all cars going straight
# input
#  - algo     : the intersection control algorithm to use in the simulations
#  - increment: the number of increments into which to divide the sweep 
#               over traffic densities i.e the north/south 
#               and east/west traffic densities simulated will come from a grid
#               of combinations with this many increments on each axis
#  - numRep   : the number of repetitions of the sweep to run
#  - show     : (optional) whether or not to plot a summary of the results
# 
# output
#  -  the results of each simulation will be written to a file 
#     with prefix (north/south increment, east/west increment, repetition)
def repeatedParameterSweep(algo, increment, numRep, N, show = True):

    timesLost = np.zeros((increment, increment, numRep))

    # repeat for number of repetitions
    for l in range(numRep):
        # sweep over the grid of traffic densities
        for i in range(1, increment):
            for j in range(1, increment):

                print("REP %i, params (%0.3f, %0.3f)" % (l, i / increment, j / increment))

                # generate the route file for this simulation with the current density on the sweep
                probs = {
                            "NS" : i / increment,
                            "NE" : 0,
                            "NW" : 0,

                            "SN" : i / increment,
                            "SE" : 0,
                            "SW" : 0,

                            "EW" : j / increment,
                            "EN" : 0,
                            "ES" : 0,

                            "WE" : j / increment,
                            "WN" : 0,
                            "WS" : 0
                        }

                generate_routefile(probs, SPEED, N, ACCEL, DECCEL)


                # run the simulation
                run(algo, dataName="({:d},{:d},{:d})".format(i,j,l))

                # if we are plotting a summary keep track of aggregate 
                # data from the log files
                if show:
                    log = open("log.txt", "r")
                    lines = log.readlines()
                    for line in lines:
                        if 'TimeLoss' in line:
                            break
                    avgTimeLoss = float(line[11:-1])
                    timesLost[i,j,l] = avgTimeLoss
                    for line in lines:
                        if 'DepartDelay' in line:
                            break
                    avgDepartDelay = float(line[14:-1])
                    timesLost[i,j,l] += avgDepartDelay

    # if the option is enabled plot the summary heatmap
    if show:
        for l in range(numRep):
            print(timesLost[:, :, l])

        ticklabels = [i / increment for i in range(increment)]
        timesLost = np.sum(timesLost, axis=2) / numRep
        timesLost = np.flip(timesLost, axis=0)
        ax = sns.heatmap(timesLost, xticklabels = ticklabels, yticklabels = ticklabels[::-1], cbar_kws={'label':'avg time lost per trip (seconds)'})
        plt.xlabel("Prob of car arrival in east/west direction each timestep")
        plt.ylabel("Prob of car arrival in north/south direction each timestep")
        plt.show()

##########################
# function to run repeated simulations across a sweep of traffic conditions 
# with half the cars going straight, 1/4 turning right, and 1/4 turning left
# input
#  - algo     : the intersection control algorithm to use in the simulations
#  - increment: the number of increments into which to divide the sweep 
#               over traffic densities i.e the north/south 
#               and east/west traffic densities simulated will come from a grid
#               of combinations with this many increments on each axis
#  - numRep   : the number of repetitions of the sweep to run
#  - show     : (optional) whether or not to plot a summary of the results
# 
# output
#  -  the results of each simulation will be written to a file 
#     with prefix (north/south increment, east/west increment, repetition)
def repeatedParameterSweepTurning(algo, increment, numRep, numCars, show = True):

    timesLost = np.zeros((increment, increment, numRep))

    for l in range(numRep):

        for i in range(1, increment):
            for j in range(1, increment):

                print("REP %i, params (%0.3f, %0.3f)" % (l, i / increment, j / increment))

                # generate the route file for this simulation with the current density on the sweep
                probs = {
                            "NS" : 0.5 * i / increment,
                            "NE" : 0.25 * i / increment,
                            "NW" : 0.25 * i / increment,

                            "SN" : 0.5 * i / increment,
                            "SE" : 0.25 * i / increment,
                            "SW" : 0.25 * i / increment,

                            "EW" : 0.5 * j / increment,
                            "EN" : 0.25 * j / increment,
                            "ES" : 0.25 * j / increment,

                            "WE" : 0.5 * j / increment,
                            "WN" : 0.25 * j / increment,
                            "WS" : 0.25 * j / increment
                        }

                generate_routefile(probs, SPEED, numCars, ACCEL, DECCEL)

                # run the simulation
                run(algo, dataName="({:d},{:d},{:d})".format(i,j,l))

                # if we are plotting a summary keep track of aggregate 
                # data from the log files
                if show:
                    log = open("log.txt", "r")
                    lines = log.readlines()
                    for line in lines:
                        if 'TimeLoss' in line:
                            break
                    avgTimeLoss = float(line[11:-1])
                    timesLost[i,j,l] = avgTimeLoss
                    for line in lines:
                        if 'DepartDelay' in line:
                            break
                    avgDepartDelay = float(line[14:-1])
                    timesLost[i,j,l] += avgDepartDelay

    # if the option is enabled plot the summary heatmap
    if show:
        for l in range(numRep):
            print(timesLost[:, :, l])

        ticklabels = [i / increment for i in range(increment)]
        timesLost = np.sum(timesLost, axis=2) / numRep
        timesLost = np.flip(timesLost, axis=0)
        ax = sns.heatmap(timesLost, xticklabels = ticklabels, yticklabels = ticklabels[::-1], cbar_kws={'label':'avg time lost per trip (seconds)'})
        plt.xlabel("Prob of car arrival in east/west direction each timestep")
        plt.ylabel("Prob of car arrival in north/south direction each timestep")
        plt.show()

# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    print("START")

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')


# #########################
# simulation parameters
# #########################
    # the radius of control of the intersection control algorithm
    RADIUS = 99999
    # the padding used by the intersection control algorithm
    # (only applies for FCFS)
    PAD = 2.0
    # the max speed for the cars
    SPEED = 10
    # the max acceleration for the cars
    ACCEL = 0.8
    # the max deceleration for the cars
    DECCEL = 4.5
    # the increments passed to repeatedParameterSweepTurning or
    # repeatedParameterSweep when running the simulation
    INCREMENTS = 2
    # the repetitions used by repeatedParameterSweepTurning or
    # repeatedParameterSweep when running the simulation
    REPS = 2
    # the number of times steps for which to generate traffic
    # in the routefile
    GENCARS = 100
    # folder to write the complete simulation data
    DATA_FOLDER = "raw_data"
    # timestep size for the simulation
    STEP_SIZE = 0.1
    # the intersection control algorithm to use
    # the options are FCFS for orbit-based first come first serve
    # Q for queuing, and anything else for unsupervised
    # NB: see also CONF_PATH
    ## ALGO = "FCFS"

    # boolean to use repeatedParameterSweepTurning or repeatedParameterSweep
    TURNING = True
    # boolean to use the custom car following model
    # CUSTOM_FOLLOW = False



# #########################
# global values - all must agree with network file
# #########################
    # the distance of the roads
    roadDist = 1000
    # roads entering the intersection
    inRoads = ["nc", "wc", "sc", "ec"]
    # roads entering from the east/west
    EWRoads = ["ec", "wc"]
    # roads entering from the north/south
    NSRoads = ["nc", "sc"]

    # the SUMO configuration file used for the simulations
    # to run a default SUMO stop sign set to `allway.sumocfg`

    # CONF_PATH = "allway.sumocfg"
    CONF_PATH = "config.sumocfg"
# #########################

    # save the parameters to a JSON metadata file

    params =  {
        "RADIUS": RADIUS,
        "PAD": PAD,
        "SPEED": SPEED,
        "ACCEL": ACCEL,
        "DECCEL": DECCEL,
        "INCREMENTS": INCREMENTS,
        "REPS": REPS,
        "GENCARS": GENCARS,
        "ALGO": options.algo,
        "TURNING": TURNING,
        "CUSTOM_FOLLOW": options.cf
    }
    json.dump( params, open( DATA_FOLDER+"/params.json", 'w' ) )


    # initialize the current algorithm using the parameters
    # and some data about the network (see network info doc)
    # and run the correct set of simulations

    if params["ALGO"] == "Q":
        print("stat")
        algo = Queuing(RADIUS, SPEED)
        algo.setRoadDist(roadDist)
        algo.setEWRoads(EWRoads)
        algo.setNSRoads(NSRoads)
        algo.setIntersectionStart(7.20)

    elif params["ALGO"] == "FCFS":
        algo = FirstComeFirstServe(RADIUS, PAD, SPEED)
        algo.setRoadDist(roadDist)
        algo.setInroads(inRoads)

        algo.setPointTimes([0]*24)

        algo.setPointLast([None]*24)

        pathTable = {
                        "EW": [(8, 0), (7, 5.6), (6, 6.3), (5, 8.1), (4, 8.8), (3, 14.4)],
                        "EN": [(8, 0), (1, 9.03)],
                        "ES": [(8, 0), (12, 4.62), (14, 6.3), (18, 7.14), (21, 8.82), (22, 13.44)],

                        "WE": [(15, 0), (16, 5.6), (17, 6.3), (18, 8.1), (19, 8.8), (20, 14.4)],
                        "WS": [(15, 0), (22, 9.03)],
                        "WN": [(15, 0), (11, 4.62), (9, 6.3), (5, 7.14), (2, 8.82), (1, 13.44)],

                        "NS": [(0, 0), (4, 5.6), (9, 6.3), (13, 8.1), (16, 8.8), (22, 14.4)],
                        "NW": [(0, 0), (3, 9.03)],
                        "NE": [(0, 0), (2, 4.62), (6, 6.3), (10, 7.14), (12, 8.82), (20, 13.44)],

                        "SN": [(23, 0), (19, 5.6), (14, 6.3), (10, 8.1), (7, 8.8), (1, 14.4)],
                        "SE": [(23, 0), (20, 9.03)],
                        "SW": [(23, 0), (21, 4.62), (17, 6.3), (13, 7.14), (11, 8.82), (3, 13.44)]
        }

        algo.setPathTable(pathTable)

        algo.setIntersectionStart(7.20)

        speedTable = {
                        "EW": SPEED,
                        "WE": SPEED,
                        "NS": SPEED,
                        "SN": SPEED,

                        "EN": min(6.51, SPEED),
                        "WS": min(6.51, SPEED),
                        "NW": min(6.51, SPEED),
                        "SE": min(6.51, SPEED),

                        "ES": min(7.8, SPEED),
                        "WN": min(7.8, SPEED),
                        "NE": min(7.8, SPEED),
                        "SW": min(7.8, SPEED)

        }

        algo.setIntersectionSpeedTable(speedTable)

        distTable = {
                        "EW": 14.4,
                        "WE": 14.4,
                        "NS": 14.4,
                        "SN": 14.4,

                        "EN": 9.03,
                        "WS": 9.03,
                        "NW": 9.03,
                        "SE": 9.03,

                        "ES": 13.44,
                        "WN": 13.44,
                        "NE": 13.44,
                        "SW": 13.44

        }

        algo.setIntersectionDistTable(distTable)

        turnList = {
                        "EW": "S",
                        "WE": "S",
                        "NS": "S",
                        "SN": "S",

                        "EN": "R",
                        "WS": "R",
                        "NW": "R",
                        "SE": "R",

                        "ES": "L",
                        "WN": "L",
                        "NE": "L",
                        "SW": "L"

        }

        algo.setTurnList(turnList)

    elif params["ALGO"] == "LR":
        algo = LRTracking(RADIUS, PAD, SPEED)
        algo.setRoadDist(roadDist)
        algo.setInroads(inRoads)

        algo.setPointTimes([0]*24)

        algo.setPointLast([None]*24)

        pathTable = {
                        "EW": [(8, 0), (7, 5.6), (6, 6.3), (5, 8.1), (4, 8.8), (3, 14.4)],
                        "EN": [(8, 0), (1, 9.03)],
                        "ES": [(8, 0), (12, 4.62), (14, 6.3), (18, 7.14), (21, 8.82), (22, 13.44)],

                        "WE": [(15, 0), (16, 5.6), (17, 6.3), (18, 8.1), (19, 8.8), (20, 14.4)],
                        "WS": [(15, 0), (22, 9.03)],
                        "WN": [(15, 0), (11, 4.62), (9, 6.3), (5, 7.14), (2, 8.82), (1, 13.44)],

                        "NS": [(0, 0), (4, 5.6), (9, 6.3), (13, 8.1), (16, 8.8), (22, 14.4)],
                        "NW": [(0, 0), (3, 9.03)],
                        "NE": [(0, 0), (2, 4.62), (6, 6.3), (10, 7.14), (12, 8.82), (20, 13.44)],

                        "SN": [(23, 0), (19, 5.6), (14, 6.3), (10, 8.1), (7, 8.8), (1, 14.4)],
                        "SE": [(23, 0), (20, 9.03)],
                        "SW": [(23, 0), (21, 4.62), (17, 6.3), (13, 7.14), (11, 8.82), (3, 13.44)]
        }

        algo.setPathTable(pathTable)

        algo.setIntersectionStart(7.20)

        speedTable = {
                        "EW": SPEED,
                        "WE": SPEED,
                        "NS": SPEED,
                        "SN": SPEED,

                        "EN": min(6.51, SPEED),
                        "WS": min(6.51, SPEED),
                        "NW": min(6.51, SPEED),
                        "SE": min(6.51, SPEED),

                        "ES": min(7.8, SPEED),
                        "WN": min(7.8, SPEED),
                        "NE": min(7.8, SPEED),
                        "SW": min(7.8, SPEED)

        }

        algo.setIntersectionSpeedTable(speedTable)

        distTable = {
                        "EW": 14.4,
                        "WE": 14.4,
                        "NS": 14.4,
                        "SN": 14.4,

                        "EN": 9.03,
                        "WS": 9.03,
                        "NW": 9.03,
                        "SE": 9.03,

                        "ES": 13.44,
                        "WN": 13.44,
                        "NE": 13.44,
                        "SW": 13.44

        }

        algo.setIntersectionDistTable(distTable)

        turnList = {
                        "EW": "S",
                        "WE": "S",
                        "NS": "S",
                        "SN": "S",

                        "EN": "R",
                        "WS": "R",
                        "NW": "R",
                        "SE": "R",

                        "ES": "L",
                        "WN": "L",
                        "NE": "L",
                        "SW": "L"

        }

        algo.setTurnList(turnList)


    else:
        algo = Unsupervised()


    if not TURNING:
        repeatedParameterSweep(algo, INCREMENTS, REPS, GENCARS)
    if TURNING:
        repeatedParameterSweepTurning(algo, INCREMENTS, REPS, GENCARS)


    
