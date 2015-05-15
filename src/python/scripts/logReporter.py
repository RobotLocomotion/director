import os
import sys
import time
import lcm
import numpy as np
import matplotlib.pyplot as plt
import datetime as dt
from ddapp import lcmspy as spy
import scipy.signal as sig

def sizeof_fmt(num, suffix='B'):
    for unit in ['','Ki','Mi','Gi','Ti','Pi','Ei','Zi']:
        if abs(num) < 1024.0:
            return "%3.1f%s%s" % (num, unit, suffix)
        num /= 1024.0
    return "%.1f%s%s" % (num, 'Yi', suffix)



class LCMLogAnalyzer(object):
    def __init__(self, logFile):
        self.logFile = logFile
        self.jointVelocityTimes = list()
        self.jointVelocityNorms = list()
        self.jointVelocities = list()
        self.batteryTimes = list()
        self.batteryPercentage = list()
        self.pressureTimes = list()
        self.pressureReadings = list()
        self.slidingWindowWidth = 100 
        self.movementThreshold = 0.4

    def parseLog(self):
        log = lcm.EventLog(self.logFile, 'r')
        print 'Log size: ' + sizeof_fmt(log.size())
        log.seek(0)
        
        while True:
            event = log.read_next_event()
            if not event:
                break
            timestamp = event.timestamp
            if event.channel == 'EST_ROBOT_STATE':
                msg = spy.decodeMessage(event.data)
                self.jointVelocityTimes.append(timestamp)
                self.jointVelocityNorms.append(np.linalg.norm(msg.joint_velocity))
            elif event.channel == 'ATLAS_BATTERY_DATA':
                msg = spy.decodeMessage(event.data)
                self.batteryTimes.append(timestamp)
                self.batteryPercentage.append(msg.remaining_charge_percentage)
            elif event.channel == 'ATLAS_STATUS':
                msg = spy.decodeMessage(event.data)
                self.pressureTimes.append(timestamp)
                self.pressureReadings.append(msg.pump_supply_pressure)

        print 'parsed ' + str(len(self.jointVelocityNorms)) + ' robot states'
        print 'parsed ' + str(len(self.batteryPercentage)) + ' battery states'
        print 'parsed ' + str(len(self.pressureReadings)) + ' pump readings'
        
    def movingAverage(self, x):
        N = self.slidingWindowWidth
        return np.convolve(x, np.ones((N,))/N)[(N-1):]


    def plotPump(self, tmin):
        plt.title('Pump')
        plt.ylabel('System Pressure (PSI)')
        scaledPressureTimes = (np.asarray(self.pressureTimes) - tmin) / 1e6
        plt.plot(scaledPressureTimes, self.pressureReadings)
        v = plt.axis()
        plt.axis([0, scaledPressureTimes[-1], v[2], v[3]])     
        plt.grid(True)

    def plotMovement(self, smoothed, movement, tmin):
        plt.title('Movement')
        scaledJointTimes = (np.asarray(self.jointVelocityTimes) - tmin) / 1e6
        
        plt.plot(scaledJointTimes, smoothed)
        plt.plot(scaledJointTimes, movement, 'r--')
        plt.axis([0, scaledJointTimes[-1], 0, 1.2])       
        plt.ylabel('smoothed norm of xdot')
        plt.grid(True)

    def plotBattery(self, tmin):
        plt.title('Battery')        
        plt.ylabel('Charge Remaining (%)')
        scaledBatteryTimes = (np.asarray(self.batteryTimes) - tmin) / 1e6
        plt.plot(scaledBatteryTimes, self.batteryPercentage)
        v = plt.axis()
        plt.axis([0, scaledBatteryTimes[-1], v[2], v[3]])     
        plt.grid(True)

    def plotResults(self):
        tmin = self.jointVelocityTimes[0]
        tmax = self.jointVelocityTimes[-1]
        totalMicroseconds = float(tmax-tmin)
        average_dt =  totalMicroseconds/len(self.jointVelocityNorms)
        smoothed = self.movingAverage(self.jointVelocityNorms)
        movement = smoothed > self.movementThreshold 
        movementSeconds = float(np.count_nonzero(movement) * average_dt / 1e6)
        totalSeconds = float((tmax-tmin)/1e6)
        print("%.2f / %.2f seconds of movement ( %.2f %% continuous motion) " % (movementSeconds, totalSeconds , movementSeconds / totalSeconds * 1e2))        
        
        minChargePercent = np.ndarray.min(np.asarray(self.batteryPercentage))
        print("Battery fell from %.2f %% to %.2f %% (Used %.2f %%)" % (self.batteryPercentage[0], minChargePercent , self.batteryPercentage[0] - minChargePercent))
        print 'plotting results'
        plt.figure(1)
        plt.suptitle('LCM Log Battery/Movement Analysis')
        plt.subplot(311)
        self.plotMovement(smoothed, movement, tmin)
        plt.subplot(312)
        self.plotPump(tmin)
        plt.subplot(313)
        self.plotBattery(tmin)

        plt.xlabel('Time (s)')
        plt.show()      

def main(argv):

    try:
        logFile = sys.argv[1]
    except IndexError:
        print 'Usage: %s <log file>' % sys.argv[0]
        sys.exit(1)

    spy.findLCMModulesInSysPath()
    
    parser = LCMLogAnalyzer(logFile)
    
    parser.parseLog()
    parser.plotResults()


if __name__ == '__main__':
    main(sys.argv)
