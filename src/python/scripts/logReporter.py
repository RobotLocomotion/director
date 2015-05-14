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
        self.batteryVoltage = list()
        self.N = 100 #sliding window width
        self.movementThreshold = 0.4
        self.minVoltage = 143
        self.maxVoltage = 190

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
                self.batteryVoltage.append(msg.voltage)

        print 'parsed ' + str(len(self.jointVelocityNorms)) + ' robot states'
        print 'parsed ' + str(len(self.batteryVoltage)) + ' battery states'
        
    def movingAverage(self, x):
        N = self.N
        return np.convolve(x, np.ones((N,))/N)[(N-1):]

    def getBatteryPercentage(self, voltage):
        return (voltage - self.minVoltage) / (self.maxVoltage - self.minVoltage) * 1e2

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
        print("Battery fell from %.2f V ( %.2f %% ) to %.2f V ( %.2f %% )" % (self.batteryVoltage[0], self.getBatteryPercentage(self.batteryVoltage[0]), self.batteryVoltage[-1], self.getBatteryPercentage(self.batteryVoltage[-1])))
        print 'plotting results'
        plt.figure(1)
        plt.suptitle('LCM Log Battery/Movement Analysis')
        plt.subplot(211)
        plt.title('Movement')
        scaledJointTimes = (np.asarray(self.jointVelocityTimes) - tmin) / 1e6
        scaledBatteryTimes = (np.asarray(self.batteryTimes) - tmin) / 1e6
        plt.plot(scaledJointTimes, smoothed)
        plt.plot(scaledJointTimes, movement, 'r--')
        plt.axis([0, scaledJointTimes[-1], 0, 2])       
        plt.ylabel('smoothed norm of xdot')
        plt.grid(True)
        batteryPlot = plt.subplot(212)
        plt.title('Battery')
        plt.xlabel('Time (s)')
        plt.ylabel('Voltage (V)')
        plt.plot(scaledBatteryTimes, self.batteryVoltage)
        v = plt.axis()
        plt.axis([0, scaledJointTimes[-1], v[2], v[3]])     
        plt.grid(True)
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
