#!/usr/bin/env python

pkg = 'hrpsys'
import imp
imp.find_module(pkg)

from hrpsys import *
from hrpsys.hrpsys_config import *
import OpenHRP

class SlaveConfigurator(HrpsysConfigurator):
    # WholeBodyMasterSLave
    wbms = None
    wbms_svc = None
    wbms_version = None

    def getRTCList(self):
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['kf', "KalmanFilter"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            ['octd', "ObjectContactTurnaroundDetector"],
            ['es', "EmergencyStopper"],
            ['rfu', "ReferenceForceUpdater"],
            ['wbms', "WholeBodyMasterSlave"],
            ['ic', "ImpedanceController"],
            ['abc', "AutoBalancer"],
            ['st', "Stabilizer"],
            ['co', "CollisionDetector"],
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            ['log', "DataLogger"]
            ]

    def connectComps(self):
        super(SlaveConfigurator,self).connectComps()

        # connection for wbms
        if self.wbms:
            connectPorts(self.sh.port("qOut"), self.wbms.port("qRef"))
            connectPorts(self.rh.port("q"), self.wbms.port("qAct"))
            connectPorts(self.sh.port("zmpOut"), self.wbms.port("zmpIn"))
            connectPorts(self.sh.port("basePosOut"), self.wbms.port("basePosIn"))
            connectPorts(self.sh.port("baseRpyOut"), self.wbms.port("baseRpyIn"))
            connectPorts(self.sh.port("optionalDataOut"), self.wbms.port("optionalData"))
            connectPorts(self.rmfo.port("off_lfsensor"), self.wbms.port("local_lleg_wrench"))
            connectPorts(self.rmfo.port("off_rfsensor"), self.wbms.port("local_rleg_wrench"))
            connectPorts(self.rmfo.port("off_lhsensor"), self.wbms.port("local_larm_wrench"))
            connectPorts(self.rmfo.port("off_rhsensor"), self.wbms.port("local_rarm_wrench"))

        if rtm.findPort(self.rh.ref, "lfsensor") and rtm.findPort(
                                     self.rh.ref, "rfsensor") and self.st:
            if self.wbms:
                disconnectPorts(self.sh.port("zmpOut"),            self.abc.port("zmpIn"))
                disconnectPorts(self.sh.port("basePosOut"),        self.abc.port("basePosIn"))
                disconnectPorts(self.sh.port("baseRpyOut"),        self.abc.port("baseRpyIn"))
                disconnectPorts(self.sh.port("optionalDataOut"),   self.abc.port("optionalData"))

                connectPorts(self.st.port("actCapturePoint"),   self.wbms.port("actCapturePoint"))
                connectPorts(self.st.port("zmp"),               self.wbms.port("zmp"))
                connectPorts(self.wbms.port("zmpOut"),          self.abc.port("zmpIn"))
                connectPorts(self.wbms.port("basePosOut"),      self.abc.port("basePosIn"))
                connectPorts(self.wbms.port("baseRpyOut"),      self.abc.port("baseRpyIn"))
                connectPorts(self.wbms.port("optionalDataOut"), self.abc.port("optionalData"))
                connectPorts(self.wbms.port("AutoBalancerService"), self.abc.port("AutoBalancerService"))
                connectPorts(self.wbms.port("StabilizerService"),   self.st.port("StabilizerService"))

    def getJointAngleControllerList(self):
        '''!@brief
        Get list of controller list that need to control joint angles
        '''
        controller_list = super(SlaveConfigurator,self).getJointAngleControllerList()
        if self.es:
            controller_list.insert(controller_list.index(self.es)+1,self.wbms)
        else:
            controller_list.insert(0,self.wbms)
        return filter(lambda c: c != None, controller_list)  # only return existing controllers

    def setupLogger(self, maxLength=4000):
        super(SlaveConfigurator,self).setupLogger(maxLength)

        if self.wbms != None:
            self.connectLoggerPort(self.wbms, 'q')
            self.connectLoggerPort(self.wbms, 'basePosOut')
            self.connectLoggerPort(self.wbms, 'baseRpyOut')
            self.connectLoggerPort(self.wbms, 'zmpOut')
            self.connectLoggerPort(self.wbms, 'baseTformOut')
            self.connectLoggerPort(self.wbms, 'slave_rleg_wrench')
            self.connectLoggerPort(self.wbms, 'slave_lleg_wrench')
            self.connectLoggerPort(self.wbms, 'slave_rarm_wrench')
            self.connectLoggerPort(self.wbms, 'slave_larm_wrench')
            self.connectLoggerPort(self.wbms, 'slave_rleg_pose')
            self.connectLoggerPort(self.wbms, 'slave_lleg_pose')
            self.connectLoggerPort(self.wbms, 'slave_rarm_pose')
            self.connectLoggerPort(self.wbms, 'slave_larm_pose')
            self.connectLoggerPort(self.wbms, 'slave_com_pose')
            self.connectLoggerPort(self.wbms, 'slave_head_pose')

