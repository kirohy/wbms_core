#!/usr/bin/env python

pkg = 'hrpsys'
import imp
imp.find_module(pkg)

from hrpsys import *
from hrpsys.hrpsys_config import *
import OpenHRP

class MasterConfigurator(HrpsysConfigurator):
    # HapticController
    hc = None
    hc_svc = None
    hc_version = None

    def getRTCList(self):
        return [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            # ['kf', "KalmanFilter"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            ['hc', "HapticController"],
            ['log', "DataLogger"]
           ]

    def connectComps(self):
        super(MasterConfigurator,self).connectComps()

        # connection for hc
        if self.hc:
            connectPorts(self.sh.port("qOut"),   self.hc.port("qRef"))
            connectPorts(self.rh.port("q"),   self.hc.port("qAct"))
            connectPorts(self.rh.port("dq"),  self.hc.port("dqAct"))
            connectPorts(self.hc.port("tau"), self.rh.port("tauRef"))

    def setupLogger(self, maxLength=4000):
        super(MasterConfigurator,self).setupLogger(maxLength)

        if self.hc != None:
            self.connectLoggerPort(self.hc, 'tau')
            self.connectLoggerPort(self.hc, 'debugData')
            self.connectLoggerPort(self.hc, 'master_rleg_pose')
            self.connectLoggerPort(self.hc, 'master_lleg_pose')
            self.connectLoggerPort(self.hc, 'master_rarm_pose')
            self.connectLoggerPort(self.hc, 'master_larm_pose')
            self.connectLoggerPort(self.hc, 'master_relbow_pose')
            self.connectLoggerPort(self.hc, 'master_lelbow_pose')
            self.connectLoggerPort(self.hc, 'master_com_pose')
            self.connectLoggerPort(self.hc, 'master_head_pose')
            self.connectLoggerPort(self.hc, 'master_rleg_wrench_dbg')
            self.connectLoggerPort(self.hc, 'master_lleg_wrench_dbg')
            self.connectLoggerPort(self.hc, 'master_rarm_wrench_dbg')
            self.connectLoggerPort(self.hc, 'master_larm_wrench_dbg')
            self.connectLoggerPort(self.hc, 'master_relbow_wrench_dbg')
            self.connectLoggerPort(self.hc, 'master_lelbow_wrench_dbg')

    def startUpSequence(self):
        print("reset virtual floor, odom")
        hp = self.hc_svc.getParams()
        hp.baselink_height_from_floor=1.5
        hp.q_ref_max_torque_ratio=0.1
        self.hc_svc.setParams(hp)
        self.hc_svc.resetOdom()

        print("servo ON")
        self.rh_svc.setServoGainPercentage("all", 0) # for tablis torque control
        time.sleep(1)
        import OpenHRP
        self.rh_svc.setJointControlMode("ALL",OpenHRP.RobotHardwareService.TORQUE) # for tablis torque control
        self.servoOn()
        self.rh_svc.setServoTorqueGainPercentage("all", 100) # for tablis torque control
        self.seq_svc.setJointAngles(self.tablisResetPoseUpper(), 5.0)
        time.sleep(5)
        self.setResetPose()
        self.hc_svc.startHapticController()

        print("wait for stand by")
        time.sleep(10)

        print("set virtual floor")
        hp = self.hc_svc.getParams()
        hp.baselink_height_from_floor=0.8
        hp.force_feedback_ratio=0.1
        hp.torque_feedback_ratio=0
        hp.wrench_hpf_gain=0.0
        hp.q_ref_max_torque_ratio=0.01
        hp.ex_gravity_compensation_ratio_lower=1
        self.hc_svc.setParams(hp)
        self.hc_svc.getParams()
        self.hc_svc.resetOdom()
        time.sleep(3)

        print("[all steps completed]")
