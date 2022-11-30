from abc import ABCMeta, abstractmethod
import raisimpy as raisim
import os
import numpy as np
import math
from eigen import Vector3d, Quaterniond

class RobotSimulator(metaclass=ABCMeta):
    @abstractmethod
    def addRobot(self):
        pass

    @abstractmethod
    def kill(self):
        pass

    @abstractmethod
    def getForces(self):
        pass

    @abstractmethod
    def getQ(self):
        pass

    @abstractmethod
    def getV(self):
        pass

    def integrate(self):
        pass

    def getFramePos(self):
        pass

    def getFrameOri(self):
        pass

    def updateTau(self):
        pass

    def setTarget(self):
        pass

class RaiSim(RobotSimulator):
    def __init__(self, time_step):
        raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")
        self.world = raisim.World()
        self.dt = time_step
        self.world.setTimeStep(time_step)
        self.ground = self.world.addGround()
        # launch raisim server
        self.server = raisim.RaisimServer(self.world)
        self.server.launchServer(8080)

    def addRobot(self, urdf, q, v):
        self.robot = self.world.addArticulatedSystem(urdf)
        self.robot.setState(q, v)
        q[0:7] *= 0
        v[0:6] *= 0
        # self.robot.setPdTarget(q, v)
        #self.robot.setPdGains(np.ones(18) * 50, np.ones(18) * 1)

    def kill(self):
        self.server.killServer()

    def getForces(self):
        contacts = self.robot.getContacts()
        conts = []
        for contact in contacts:
            if contact.skip():
                continue
            if contact.isObjectA():
                continue
            if(contact.getlocalBodyIndex()):
                R = self.robot.getFrameOrientation(contact.getlocalBodyIndex())
                # print(R)
                force = (contact.getImpulse() / self.dt)
                # force = np.matmul(force, R)
                print(contact.getlocalBodyIndex())
                print(force)
                force[1] *= -1
                if (force[2] != 0):
                # if True:
                    # print(force)
                    conts.append([contact.getlocalBodyIndex(), force, contact.getContactFrame()])
        return conts

    def getAngularVelTransform(self):
        ori = self.robot.getBaseOrientation()
        w_R_B_quat = Quaterniond(ori[1], ori[2], ori[3], ori[0])
        w_R_B_quat = w_R_B_quat.toRotationMatrix().transpose()
        return w_R_B_quat

    def untransformAngularVel(self, av):
        ori = self.robot.getBaseOrientation()
        w_R_B_quat = Quaterniond(ori[1], ori[2], ori[3], ori[0])
        w_R_B_quat = w_R_B_quat.toRotationMatrix().transpose()
        print(av)
        print(w_R_B_quat)
        return np.matmul(av, w_R_B_quat)


    def getQ(self):
        return self.robot.getGeneralizedCoordinate()

    def getV(self):
        return self.robot.getGeneralizedVelocity()

    def integrate(self):
        self.server.integrateWorldThreadSafe()
        # self.world.integrate()
        # self.world.integrate2()

    def getFramePos(self, name):
        return self.robot.getFramePosition(name)

    def getFrameOri(self, name):
        return self.robot.getFrameOrientation(name)

    def updateTau(self, tau):
        self.robot.setGeneralizedForce(tau)

    def setTarget(self, q, v):
        q[0:7] *= 0
        v[0:6] *= 0
        self.robot.setPdTarget(q, v)