from abc import ABCMeta, abstractmethod
import raisimpy as raisim
import os
import numpy as np

class RobotSimulator(metaclass=ABCMeta):
    @abstractmethod
    def addRobot(self):
        print("Robot Added")

    @abstractmethod
    def kill(self):
        print("Server Killed")

    @abstractmethod
    def getForces(self):
        print("")

    @abstractmethod
    def getQ(self):
        print("")

    @abstractmethod
    def getV(self):
        print("")

    def integrate(self):
        print("")

    def getFramePos(self):
        print("")

    def getFrameOri(self):
        print("")

    def updateTau(self):
        print("")

    def setTarget(self):
        print("")

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
        # self.robot.setPdGains(np.ones(18) * 50, np.ones(18) * 1)

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
                force = (contact.getImpulse() / self.dt)
                # force = np.matmul(R, force)
                force[1:] *= -1
                conts.append([contact.getlocalBodyIndex(), force, contact.getContactFrame()])
        return conts

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