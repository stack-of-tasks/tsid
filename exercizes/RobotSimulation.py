from abc import ABCMeta, abstractmethod
import raisimpy as raisim
import os

class RobotSimulator(metaclass=ABCMeta):

    @abstractmethod
    def updateConfiguration(self, time_step):
        print("Configuration Updated")

    @abstractmethod
    def addRobot(self):
        print("Robot Added")

    @abstractmethod
    def kill(self):
        print("Server Killed")



class RaiSim(RobotSimulator):
    def __init__(self, time_step):
        raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")
        self.world = raisim.World()
        self.world.setTimeStep(time_step)
        self.ground = self.world.addGround()
        # launch raisim server
        self.server = raisim.RaisimServer(self.world)
        self.server.launchServer(8080)

    def updateConfiguration(self, q):
        if self.robot:
            self.robot.setGeneralizedCoordinate(q)
            self.server.integrateWorldThreadSafe()

    def addRobot(self, urdf):
        self.robot = self.world.addArticulatedSystem(urdf)

    def kill(self):
        self.server.killServer()

    def printImpulse(self):
        contacts = self.robot.getContacts()
        for i in range(0, len(contacts)):
            if (contacts[i].getlocalBodyIndex() == 6) or (contacts[i].getlocalBodyIndex() == 9):
                print(contacts[i].getImpulse())
