import pinocchio as pin
import hppfcl as fcl
import numpy as np
from math import pi
import time
import sys


def set_limit(model, joint_id, val):
    idx_q = model.joints[joint_id].idx_q
    nq =  model.joints[joint_id].nq
    for idx in range(idx_q, idx_q+nq):
        model.upperPositionLimit[idx] = val
        model.lowerPositionLimit[idx] = -val

# Create models
model = pin.Model()
geom_model = pin.GeometryModel()

radius = 0.1

# Base
id_base = 0

color_base = np.array([1.,0.1,0.1,1.])

geom_base = pin.GeometryObject("base",
                               id_base, # parent
                               fcl.Box(0.5,0.5,0.02),
                               pin.SE3(np.identity(3), np.array([0,0,-0.01-radius])),
                               "", np.ones(3), False, # Default values
                               color_base
                              )
geom_model.addGeometryObject(geom_base)

geom_shoulder = pin.GeometryObject(
                                    "shoulderGeom",
                                    id_base,
                                    fcl.Sphere(radius),
                                    pin.SE3.Identity(),
                                    "", np.ones(3), False, # Default values
                                    color_base
                                  )
geom_model.addGeometryObject(geom_shoulder)


# Upper arm
placement_upper = pin.SE3(np.identity(3), np.array([0,0, 0.33]))
id_upper = model.addJoint(
                           id_base,
                           pin.JointModelRX(),
                           pin.SE3.Identity(),
                           "shoulder"
                          )
model.appendBodyToJoint(
                         id_upper,
                         pin.Inertia.FromCylinder(1.0 , radius, 0.66),
                         placement_upper
                        )
set_limit(model, id_upper, pi)

color_upper = np.array([0.1,0.8,0.1,.8])

elbow_obj = pin.GeometryObject(
                                "elbowGeom",
                                id_upper,
                                fcl.Sphere(radius),
                                pin.SE3(np.identity(3), np.array([0,0, 0.66])),
                                "", np.ones(3), False, # Default values
                                color_upper
                               )
geom_model.addGeometryObject(elbow_obj)

upper_obj = pin.GeometryObject(
                                "upperGeom",
                                id_upper,
                                fcl.Cylinder(radius, 0.66),
                                placement_upper,
                                "", np.ones(3), False, # Default values
                                color_upper
                               )
geom_model.addGeometryObject(upper_obj)

# Lower arm
placement_lower = pin.SE3(np.identity(3), np.array([0,0, 0.33]))
id_lower = model.addJoint(
                           id_upper,
                           pin.JointModelRX(),
                           pin.SE3(np.identity(3), np.array([0,0, 0.66])),
                           "elbow"
                          )
model.appendBodyToJoint(
                         id_lower,
                         pin.Inertia.FromCylinder(1.0 , radius, 0.66),
                         placement_lower
                        )
set_limit(model, id_lower, pi)

color_lower = np.array([0.1,0.1,0.8,.8])

wrist_obj = pin.GeometryObject(
                                "wristGeom",
                                id_lower,
                                fcl.Sphere(radius),
                                pin.SE3(np.identity(3), np.array([0,0, 0.66])),
                                "", np.ones(3), False, # Default values
                                color_lower
                               )
geom_model.addGeometryObject(wrist_obj)

upper_obj = pin.GeometryObject(
                                "lowerGeom",
                                id_lower,
                                fcl.Cylinder(radius, 0.66),
                                placement_lower,
                                "", np.ones(3), False, # Default values
                                color_lower
                               )
geom_model.addGeometryObject(upper_obj)


# Hand
placement_hand = pin.SE3(np.identity(3), np.array([0,0, 0.66]))
id_hand = model.addJoint(
                           id_lower,
                           pin.JointModelRX(),
                           pin.SE3(np.identity(3), np.array([0,0, 0.66])),
                           "elbow"
                          )
model.appendBodyToJoint(
                         id_hand,
                         pin.Inertia.FromSphere(1.0 , radius),
                         placement_hand
                        )
set_limit(model, id_hand, pi)

color_hand = np.array([0.7,0.7,0.7,.8])

palm_obj = pin.GeometryObject(
                                "palmGeom",
                                id_hand,
                                fcl.Sphere(radius),
                                pin.SE3(np.identity(3), np.array([0,0, 2*radius])),
                                "", np.ones(3), False, # Default values
                                color_hand
                               )
geom_model.addGeometryObject(palm_obj)


# Main
from pinocchio.visualize import GepettoVisualizer as Visualizer

visual_model = geom_model
viz = Visualizer(model, geom_model, visual_model)
viz.initViewer()
viz.loadViewerModel("pinocchio")

# Display a robot configuration.
q0 = pin.neutral(model)
viz.display(q0)
while True:
    viz.display(pin.randomConfiguration(model))
    input()