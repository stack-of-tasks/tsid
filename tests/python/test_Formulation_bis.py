import pinocchio as pin
import hppfcl as fcl
import numpy as np
from math import pi
import time
import sys

# Create models
model = pin.Model()
geom_model = pin.GeometryModel()

radius = 0.1

# Base
geom_base = pin.GeometryObject("base", 0, fcl.Box(0.5,0.5,0.02), pin.SE3(np.identity(3), np.array([0,0,-0.01])))
geom_base.meshColor = np.array([1.,0.1,0.1,1.])
geom_model.addGeometryObject(geom_base)

parent_id = 0

# Upper arm
joint_placement = pin.SE3.Identity()
joint_id = model.addJoint(parent_id, pin.JointModelRX(), joint_placement, "shoulder")
body_inertia = pin.Inertia.FromCylinder(1.0 , radius, 0.66)
body_placement = joint_placement.copy()
body_placement.translation[2] = +0.33
model.appendBodyToJoint(joint_id, body_inertia, body_placement)

idx_q = model.joints[joint_id].idx_q
model.upperPositionLimit[idx_q] = pi
model.lowerPositionLimit[idx_q] = -pi

# geom1_name = "UpperArm"
# shape1 = fcl.Sphere(body_radius)
# geom1_obj = pin.GeometryObject(geom1_name, joint_id, shape1, body_placement)
# geom1_obj.meshColor = np.ones((4))
# geom_model.addGeometryObject(geom1_obj)

geom2_name = "UpperArm"
shape2 = fcl.Cylinder(radius, 0.66)
shape2_placement = body_placement.copy()

geom2_obj = pin.GeometryObject(geom2_name, joint_id, shape2, shape2_placement)
geom2_obj.meshColor = np.array([0.1,0.8,0.1,.8])
geom_model.addGeometryObject(geom2_obj)

# Update
parent_id = joint_id
# joint_placement = body_placement.copy()

# Lower arm
joint_placement = pin.SE3(np.identity(3), np.array([0,0, 0.66]))
joint_id = model.addJoint(parent_id, pin.JointModelRX(), joint_placement, "elbow")

body_inertia = pin.Inertia.FromCylinder(1.0 , radius, 0.66)

body_placement = joint_placement = pin.SE3(np.identity(3), np.array([0,0, 0.33]))
model.appendBodyToJoint(joint_id, body_inertia, body_placement)

idx_q = model.joints[joint_id].idx_q
model.upperPositionLimit[idx_q] = pi
model.lowerPositionLimit[idx_q] = -pi

# geom1_name = "UpperArm"
# shape1 = fcl.Sphere(body_radius)
# geom1_obj = pin.GeometryObject(geom1_name, joint_id, shape1, body_placement)
# geom1_obj.meshColor = np.ones((4))
# geom_model.addGeometryObject(geom1_obj)

geom2_name = "LowerArm"
shape2 = fcl.Cylinder(radius, 0.66)
shape2_placement = body_placement.copy()

geom2_obj = pin.GeometryObject(geom2_name, joint_id, shape2, shape2_placement)
geom2_obj.meshColor = np.array([0.1,0.1,0.8,.8])
geom_model.addGeometryObject(geom2_obj)


# Update
parent_id = joint_id
# joint_placement = body_placement.copy()

# Hand
joint_placement = pin.SE3(np.identity(3), np.array([0,0,0.66]))
joint_id = model.addJoint(parent_id, pin.JointModelRX(), joint_placement, "wrist")

body_inertia = pin.Inertia.FromCylinder(1.0 , radius, 0.2)
body_placement = joint_placement = pin.SE3(np.identity(3), np.array([0,0, 0.1]))
model.appendBodyToJoint(joint_id, body_inertia, body_placement)

idx_q = model.joints[joint_id].idx_q
model.upperPositionLimit[idx_q] = pi
model.lowerPositionLimit[idx_q] = -pi

# geom1_name = "UpperArm"
# shape1 = fcl.Sphere(body_radius)
# geom1_obj = pin.GeometryObject(geom1_name, joint_id, shape1, body_placement)
# geom1_obj.meshColor = np.ones((4))
# geom_model.addGeometryObject(geom1_obj)

geom2_name = "Wrist"
shape2 = fcl.Cylinder(radius, 0.2)
shape2_placement = body_placement.copy()

geom2_obj = pin.GeometryObject(geom2_name, joint_id, shape2, shape2_placement)
geom2_obj.meshColor = np.array([0.7,0.7,0.7,0.8])
geom_model.addGeometryObject(geom2_obj)


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