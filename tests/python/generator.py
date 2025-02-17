from math import pi

import hppfcl as fcl
import numpy as np
import pinocchio as pin


def create_7dof_arm(revoluteOnly=False):
    """Create a 7 DoF robot arm (with spherical joint for shoulder and wrist and revolute
    joint for elbow)

    Optionnal parameters:
        revoluteOnly (default=False): if True, the arm is created with only revolute
        joints. (Spherical joints are replaced by 3 revolute joints)

    Return:
    ------
        model: pinocchio model of the robot
        geom_model: pinocchio geometric model of the robot

    """
    # Some useful values
    geom_radius = 0.1  # radius of the arm geometries
    geom_length = 0.66  # length of the lower or upper arm geometries

    def set_limit(model, joint_id, pos_absmax, vel_absmax, eff_absmax):
        idx_q = model.joints[joint_id].idx_q
        nq = model.joints[joint_id].nq
        for idx in range(idx_q, idx_q + nq):
            model.upperPositionLimit[idx] = pos_absmax
            model.lowerPositionLimit[idx] = -pos_absmax

        idx_v = model.joints[joint_id].idx_v
        nv = model.joints[joint_id].nv
        for idx in range(idx_v, idx_v + nv):
            model.velocityLimit[idx] = vel_absmax
            model.effortLimit[idx] = eff_absmax

    # Create models
    model = pin.Model()
    geom_model = pin.GeometryModel()

    # Base
    id_base = 0

    color_base = np.array([1.0, 0.1, 0.1, 1.0])

    geom_model.addGeometryObject(
        pin.GeometryObject(
            "base",
            id_base,  # parent
            fcl.Box(0.5, 0.5, 0.02),
            pin.SE3(np.identity(3), np.array([0, 0, -0.01 - geom_radius])),
            "",
            np.ones(3),
            False,  # Default values
            color_base,
        )
    )

    geom_model.addGeometryObject(
        pin.GeometryObject(
            "shoulderGeom",
            id_base,
            fcl.Sphere(geom_radius),
            pin.SE3.Identity(),
            "",
            np.ones(3),
            False,  # Default values
            color_base,
        )
    )

    # Upper arm
    placement_upper = pin.SE3(np.identity(3), np.array([0, 0, geom_length / 2.0]))
    if not revoluteOnly:
        id_upper = model.addJoint(
            id_base, pin.JointModelSpherical(), pin.SE3.Identity(), "shoulder"
        )
        set_limit(model, id_upper, pi, 3.0, 100.0)
    else:
        id_upper = model.addJoint(
            id_base, pin.JointModelRX(), pin.SE3.Identity(), "shoulderX"
        )
        set_limit(model, id_upper, pi, 3.0, 100.0)
        id_upper = model.addJoint(
            id_upper, pin.JointModelRY(), pin.SE3.Identity(), "shoulderY"
        )
        set_limit(model, id_upper, pi, 3.0, 100.0)
        id_upper = model.addJoint(
            id_upper, pin.JointModelRZ(), pin.SE3.Identity(), "shoulderZ"
        )
        set_limit(model, id_upper, pi, 3.0, 100.0)
    model.appendBodyToJoint(
        id_upper,
        pin.Inertia.FromCylinder(1.0, geom_radius, geom_length),
        placement_upper,
    )

    color_upper = np.array([0.1, 0.8, 0.1, 0.8])

    geom_model.addGeometryObject(
        pin.GeometryObject(
            "elbowGeom",
            id_upper,
            fcl.Sphere(geom_radius),
            pin.SE3(np.identity(3), np.array([0, 0, geom_length])),
            "",
            np.ones(3),
            False,  # Default values
            color_upper,
        )
    )

    geom_model.addGeometryObject(
        pin.GeometryObject(
            "upperGeom",
            id_upper,
            fcl.Cylinder(geom_radius, geom_length),
            placement_upper,
            "",
            np.ones(3),
            False,  # Default values
            color_upper,
        )
    )

    # Lower arm
    placement_lower = pin.SE3(np.identity(3), np.array([0, 0, geom_length / 2.0]))
    id_lower = model.addJoint(
        id_upper,
        pin.JointModelRX(),
        pin.SE3(np.identity(3), np.array([0, 0, geom_length])),
        "elbow",
    )
    model.appendBodyToJoint(
        id_lower,
        pin.Inertia.FromCylinder(1.0, geom_radius, geom_length),
        placement_lower,
    )
    set_limit(model, id_lower, pi, 3.0, 100.0)

    color_lower = np.array([0.1, 0.1, 0.8, 0.8])

    geom_model.addGeometryObject(
        pin.GeometryObject(
            "wristGeom",
            id_lower,
            fcl.Sphere(geom_radius),
            pin.SE3(np.identity(3), np.array([0, 0, geom_length])),
            "",
            np.ones(3),
            False,  # Default values
            color_lower,
        )
    )

    geom_model.addGeometryObject(
        pin.GeometryObject(
            "lowerGeom",
            id_lower,
            fcl.Cylinder(geom_radius, geom_length),
            placement_lower,
            "",
            np.ones(3),
            False,  # Default values
            color_lower,
        )
    )

    # Hand
    placement_hand = pin.SE3(np.identity(3), np.array([0, 0, geom_length]))
    if not revoluteOnly:
        id_hand = model.addJoint(
            id_lower,
            pin.JointModelSpherical(),
            pin.SE3(np.identity(3), np.array([0, 0, geom_length])),
            "wrist",
        )
        set_limit(model, id_hand, pi, 3.0, 100.0)
    else:
        id_hand = model.addJoint(
            id_lower,
            pin.JointModelRX(),
            pin.SE3(np.identity(3), np.array([0, 0, geom_length])),
            "wristX",
        )
        set_limit(model, id_hand, pi, 3.0, 100.0)
        id_hand = model.addJoint(
            id_hand, pin.JointModelRY(), pin.SE3.Identity(), "wristY"
        )
        set_limit(model, id_hand, pi, 3.0, 100.0)
        id_hand = model.addJoint(
            id_hand, pin.JointModelRZ(), pin.SE3.Identity(), "wristZ"
        )
        set_limit(model, id_hand, pi, 3.0, 100.0)
        model.appendBodyToJoint(
            id_hand, pin.Inertia.FromSphere(1.0, geom_radius), placement_hand
        )

    color_hand = np.array([0.7, 0.7, 0.7, 0.8])

    geom_model.addGeometryObject(
        pin.GeometryObject(
            "palmGeom",
            id_hand,
            fcl.Sphere(geom_radius),
            pin.SE3(np.identity(3), np.array([0, 0, 2 * geom_radius])),
            "",
            np.ones(3),
            False,  # Default values
            color_hand,
        )
    )

    geom_model.addGeometryObject(
        pin.GeometryObject(
            "finger1Geom",
            id_hand,
            fcl.Cylinder(0.02, 0.1),
            pin.SE3(np.identity(3), np.array([0.05, 0, 3 * geom_radius])),
            "",
            np.ones(3),
            False,  # Default values
            color_hand,
        )
    )

    geom_model.addGeometryObject(
        pin.GeometryObject(
            "finger2Geom",
            id_hand,
            fcl.Cylinder(0.02, 0.1),
            pin.SE3(np.identity(3), np.array([0, 0, 3 * geom_radius])),
            "",
            np.ones(3),
            False,  # Default values
            color_hand,
        )
    )

    geom_model.addGeometryObject(
        pin.GeometryObject(
            "finger3Geom",
            id_hand,
            fcl.Cylinder(0.02, 0.1),
            pin.SE3(np.identity(3), np.array([-0.05, 0, 3 * geom_radius])),
            "",
            np.ones(3),
            False,  # Default values
            color_hand,
        )
    )

    geom_model.addGeometryObject(
        pin.GeometryObject(
            "thumbGeom",
            id_hand,
            fcl.Cylinder(0.02, 0.1),
            pin.XYZQUATToSE3(
                np.array([geom_radius, 0, 2 * geom_radius, 0, 0.707, 0, 0.707])
            ),
            "",
            np.ones(3),
            False,  # Default values
            color_hand,
        )
    )

    # Add end-effector frame
    model.addFrame(
        pin.Frame(
            "eeFrame",
            id_hand,
            0,
            pin.SE3(np.identity(3), np.array([0, 0, 2 * geom_radius])),
            pin.FrameType.OP_FRAME,
        )
    )

    return model, geom_model
