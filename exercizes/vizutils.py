import numpy as np
import pinocchio as pin

# Meshcat utils


def meshcat_material(r, g, b, a):
    import meshcat

    material = meshcat.geometry.MeshPhongMaterial()
    material.color = int(r * 255) * 256**2 + int(g * 255) * 256 + int(b * 255)
    material.opacity = a
    return material


def meshcat_transform(x, y, z, q, u, a, t):
    return np.array(pin.XYZQUATToSE3([x, y, z, q, u, a, t]))


# Gepetto/meshcat abstraction


def addViewerBox(viz, name, sizex, sizey, sizez, rgba):
    if isinstance(viz, pin.visualize.MeshcatVisualizer):
        import meshcat

        viz.viewer[name].set_object(
            meshcat.geometry.Box([sizex, sizey, sizez]), meshcat_material(*rgba)
        )
    elif isinstance(viz, pin.visualize.GepettoVisualizer):
        viz.viewer.gui.addBox(name, sizex, sizey, sizez, rgba)
    else:
        msg = f"Viewer {viz.__class__} is not supported."
        raise AttributeError(msg)


def addViewerSphere(viz, name, size, rgba):
    if isinstance(viz, pin.visualize.MeshcatVisualizer):
        import meshcat

        viz.viewer[name].set_object(
            meshcat.geometry.Sphere(size), meshcat_material(*rgba)
        )
    elif isinstance(viz, pin.visualize.GepettoVisualizer):
        viz.viewer.gui.addSphere(name, size, rgba)
    else:
        msg = f"Viewer {viz.__class__} is not supported."
        raise AttributeError(msg)


def applyViewerConfiguration(viz, name, xyzquat):
    if isinstance(viz, pin.visualize.MeshcatVisualizer):
        viz.viewer[name].set_transform(meshcat_transform(*xyzquat))
    elif isinstance(viz, pin.visualize.GepettoVisualizer):
        viz.viewer.gui.applyConfiguration(name, xyzquat)
        viz.viewer.gui.refresh()
    else:
        msg = f"Viewer {viz.__class__} is not supported."
        raise AttributeError(msg)


"""


        viz.viewer['world/ball'].set_object(meshcat.geometry.Sphere(.1),
                                       meshcat_material(.2, .2, 1., .5))
viz.viewer['world/box'].set_transform(meshcat_transform(.5, .2, .2, 1, 0, 0, 0))
viz.viewer['world/ball'].set_transform(meshcat_transform(-.5, .2, .2, 1, 0, 0, 0))
"""
