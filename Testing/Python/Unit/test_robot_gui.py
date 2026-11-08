"""Run in Slicer with --python-script and the ROS2 module on its module path.

Checks GUI synchronization without requiring robot_state_publishers or meshes.
"""

import time
import traceback

import qt
import slicer
import vtk

from slicerros2.dvrk import PatientCart


def run():
    cart = PatientCart()
    # Exercise loading before the module widget exists.
    cart.load()
    widget = slicer.modules.ros2.widgetRepresentation()

    def rows():
        return {edit.text: edit.parentWidget()
                for edit in widget.findChildren(qt.QLineEdit)
                if edit.objectName == "robotNameLineEdit"}

    def expect(names):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            slicer.app.processEvents()
            qt.QCoreApplication.sendPostedEvents(None, qt.QEvent.DeferredDelete)
            if set(rows()) == set(names):
                return
            time.sleep(.01)
        raise AssertionError(f"Expected robot rows {names}, got {list(rows())}")

    def button(row, name):
        return next(child for child in row.findChildren(qt.QPushButton)
                    if child.objectName == name)

    try:
        names = {"PSM1", "PSM2", "PSM3", "ECM"}
        expect(names)
        for row in rows().values():
            assert not button(row, "loadRobotButton").enabled
            assert button(row, "removeRobotButton").enabled

        # Repeated loads must not create duplicate rows.
        cart.load()
        slicer.app.processEvents()
        assert len(widget.findChildren(qt.QLineEdit, "robotNameLineEdit")) == 4

        # Removal from Python, including removal and replacement in one turn.
        node = cart.ros_node
        node.RemoveAndDeleteRobotNode("PSM1")
        expect(names - {"PSM1"})
        cart.load()
        expect(names)
        cart.unload()
        cart.load()
        expect(names)

        # GUI removal must remove the underlying MRML robot, too.
        button(rows()["ECM"], "removeRobotButton").click()
        expect(names - {"ECM"})
        assert node.GetRobotNodeByName("ECM") is None
        cart.unload()
        expect(set())

        # Exercise loading after the widget has already been opened.
        cart.load()
        expect(names)
        print("PASS: robot GUI creation, repeated load, Python/GUI removal, and reload", flush=True)
    finally:
        cart.unload()
        slicer.app.processEvents()


if __name__ == "__main__":
    status = 0
    try:
        run()
    except Exception:
        traceback.print_exc()
        status = 1
    slicer.mrmlScene.Clear(0)
    vtk.vtkDebugLeaks.SetExitError(False)
    slicer.util.exit(status)
