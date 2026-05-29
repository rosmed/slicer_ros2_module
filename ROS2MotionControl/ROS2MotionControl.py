import logging
import os, time, sys
import subprocess
from typing import Optional
import xml.etree.ElementTree as ET
import math
import re
import qt

# TrajectoryGenerators.py lives in a subdirectory to prevent Slicer from
# treating it as a scripted module.  Add that directory to sys.path once.
_tg_dir = os.path.join(os.path.dirname(__file__), "ROS2MotionControl")
if _tg_dir not in sys.path:
    sys.path.insert(0, _tg_dir)
import TrajectoryGenerators
try:
    from ROS2Tests import ROS2TestsLogic
except ImportError:
    ROS2TestsLogic = None

import vtk

import slicer
from slicer.i18n import tr as _
from slicer.i18n import translate
from slicer.ScriptedLoadableModule import *
from slicer.util import VTKObservationMixin
from slicer.parameterNodeWrapper import (
    parameterNodeWrapper,
)

# Set to True to enable verbose debug prints
DEBUG = False


def _check_ros2_node_running(node_name: str) -> bool:
    if ROS2TestsLogic is not None:
        try:
            return ROS2TestsLogic.check_ros2_node_running(node_name)
        except Exception:
            pass
    try:
        output = subprocess.check_output("ros2 node list", shell=True, stderr=subprocess.DEVNULL)
        nodes = output.decode("utf-8").splitlines()
        return node_name in nodes
    except Exception:
        return False


#
# ROS2MotionControl
#


class ROS2MotionControl(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = _("Motion Control")
        self.parent.categories = [translate("qSlicerAbstractCoreModule", "ROS 2")]
        self.parent.dependencies = ["ROS2"]
        self.parent.contributors = ["Kaito Hara-Lee", "Anton Deguet"]
        self.parent.helpText = _("""
    Scripted module for robot control and motion planning workflows integrated with SlicerROS2.
""")
        self.parent.acknowledgementText = _("""
    Integrated from the MedicalRobotMotionPlanner project and adapted for this repository.
""")

        pass  # no startup work needed


#
# ROS2MotionControlParameterNode
#


@parameterNodeWrapper
class ROS2MotionControlParameterNode:
    """Persistent settings for the ROS2MotionControl module.

    All fields are saved with the MRML scene and restored on reload,
    so user configuration survives module reload and .mrb save/open.
    """

    # Joint state subscriber topic (without leading /)
    jointStateTopic: str = "joint_states"

    # MRML Node IDs
    robotNodeID: str = ""
    motionControlNodeID: str = ""

    # MoveIt planning group name last selected by the user
    planningGroup: str = ""

    # Whether the /move_group node is expected to be running
    moveGroupExists: bool = False

    # Maximum time budget for MoveIt motion planning (seconds)
    planningTimeSec: float = 5.0

    # Velocity / acceleration scaling sent to MoveIt
    velocityScaling: float = 0.5
    accelerationScaling: float = 0.5

    # Goal-state robot display color as hex string (e.g. "#FF8000" = orange)
    goalColor: str = "#FF8000"


#
# ROS2MotionControlWidget
#


class ROS2MotionControlWidget(ScriptedLoadableModuleWidget, VTKObservationMixin):
    """Uses ScriptedLoadableModuleWidget base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent=None) -> None:
        """Called when the user opens the module the first time and the widget is initialized."""
        ScriptedLoadableModuleWidget.__init__(self, parent)
        VTKObservationMixin.__init__(self)  # needed for parameter node observation
        self.logic = None
        self._parameterNode = None
        self._parameterNodeGuiTag = None
        self.fromtransform = None
        self.totransform = None
        self.robot = None
        self.jointPositionsRad = []
        self.rootlink = None
        self.tiplink = None
        self.goaltiplink = None
        self.isRobotLoaded = False
        self.trajectoryTimer = None
        self.trajectoryData = None
        self.trajectoryIndex = 0
        self.trajectorySlider = None
        self.jointSliders = []
        self.jointSpinboxes = []
        self._sliderInitRetryCount = 0
        self._moveGroupParamObsId = None
        self._srdfEndEffectors = []  # list of {name, parent_link} dicts from SRDF
        self.obstaclePublishers = {} # modelNodeID -> modelNode
        self._syncingFromParameterNode = False

    def setup(self) -> None:
        """Called when the user opens the module the first time and the widget is initialized."""
        ScriptedLoadableModuleWidget.setup(self)

        # Load widget from .ui file (created by Qt Designer).
        # Additional widgets can be instantiated manually and added to self.layout.
        uiWidget = slicer.util.loadUI(self.resourcePath("UI/ROS2MotionControl.ui"))
        self.layout.addWidget(uiWidget)
        self.ui = slicer.util.childWidgetVariables(uiWidget)

        # Set scene in MRML widgets. Make sure that in Qt designer the top-level qMRMLWidget's
        # "mrmlSceneChanged(vtkMRMLScene*)" signal in is connected to each MRML widget's.
        # "setMRMLScene(vtkMRMLScene*)" slot.
        uiWidget.setMRMLScene(slicer.mrmlScene)
        self.ui.obstacleModelComboBox.setMRMLScene(slicer.mrmlScene)

        # Embed Slicer ROS2 module widget
        import ctk
        self.ros2CollapsibleButton = ctk.ctkCollapsibleButton()
        self.ros2CollapsibleButton.text = "Slicer ROS2"
        self.ros2CollapsibleButton.collapsed = True
        try:
            self.ros2Widget = slicer.modules.ros2.createNewWidgetRepresentation()
            self.ros2Widget.setMRMLScene(slicer.mrmlScene)
            layout = qt.QVBoxLayout(self.ros2CollapsibleButton)
            layout.addWidget(self.ros2Widget)
            self.layout.addWidget(self.ros2CollapsibleButton)
        except Exception as e:
            print(f"Warning: Could not embed Slicer ROS2 widget: {e}")

        # Create logic class. Logic implements all computations that should be possible to run
        # in batch mode, without a graphical user interface.
        self.logic = ROS2MotionControlLogic()

        # Connections

        # These connections ensure that we update parameter node when scene is closed
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent, self.onSceneStartClose)
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.EndCloseEvent, self.onSceneEndClose)
        self.addObserver(slicer.mrmlScene, slicer.vtkMRMLScene.NodeAboutToBeRemovedEvent, self.onNodeAboutToBeRemoved)
        self.ui.tabWidget.currentChanged.connect(self.onTabChanged)

        # Buttons
        self.ui.useButton.connect("clicked(bool)", self.onUseButton)
        self.ui.opacitySlider.connect("valueChanged(int)", self.onOpacitySliderChanged)
        self.ui.robotColorButton.connect("colorChanged(QColor)", self.onRobotColorChanged)
        self.ui.zeroPushButton.connect("clicked(bool)", self.onZeroButton)
        self.ui.currentStatePushButton.connect("clicked(bool)", self.onCurrentStateButton)
        self.ui.zeroPushButton3DControl.connect("clicked(bool)", self.onZeroButton)
        self.ui.lastGoalPushButton3DControl.connect("clicked(bool)", self.onLastGoalButton)
        self.ui.currentStatePushButton3DControl.connect("clicked(bool)", self.onCurrentStateButton)
        self.ui.moveGroupExistsCheckBox.connect("toggled(bool)", self.onMoveGroupExistsToggled)
        self.ui.planGroupComboBox.connect("activated(int)", self.onPlanGroupActivated)
        self.ui.endEffectorLinkComboBox.connect("activated(int)", self.onEndEffectorLinkActivated)
        self.ui.planButton.connect("clicked(bool)", self.onPlanButton)
        self.ui.previewButton.connect("clicked(bool)", self.onPreviewButton)
        self.ui.executeButton.connect("clicked(bool)", self.onExecuteButton)
        self.ui.addObstacleButton.connect("clicked(bool)", self.onAddObstacle)

        # Populate generator combo box
        for gen in TrajectoryGenerators.get_all():
            self.ui.generatorComboBox.addItem(gen.name)
        self.ui.generatorComboBox.currentIndexChanged.connect(self.onGeneratorChanged)

        # Set appearance collapsible button to be collapsed and disabled initially
        self.ui.appCollapsibleButton.collapsed = True
        self.ui.appCollapsibleButton.enabled = False
        self.ui.jointStateCollapsibleButton.collapsed = True
        self.ui.jointStateCollapsibleButton.enabled = False
        self.ui.moveitCollapsibleButton.collapsed = True
        self.ui.moveitCollapsibleButton.enabled = False
        self.ui.planButton.enabled = False
        self.ui.previewButton.enabled = False
        self.ui.executeButton.enabled = False
        self.ui.planGroupComboBox.enabled = False
        self.ui.endEffectorLinkComboBox.enabled = False
        self.ui.endEffectorLinkLabel.enabled = False
        self.ui.planningTimeSpinBox.enabled = False
        self.ui.planningTimeLabel.enabled = False
        self.onGeneratorChanged(self.ui.generatorComboBox.currentIndex)

        # Make sure parameter node is initialized (needed for module reload)
        self.initializeParameterNode()

    def cleanup(self) -> None:
        """Called when the application closes and the module widget is destroyed."""
        # Stop trajectory animation if running
        if self.trajectoryTimer:
            self.trajectoryTimer.stop()
        # Stop streaming and remove observers before cleanup
        if self.logic:
            self.logic.removeObserver()
            self.logic.ClearJointStateSubscriber()
        if self._moveGroupParamNode is not None:
            try:
                if self._moveGroupParamObsId is not None:
                    self._moveGroupParamNode.RemoveObserver(self._moveGroupParamObsId)
                slicer.mrmlScene.RemoveNode(self._moveGroupParamNode)
            except Exception:
                pass
            self._moveGroupParamNode = None
        self.removeObservers()

    def enter(self) -> None:
        """Called each time the user opens this module."""
        # Make sure parameter node exists and observed
        self.initializeParameterNode()

    def exit(self) -> None:
        """Called each time the user opens a different module."""
        # Stop streaming when exiting module
        if self.logic:
            self.logic.removeObserver()
        self.exitControlMode()
        # Do not react to parameter node changes (GUI will be updated when the user enters into the module)
        if self._parameterNode:
            self._parameterNode.disconnectGui(self._parameterNodeGuiTag)
            self._parameterNodeGuiTag = None
            self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._onParameterNodeModified)

    @vtk.calldata_type(vtk.VTK_OBJECT)
    def onNodeAboutToBeRemoved(self, caller, event, callData) -> None:
        if self.robot and callData == self.robot:
            print("Motion Control: Robot node about to be removed. Disabling UI and cleaning up.")
            if self.logic:
                self.logic.ClearJointStateSubscriber()
            self.exitControlMode()
            self.ui.appCollapsibleButton.enabled = False
            self.ui.jointStateCollapsibleButton.enabled = False
            self.ui.moveitCollapsibleButton.enabled = False
            self.isRobotLoaded = False
            self.robot = None

            # Clean up dynamic sliders
            container = self.ui.JointTab.layout()
            if container is not None:
                for i in reversed(range(container.count())):
                    item = container.itemAt(i)
                    widget = item.widget()
                    if widget == self.ui.zeroPushButton:
                        continue
                    if widget is not None:
                        container.takeAt(i)
                        widget.deleteLater()
            self.jointSliders = []
            self.jointSpinboxes = []
            self.jointPositionsRad = []

            # Clean up move group param
            if self._moveGroupParamNode is not None:
                try:
                    if self._moveGroupParamObsId is not None:
                        self._moveGroupParamNode.RemoveObserver(self._moveGroupParamObsId)
                    slicer.mrmlScene.RemoveNode(self._moveGroupParamNode)
                except Exception:
                    pass
                self._moveGroupParamNode = None
                self._moveGroupParamObsId = None

            # Disable Joint Control buttons
            self.ui.zeroPushButton.enabled = False
            self.ui.currentStatePushButton.enabled = False

            # Disable 3D Control buttons
            self.ui.zeroPushButton3DControl.enabled = False
            self.ui.lastGoalPushButton3DControl.enabled = False
            self.ui.currentStatePushButton3DControl.enabled = False

            # Disable Trajectory buttons
            self.ui.planButton.enabled = False
            self.ui.previewButton.enabled = False
            self.ui.executeButton.enabled = False

            # Remove motion control node from the scene
            if self.motionControlNode is not None:
                try:
                    slicer.mrmlScene.RemoveNode(self.motionControlNode)
                except Exception:
                    pass
                self.motionControlNode = None

    def onSceneStartClose(self, caller, event) -> None:
        """Called just before the scene is closed."""
        # Parameter node will be reset, do not use it anymore
        self.setParameterNode(None)

    def onSceneEndClose(self, caller, event) -> None:
        """Called just after the scene is closed."""
        # If this module is shown while the scene is closed then recreate a new parameter node immediately
        if self.parent.isEntered:
            self.initializeParameterNode()

    def initializeParameterNode(self) -> None:
        """Ensure parameter node exists and observed."""
        self.setParameterNode(self.logic.getParameterNode())

    def setParameterNode(self, inputParameterNode: Optional[ROS2MotionControlParameterNode]) -> None:
        """Set and observe parameter node."""
        if self._parameterNode:
            self._parameterNode.disconnectGui(self._parameterNodeGuiTag)
            self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._onParameterNodeModified)
        self._parameterNode = inputParameterNode
        if self._parameterNode:
            self._parameterNodeGuiTag = self._parameterNode.connectGui(self.ui)
            self.addObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._onParameterNodeModified)
            self._syncUiFromParameterNode()

    def _onParameterNodeModified(self, caller=None, event=None) -> None:
        """Called whenever the parameter node changes; keeps derived UI state in sync."""
        if self._parameterNode is None:
            return
        # Propagate moveGroupExists flag to logic
        if self.logic:
            self.logic.useMoveItIK = self._parameterNode.moveGroupExists
        self._syncUiFromParameterNode()

    def _syncUiFromParameterNode(self) -> None:
        """Push parameter-node values into widgets that are not auto-connected via SlicerParameterName."""
        if self._syncingFromParameterNode or not self.logic or not self._parameterNode or not self._parameterNode.robotNodeID:
            return

        # If UI hasn't been activated for this robot yet, do it now.
        if self.robot is None or self.robot.GetID() != self._parameterNode.robotNodeID:
            robotNode = slicer.mrmlScene.GetNodeByID(self._parameterNode.robotNodeID)
            if robotNode:
                self._syncingFromParameterNode = True
                try:
                    # Ensure the backend logic is fully set up (idempotent)
                    if self.logic.SetupRobotForMotionControl(self._parameterNode):
                        self._activateRobotUi(robotNode)
                finally:
                    self._syncingFromParameterNode = False

    def _checkCanApply(self, caller=None, event=None) -> None:
        pass

    def onUseButton(self) -> None:
        # Stop any prior streaming callbacks
        self.logic.removeObserver()
        self.logic.ClearJointStateSubscriber()

        # Get robot node
        robotNode = self.ui.ikrobotcombobox.currentNode()
        if not robotNode:
            print("Error: No robot selected.")
            return

        if self._parameterNode:
            self._syncingFromParameterNode = True
            try:
                # Update topic from UI if necessary before setup
                self._parameterNode.jointStateTopic = self.ui.jointStateTopicLineEdit.text.strip() or "joint_states"
                self._parameterNode.robotNodeID = robotNode.GetID()
            finally:
                self._syncingFromParameterNode = False
            if not self.logic.SetupRobotForMotionControl(self._parameterNode):
                return

        self._activateRobotUi(robotNode)

    def _activateRobotUi(self, robotNode) -> None:
        """Initializes the UI state (sliders, buttons) for the given robot."""
        self.robot = robotNode
        rootandtip = robotNode.FindRootAndTipLinks()
        if rootandtip and len(rootandtip) >= 2:
            self.rootlink, self.tiplink = rootandtip[0], rootandtip[1]
            self.goaltiplink = self.tiplink

        if self._parameterNode:
            self.motionControlNode = slicer.mrmlScene.GetNodeByID(self._parameterNode.motionControlNodeID)

        joint_names = self.logic.joint_names

        goal_rgb = self.logic.GetGoalRobotColor(self.robot)
        if goal_rgb is not None:
            color = qt.QColor()
            color.setRgbF(goal_rgb[0], goal_rgb[1], goal_rgb[2])
            self.ui.robotColorButton.blockSignals(True)
            self.ui.robotColorButton.color = color
            self.ui.robotColorButton.blockSignals(False)
            if self._parameterNode is not None:
                self._parameterNode.goalColor = color.name()

        # Seed initial joint positions from /joint_states; fall back to zeros.
        initial_joint_pos = self.logic.GetCurrentJointState(joint_names)
        if not initial_joint_pos:
            print("Warning: No joint state message received yet; initializing to zeros. Sliders will update automatically.")
            initial_joint_pos = [0.0] * len(joint_names)
            self._sliderInitRetryCount = 0
            qt.QTimer.singleShot(200, self._trySyncSlidersFromJointState)
        self.logic.last_ik_solution = initial_joint_pos
        self.jointPositionsRad = list(initial_joint_pos)

        if DEBUG:
            print(f"CURRENT: Root link={self.rootlink}, Tip link={self.tiplink}, Goal Tip Link={self.goaltiplink}")
            print(f"Initial Joint Positions (rad): {[f'{j:.4f}' for j in initial_joint_pos]}")

        # Enable buttons
        self.ui.appCollapsibleButton.collapsed = False
        self.ui.appCollapsibleButton.enabled = True
        self.ui.jointStateCollapsibleButton.collapsed = False
        self.ui.jointStateCollapsibleButton.enabled = True
        self.ui.moveitCollapsibleButton.collapsed = False
        self.ui.moveitCollapsibleButton.enabled = True

        # Try to populate planning group dropdown from /move_group's SRDF parameter
        self._setupMoveGroupDropdown(robotNode)
        if self._parameterNode is not None:
            move_group_exists = self._parameterNode.moveGroupExists
            if self.ui.moveGroupExistsCheckBox.checked != move_group_exists:
                was_blocked = self.ui.moveGroupExistsCheckBox.blockSignals(True)
                self.ui.moveGroupExistsCheckBox.checked = move_group_exists
                self.ui.moveGroupExistsCheckBox.blockSignals(was_blocked)
            self.onMoveGroupExistsToggled(move_group_exists)

        self.updateObstacleTable()

        # Create Joint Sliders Dynamically
        self.ui.zeroPushButton.enabled = True
        self.ui.currentStatePushButton.enabled = True
        self.ui.zeroPushButton3DControl.enabled = True
        self.ui.lastGoalPushButton3DControl.enabled = True
        self.ui.currentStatePushButton3DControl.enabled = True

        # Build joint limit dict from C++ robot node
        _lower_list = list(robotNode.GetJointLowerPositionLimits())
        _upper_list = list(robotNode.GetJointUpperPositionLimits())
        _type_list  = list(robotNode.GetJointTypes())
        limits = {
            name: (_lower_list[i], _upper_list[i], _type_list[i])
            for i, name in enumerate(joint_names)
            if i < len(_lower_list)
        }
        container = self.ui.JointTab.layout()
        if container is not None:
            # FIX: Iterate backwards to delete dynamic items but KEEP the zero button
            for i in reversed(range(container.count())):
                item = container.itemAt(i)
                widget = item.widget()

                # If this is your specific button, skip it!
                if widget == self.ui.zeroPushButton:
                    continue

                # Otherwise, remove from layout and destroy
                if widget is not None:
                    container.takeAt(i)
                    widget.deleteLater()

            # Create sliders dynamically
            self.jointSliders = []
            self.jointSpinboxes = []
            for i, joint_name in enumerate(joint_names):
                # --- 1. SETUP MAIN CONTAINER (Vertical: Label Top, Controls Bottom) ---
                joint_block_widget = qt.QWidget()
                joint_block_layout = qt.QVBoxLayout(joint_block_widget)
                joint_block_layout.setContentsMargins(0, 5, 0, 5) # Add small vertical spacing between joints
                joint_block_layout.setSpacing(2) # Reduce gap between label and slider

                # --- 2. SETUP CONTROLS CONTAINER (Horizontal: Slider Left, Spinbox Right) ---
                controls_layout = qt.QHBoxLayout()
                controls_layout.setContentsMargins(0, 0, 0, 0)

                # Create Widgets
                joint_label = qt.QLabel(joint_name)
                # Optional: Make label bold or smaller if you want
                # joint_label.setStyleSheet("font-weight: bold;")

                joint_slider = qt.QSlider(qt.Qt.Horizontal)
                joint_spinbox = qt.QDoubleSpinBox()

                # --- 3. CALCULATE LIMITS ---
                lo_hi = limits.get(joint_name)
                if lo_hi:
                    jtype = lo_hi[2] if len(lo_hi) > 2 else "revolute"
                    if jtype == "prismatic":
                        lo_ui = lo_hi[0] * 1000.0
                        hi_ui = lo_hi[1] * 1000.0
                    else:
                        lo_ui = math.degrees(lo_hi[0])
                        hi_ui = math.degrees(lo_hi[1])

                    if lo_ui > hi_ui:
                        lo_ui, hi_ui = hi_ui, lo_ui
                else:
                    jtype = "revolute"
                    lo_ui, hi_ui = -180.0, 180.0

                lo_ui_int = int(round(lo_ui))
                hi_ui_int = int(round(hi_ui))

                # Determine initial value
                initial_val_rad = self.jointPositionsRad[i] if self.jointPositionsRad and i < len(self.jointPositionsRad) else 0.0

                if jtype == "prismatic":
                    initial_val_ui = initial_val_rad * 1000.0
                else:
                    initial_val_ui = math.degrees(initial_val_rad)

                initial_val_ui_int = int(round(initial_val_ui))
                initial_val_ui_int = max(lo_ui_int, min(hi_ui_int, initial_val_ui_int))

                # Store jtype on UI elements for later use
                joint_slider.setProperty("jtype", jtype)
                joint_spinbox.setProperty("jtype", jtype)

                # --- 4. CONFIGURE SLIDER ---
                joint_slider.setMinimum(lo_ui_int)
                joint_slider.setMaximum(hi_ui_int)
                joint_slider.setValue(initial_val_ui_int)
                joint_slider.setTickInterval(10)
                joint_slider.setTickPosition(qt.QSlider.TicksBelow)

                # --- 5. CONFIGURE SPINBOX ---
                joint_spinbox.setMinimum(lo_ui)
                joint_spinbox.setMaximum(hi_ui)

                if jtype == "prismatic":
                    joint_spinbox.setSingleStep(1.0)
                    joint_spinbox.setSuffix(" mm")
                else:
                    joint_spinbox.setSingleStep(1.0)
                    joint_spinbox.setSuffix(" deg")

                joint_spinbox.setValue(initial_val_ui)

                # --- 6. SYNC LOGIC ---
                # A. Slider moves -> Update Spinbox
                joint_slider.valueChanged.connect(lambda val, sb=joint_spinbox: sb.setValue(val))

                # B. Spinbox changes -> Update Slider AND IK Logic
                joint_spinbox.valueChanged.connect(lambda val, sl=joint_slider: sl.setValue(int(round(val))))
                joint_spinbox.valueChanged.connect(lambda value, idx=i: self.onJointSliderChanged(idx, value))

                # --- 7. ADD TO LAYOUTS ---

                # Add Slider + Spinbox to the Horizontal controls layout
                controls_layout.addWidget(joint_slider)
                controls_layout.addWidget(joint_spinbox)

                # Add Label and the Controls Layout to the Main Vertical Block
                joint_block_layout.addWidget(joint_label)
                joint_block_layout.addLayout(controls_layout)

                # Add the whole block to your main container
                container.addWidget(joint_block_widget)
                self.jointSliders.append(joint_slider)
                self.jointSpinboxes.append(joint_spinbox)

        # Set robot true
        self.isRobotLoaded = True

    # Opacity slider handler
    def onOpacitySliderChanged(self, value: int) -> None:
        if self.logic is None or self.robot is None:
            return
        opacity = value / 100.0
        self.logic.setOpacity(self.robot, opacity)

    # Robot color button handler
    def onRobotColorChanged(self) -> None:
        if self.logic is None or self.robot is None:
            return
        color = self.ui.robotColorButton.color
        self.logic.setGoalRobotColor(self.robot, color)
        # Persist the new goal color in the parameter node
        if self._parameterNode is not None:
            self._parameterNode.goalColor = color.name()

    # Joint slider change handler
    def onJointSliderChanged(self, idx: int, value: float) -> None:
        # Ensure array is large enough
        while len(self.jointPositionsRad) <= idx:
            self.jointPositionsRad.append(0.0)

        jtype = self.jointSliders[idx].property("jtype") if idx < len(self.jointSliders) else "revolute"

        if jtype == "prismatic":
            self.jointPositionsRad[idx] = value / 1000.0
        else:
            self.jointPositionsRad[idx] = math.radians(value)

        # Update goal robot transforms with new joint positions
        if self.logic is not None and self.robot is not None:
            self.logic.updategoalTransformsFromJointsKDL(self.robot, self.jointPositionsRad)
            # Keep last_ik_solution in sync so onPlanButton uses the slider-driven goal
            self.logic.last_ik_solution = self.jointPositionsRad.copy()
        if DEBUG:
            print(f"All joint values (rad/m): {[f'{j:.4f}' for j in self.jointPositionsRad]}")

    # Zero button handler
    def onZeroButton(self) -> None:

        print("Resetting joint sliders to zero.")

        container = self.ui.JointTab.layout()
        if container is None:
            return

        sliders_found = []

        # 1. Iterate through the main layout to find the Row Widgets
        for i in range(container.count()):
            item = container.itemAt(i)
            widget = item.widget()

            # Skip empty items or the zero button itself
            if widget is None or widget == self.ui.zeroPushButton:
                continue

            # 2. Look INSIDE the widget for the Slider and Spinbox
            # findChild searches the children of the widget
            slider = widget.findChild(qt.QSlider)
            spinbox = widget.findChild(qt.QDoubleSpinBox)

            # If both exist, this is a valid joint row
            if slider and spinbox:
                sliders_found.append((slider, spinbox))

        if not sliders_found:
            print("No sliders found to reset.")
            return

        # 3. Reset values
        for slider, spinbox in sliders_found:
            # Block signals on BOTH so we don't trigger 6 separate IK updates
            # or cause the two widgets to fight each other
            slider.blockSignals(True)
            spinbox.blockSignals(True)

            slider.setValue(0)
            spinbox.setValue(0)

            slider.blockSignals(False)
            spinbox.blockSignals(False)

        # Reset stored joint positions to match slider count
        self.jointPositionsRad = [0.0] * len(sliders_found)

        # Update goal robot with zero positions
        if self.logic is not None and self.robot is not None:
            self.logic.updategoalTransformsFromJointsKDL(self.robot, self.jointPositionsRad)
            self.logic.last_ik_solution = self.jointPositionsRad.copy()
            self._syncProbeToCurrentTipPose()

    def onLastGoalButton(self) -> None:
        """Resets the tip transform node to match the current goal model position."""
        if self.logic is None or self.fromtransform is None:
            return
        self._syncProbeToCurrentTipPose()
        if DEBUG:
            print("Probe (Last Goal) snapped to goal robot tip.")

    def _trySyncSlidersFromJointState(self) -> None:
        """Retry syncing sliders from the joint state subscriber.
        Called by a QTimer when the subscriber had no data at slider-build time.
        Retries every 200 ms for up to 5 seconds."""
        if not self.jointSliders or self.logic is None or self.robot is None:
            return  # sliders were torn down or module unloaded
        joint_names = list(self.robot.GetJoints())
        live = self.logic.GetCurrentJointState(joint_names)
        if live:
            self.jointPositionsRad = live
            self.logic.last_ik_solution = live.copy()
            self.logic.updategoalTransformsFromJointsKDL(self.robot, live)
            self._setJointUi_SIToSlicer(live)
            self._syncProbeToCurrentTipPose()
            print("Joint sliders initialized from live /joint_states.")
        elif self._sliderInitRetryCount < 25:  # up to ~5 s
            self._sliderInitRetryCount += 1
            qt.QTimer.singleShot(200, self._trySyncSlidersFromJointState)
        else:
            print("Warning: /joint_states not received after 5 s; sliders remain at zero.")

    def onCurrentStateButton(self) -> None:
        if self.logic is None or self.robot is None:
            print("Current state: robot is not initialized")
            return

        joint_names = self.robot.GetJoints()
        if not joint_names:
            print("Current state: no joints found on robot")
            return

        joint_values = self.logic.GetCurrentJointState(joint_names)
        if not joint_values:
            print("Current state: failed to read positions from JointState subscriber")
            return

        if len(joint_values) != len(joint_names):
            print(f"Current state: joint count mismatch ({len(joint_values)} vs {len(joint_names)})")
            return

        self.jointPositionsRad = joint_values.copy()
        self.logic.last_ik_solution = joint_values.copy()
        self.logic.updategoalTransformsFromJointsKDL(self.robot, joint_values)
        self._setJointUi_SIToSlicer(joint_values)
        self._syncProbeToCurrentTipPose()
        if DEBUG:
            print(f"Current state applied from subscriber: {[f'{j:.4f}' for j in joint_values]}")

    def _syncProbeToCurrentTipPose(self) -> None:
        if self.logic is None or self.fromtransform is None:
            return

        # Work only with goal tree
        tip_transform_node = None
        try:
            if self.goaltiplink:
                tip_transform_node = self.logic.findRobotTransforms(self.goaltiplink, goal=True)
        except Exception as e:
            print(f"Could not find goal tip transform: {e}")
            return

        if tip_transform_node is None:
            return

        tip_matrix = vtk.vtkMatrix4x4()
        tip_transform_node.GetMatrixTransformToWorld(tip_matrix)
        self.fromtransform.SetMatrixTransformToParent(tip_matrix)

    def _setJointUi_SIToSlicer(self, joint_values_rad) -> None:
        for i, val in enumerate(joint_values_rad):
            if i >= len(self.jointSliders):
                break
            slider = self.jointSliders[i]
            spinbox = self.jointSpinboxes[i]

            jtype = slider.property("jtype")
            if jtype == "prismatic":
                ui_val = val * 1000.0
            else:
                ui_val = math.degrees(val)

            ui_val_int = int(round(ui_val))

            lo = slider.minimum
            hi = slider.maximum
            clamped_int = max(lo, min(hi, ui_val_int))
            clamped_float = max(spinbox.minimum, min(spinbox.maximum, ui_val))

            slider.blockSignals(True)
            spinbox.blockSignals(True)
            slider.setValue(clamped_int)
            spinbox.setValue(clamped_float)
            slider.blockSignals(False)
            spinbox.blockSignals(False)

    def onAddObstacle(self) -> None:
        modelNode = self.ui.obstacleModelComboBox.currentNode()
        if not modelNode:
            return

        frameId = self.ui.obstacleFrameLineEdit.text.strip()
        if not frameId:
            frameId = "world"

        if not self.logic.AddMoveItObstacle(modelNode, frameId, self.robot):
            return

        modelID = modelNode.GetID()
        self.obstaclePublishers[modelID] = modelNode
        self.removeObserver(modelNode, vtk.vtkCommand.ModifiedEvent, self.onObstacleModified)
        self.addObserver(modelNode, vtk.vtkCommand.ModifiedEvent, self.onObstacleModified)

        self.updateObstacleTable()
        print(f"Obstacle {modelNode.GetName()} added to MoveIt planning scene.")

    def onObstacleModified(self, caller, event) -> None:
        if caller and caller.GetAttribute(self.logic.MOVEIT_OBSTACLE_ATTRIBUTE):
            self.logic.PublishMoveItObstacle(caller, robotNode=self.robot)

    def removeObstacle(self, modelID) -> None:
        modelNode = slicer.mrmlScene.GetNodeByID(modelID)
        if modelNode:
            self.logic.RemoveMoveItObstacle(modelNode, self.robot)
            self.removeObserver(modelNode, vtk.vtkCommand.ModifiedEvent, self.onObstacleModified)

        if modelID in self.obstaclePublishers:
            del self.obstaclePublishers[modelID]
        self.updateObstacleTable()

    def updateObstacleTable(self) -> None:
        self.ui.obstaclesTable.setRowCount(0)
        self.obstaclePublishers = {
            modelID: modelNode
            for modelID, modelNode, _frameId in self.logic.GetMoveItObstacles()
        }
        for modelID, modelNode in self.obstaclePublishers.items():
            self.removeObserver(modelNode, vtk.vtkCommand.ModifiedEvent, self.onObstacleModified)
            self.addObserver(modelNode, vtk.vtkCommand.ModifiedEvent, self.onObstacleModified)
            row = self.ui.obstaclesTable.rowCount
            self.ui.obstaclesTable.insertRow(row)

            # Model Name
            name = modelNode.GetName() if modelNode else modelID
            self.ui.obstaclesTable.setItem(row, 0, qt.QTableWidgetItem(name))

            # Frame ID
            frameId = modelNode.GetAttribute(self.logic.MOVEIT_OBSTACLE_FRAME_ATTRIBUTE) or "world"
            self.ui.obstaclesTable.setItem(row, 1, qt.QTableWidgetItem(frameId))

            # Action Button
            removeBtn = qt.QPushButton("Remove")
            removeBtn.clicked.connect(lambda checked, mID=modelID: self.removeObstacle(mID))
            self.ui.obstaclesTable.setCellWidget(row, 2, removeBtn)

    def onMoveGroupExistsToggled(self, toggled: bool) -> None:
        self.ui.planGroupLabel.enabled = toggled
        self.ui.planGroupComboBox.enabled = toggled
        self.ui.endEffectorLinkLabel.enabled = toggled
        self.ui.endEffectorLinkComboBox.enabled = toggled
        self.ui.planningTimeLabel.enabled = toggled
        self.ui.planningTimeSpinBox.enabled = toggled
        # Persist to parameter node
        if self._parameterNode is not None:
            self._parameterNode.moveGroupExists = toggled
        if toggled:
            print("Move Group Exists checked: MoveIt functionality enabled")
            self.logic.useMoveItIK = True
            group = self.ui.planGroupComboBox.currentText
            if group and self.robot:
                self.logic.SetupMoveItPlanningGroup(self.robot, group)
            self.ui.planButton.enabled = True
        else:
            self.logic.useMoveItIK = False
            self.ui.planButton.enabled = False
            self.ui.previewButton.enabled = False
            self.ui.executeButton.enabled = False

        # If a simple-generator trajectory is already planned, update execute availability
        if self.trajectoryData is not None:
            generators = TrajectoryGenerators.get_all()
            idx = self.ui.generatorComboBox.currentIndex
            if 0 <= idx < len(generators):
                is_moveit_gen = isinstance(generators[idx], TrajectoryGenerators.MoveItTrajectoryGenerator)
                if not is_moveit_gen:
                    self.ui.executeButton.enabled = toggled

    def onPlanGroupActivated(self, index: int) -> None:
        """Called when the user selects or confirms a planning group from the combobox."""
        if not self.ui.moveGroupExistsCheckBox.checked or not self.robot:
            return
        group = self.ui.planGroupComboBox.currentText
        if group:
            if self._parameterNode is not None:
                self._parameterNode.planningGroup = group
            self.logic.SetupMoveItPlanningGroup(self.robot, group)
            print(f"Planning group set to: {group}")

    def _setupMoveGroupDropdown(self, robotNode) -> None:
        """Fetch robot_description_semantic from /move_group and populate planGroupComboBox."""
        self._moveGroupParamNode = None
        try:
            paramNode = self.logic.CreateMoveGroupSRDFParameterNode(robotNode)
            if paramNode is None:
                return
            self._moveGroupParamNode = paramNode

            # If already available (e.g. cached), populate immediately
            if paramNode.IsParameterSet("robot_description_semantic"):
                self._onMoveGroupSRDFReceived()
            else:
                self._moveGroupParamObsId = paramNode.AddObserver(
                    slicer.vtkMRMLROS2ParameterNode.ParameterModifiedEvent,
                    self._onMoveGroupSRDFReceived
                )
        except Exception as e:
            print(f"Warning: Could not set up move group parameter monitoring: {e}")

    def _onMoveGroupSRDFReceived(self, caller=None, event=None) -> None:
        """Parse SRDF XML from /move_group, populate planGroupComboBox and endEffectorLinkComboBox."""
        if self._moveGroupParamNode is None:
            return
        srdf_xml = self._moveGroupParamNode.GetParameterAsString("robot_description_semantic")
        groups, end_effectors = self.logic.ParseMoveGroupSRDF(srdf_xml)

        if not groups:
            return

        # --- Planning group combo ---
        combo = self.ui.planGroupComboBox
        current = self._parameterNode.planningGroup if self._parameterNode is not None else combo.currentText
        combo.blockSignals(True)
        combo.clear()
        for g in groups:
            combo.addItem(g)
        idx = combo.findText(current)
        combo.setCurrentIndex(idx if idx >= 0 else 0)
        combo.blockSignals(False)
        if self._parameterNode is not None:
            selected = combo.currentText
            if selected:
                self._parameterNode.planningGroup = selected
        print(f"Planning groups available: {groups}")

        # --- End effector link combo ---
        self._srdfEndEffectors = end_effectors
        eeCombo = self.ui.endEffectorLinkComboBox
        eeCombo.blockSignals(True)
        eeCombo.clear()
        for ee in end_effectors:
            # Display as "name (parent_link)" so the user sees both pieces of info
            eeCombo.addItem(f"{ee['name']} ({ee['parent_link']})", ee["parent_link"])
        # Pre-select the entry whose parent_link matches the current tiplink (from URDF traversal)
        preselect_idx = -1
        if self.tiplink:
            for i, ee in enumerate(end_effectors):
                if ee["parent_link"] == self.tiplink:
                    preselect_idx = i
                    break
        eeCombo.setCurrentIndex(preselect_idx if preselect_idx >= 0 else 0)
        eeCombo.blockSignals(False)
        if end_effectors:
            # Apply the selected entry immediately so tiplink is authoritative
            selected_link = eeCombo.itemData(eeCombo.currentIndex)
            if selected_link:
                self.tiplink = selected_link
                self.goaltiplink = selected_link
                if self.logic is not None:
                    self.logic.tipLink = selected_link
            print(f"End effectors available: {[ee['name'] for ee in end_effectors]}")
            print(f"Active end effector link: {self.tiplink}")
        else:
            print("No end_effector elements found in SRDF; tip link unchanged.")

        # Remove the observer — no need to keep listening once populated
        if self._moveGroupParamObsId is not None:
            try:
                self._moveGroupParamNode.RemoveObserver(self._moveGroupParamObsId)
            except Exception:
                pass
            self._moveGroupParamObsId = None


    def onEndEffectorLinkActivated(self, index: int) -> None:
        """Called when the user selects an end-effector link from the dropdown.
        Updates tiplink/goaltiplink so subsequent IK calls use the correct frame.
        """
        eeCombo = self.ui.endEffectorLinkComboBox
        link = eeCombo.itemData(index)
        if not link:
            # Editable combo: user may have typed a raw link name
            link = eeCombo.currentText.strip()
        if not link:
            return
        self.tiplink = link
        self.goaltiplink = link
        if self.logic is not None:
            self.logic.tipLink = link
        if self.robot is not None:
            print(f"End effector link set to: {link}")

    def onTabChanged(self, index):
        if not self.isRobotLoaded:
            return

        current_widget = self.ui.tabWidget.widget(index)
        if current_widget == self.ui.controlTab:
            self.enterControlMode()
        else:
            self.exitControlMode()
            self._restoreGoalRobotColor()

        if current_widget == self.ui.loadtab:
            self._refreshRobotTab()
        elif current_widget == self.ui.JointTab:
            self._refreshJointTab()
        elif current_widget == self.ui.moveItTab:
            self._refreshMoveItTab()
        elif current_widget == self.ui.obstaclesTab:
            self._refreshObstaclesTab()

    def _restoreGoalRobotColor(self) -> None:
        if self.logic and self.robot and self._parameterNode is not None:
            c = qt.QColor(self._parameterNode.goalColor)
            base = (c.redF(), c.greenF(), c.blueF())
            self.logic.setGoalRobotColorRGB(self.robot, base)

    def _refreshRobotTab(self) -> None:
        self._syncUiFromParameterNode()

    def _refreshJointTab(self) -> None:
        self._syncJointUiFromLatestSolution()

    def _refreshMoveItTab(self) -> None:
        if self._parameterNode is not None:
            move_group_exists = self._parameterNode.moveGroupExists
            if self.ui.moveGroupExistsCheckBox.checked != move_group_exists:
                was_blocked = self.ui.moveGroupExistsCheckBox.blockSignals(True)
                self.ui.moveGroupExistsCheckBox.checked = move_group_exists
                self.ui.moveGroupExistsCheckBox.blockSignals(was_blocked)
            self.onMoveGroupExistsToggled(move_group_exists)

        if self.robot is not None and self.ui.planGroupComboBox.count == 0:
            self._setupMoveGroupDropdown(self.robot)

    def _refreshObstaclesTab(self) -> None:
        self.updateObstacleTable()

    def _syncJointUiFromLatestSolution(self) -> None:
        if self.logic is None or self.robot is None:
            return

        joint_names = self.robot.GetJoints()
        expected_count = len(joint_names) if joint_names else 0
        if expected_count == 0:
            return

        latest_solution = self.logic.last_ik_solution
        if not latest_solution or len(latest_solution) != expected_count:
            return

        self.jointPositionsRad = list(latest_solution)
        self._setJointUi_SIToSlicer(self.jointPositionsRad)

    def enterControlMode(self):
        if not self.isRobotLoaded or not self.robot or not self.rootlink:
            return
        if self._parameterNode is not None:
            _c = qt.QColor(self._parameterNode.goalColor)
            base_color = (_c.redF(), _c.greenF(), _c.blueF())
        else:
            base_color = None
        state = self.logic.EnterControlMode(
            self.robot,
            self.rootlink,
            self.tiplink,
            self.goaltiplink,
            baseGoalColor=base_color
        )
        if state:
            self.fromtransform = state["fromTransform"]
            self.totransform = state["toTransform"]

    def exitControlMode(self):
        if not self.isRobotLoaded:
            return
        if self.logic:
            self.logic.ExitControlMode(self.fromtransform)
        self.fromtransform = None
        self.totransform = None

    def onGeneratorChanged(self, index: int) -> None:
        """Enable/disable Plan button based on selected generator and MoveIt state."""
        generators = TrajectoryGenerators.get_all()
        if index < 0 or index >= len(generators):
            return
        is_moveit = isinstance(generators[index], TrajectoryGenerators.MoveItTrajectoryGenerator)
        if is_moveit:
            self.ui.planButton.enabled = self.ui.moveGroupExistsCheckBox.checked
        else:
            self.ui.planButton.enabled = True

    def onPlanButton(self) -> None:
        generators = TrajectoryGenerators.get_all()
        idx = self.ui.generatorComboBox.currentIndex
        if idx < 0 or idx >= len(generators):
            print("No trajectory generator selected.")
            return
        generator = generators[idx]

        joint_names = list(self.robot.GetJoints()) if self.robot else []
        # Always read the latest joint state from the subscriber so the trajectory
        # starts from the true current robot position, not a stale cached value.
        live_positions = self.logic.GetCurrentJointState(joint_names)
        if live_positions:
            self.jointPositionsRad = live_positions
        start_positions = self.jointPositionsRad if self.jointPositionsRad else [0.0] * len(joint_names)
        goal_positions = self.logic.last_ik_solution if self.logic.last_ik_solution else start_positions

        self.logic.PublishAllMoveItObstacles(self.robot)

        sol = generator.plan(
            joint_names=joint_names,
            start_positions=start_positions,
            goal_positions=goal_positions,
            robot=self.robot,
            motion_control_node=self.motionControlNode,
            plan_group=(
                self._parameterNode.planningGroup
                if self._parameterNode is not None
                else self.ui.planGroupComboBox.currentText
            ),
            planning_time=(
                self._parameterNode.planningTimeSec
                if self._parameterNode is not None
                else self.ui.planningTimeSpinBox.value
            ),
            velocity_scaling=(
                self._parameterNode.velocityScaling
                if self._parameterNode is not None
                else 0.5
            ),
            acceleration_scaling=(
                self._parameterNode.accelerationScaling
                if self._parameterNode is not None
                else 0.5
            ),
        )

        # Store and display the trajectory
        if sol is not None and sol.GetJointTrajectory().GetPoints():
            is_moveit_gen = isinstance(generator, TrajectoryGenerators.MoveItTrajectoryGenerator)
            move_group_ready = self._parameterNode.moveGroupExists if self._parameterNode is not None else self.ui.moveGroupExistsCheckBox.checked
            self.ui.previewButton.enabled = True
            self.ui.executeButton.enabled = is_moveit_gen or move_group_ready
            self.trajectoryData = sol
            num_points = len(sol.GetJointTrajectory().GetPoints())

            # Remove old slider if it exists
            if self.trajectorySlider:
                if self.trajectorySliderWidget:
                    self.trajectorySliderWidget.deleteLater()
                self.trajectorySlider = None
                self.trajectorySliderWidget = None
                self.trajectorySpinBox = None

            # Create new trajectory scrubber slider
            self.trajectorySliderWidget = qt.QWidget()
            layout = qt.QVBoxLayout(self.trajectorySliderWidget)
            layout.setContentsMargins(0, 10, 0, 0)

            # Create label
            label = qt.QLabel("Trajectory Scrubber:")
            layout.addWidget(label)

            # Create horizontal layout for slider and spinbox
            sliderLayout = qt.QHBoxLayout()

            # Create slider
            self.trajectorySlider = qt.QSlider(qt.Qt.Horizontal)
            self.trajectorySlider.setMinimum(0)
            self.trajectorySlider.setMaximum(num_points - 1)
            self.trajectorySlider.setValue(0)
            self.trajectorySlider.valueChanged.connect(self.onTrajectorySliderChanged)
            sliderLayout.addWidget(self.trajectorySlider)

            # Create spinbox to show point number
            self.trajectorySpinBox = qt.QSpinBox()
            self.trajectorySpinBox.setMinimum(0)
            self.trajectorySpinBox.setMaximum(num_points - 1)
            self.trajectorySpinBox.setValue(0)
            self.trajectorySpinBox.setSuffix(f" / {num_points - 1}")
            self.trajectorySpinBox.valueChanged.connect(lambda val: self.trajectorySlider.setValue(val))
            sliderLayout.addWidget(self.trajectorySpinBox)

            layout.addLayout(sliderLayout)

            # Add to the moveittab layout
            moveitLayout = self.ui.moveItTab.layout()
            if moveitLayout:
                moveitLayout.addWidget(self.trajectorySliderWidget)

            # Show first point of trajectory
            if self.robot and num_points > 0:
                positions = list(sol.GetJointTrajectory().GetPoints()[0].GetPositions())
                self.logic.updategoalTransformsFromJointsKDL(self.robot, positions)
        elif sol is not None:
            print("Error: trajectory returned by generator has no points")

    def _resetTrajectoryState(self) -> None:
        """Clear planned trajectory and reset UI to pre-plan state."""
        self.trajectoryData = None
        self.trajectoryIndex = 0
        self.ui.previewButton.enabled = False
        self.ui.executeButton.enabled = False
        if self.trajectoryTimer:
            self.trajectoryTimer.stop()
            self.trajectoryTimer = None
        if self.trajectorySlider:
            if self.trajectorySliderWidget:
                self.trajectorySliderWidget.deleteLater()
            self.trajectorySlider = None
            self.trajectorySliderWidget = None
            self.trajectorySpinBox = None

    def onPreviewButton(self) -> None:
        """Preview the planned trajectory on the goal robot"""
        if not self.trajectoryData:
            print("No trajectory to preview. Run Plan first.")
            return

        # Stop any existing animation
        if self.trajectoryTimer:
            self.trajectoryTimer.stop()

        # Start animating the trajectory
        self.trajectoryIndex = 0
        self.trajectoryTimer = qt.QTimer()
        self.trajectoryTimer.timeout.connect(self.animateTrajectoryStep)
        self.trajectoryTimer.start(50)  # Update every 50ms

    def animateTrajectoryStep(self):
        """Animate one step of the trajectory"""
        if not self.trajectoryData or self.trajectoryIndex >= len(self.trajectoryData.GetJointTrajectory().GetPoints()):
            # Animation complete
            if self.trajectoryTimer:
                self.trajectoryTimer.stop()
            print("Trajectory preview complete")
            return

        # Get current point
        point = self.trajectoryData.GetJointTrajectory().GetPoints()[self.trajectoryIndex]
        positions = list(point.GetPositions())

        # Apply to goal robot
        if self.robot:
            self.logic.updategoalTransformsFromJointsKDL(self.robot, positions)

        self.trajectoryIndex += 1

    def onTrajectorySliderChanged(self, value):
        """Called when trajectory slider is moved"""
        if not self.trajectoryData or not self.robot:
            return

        # Stop any running animation
        if self.trajectoryTimer:
            self.trajectoryTimer.stop()

        # Get the trajectory point at this index
        if 0 <= value < len(self.trajectoryData.GetJointTrajectory().GetPoints()):
            positions = list(self.trajectoryData.GetJointTrajectory().GetPoints()[value].GetPositions())

            # Apply to goal robot
            self.logic.updategoalTransformsFromJointsKDL(self.robot, positions)

            # Update spinbox if it's not the source of the change
            if self.trajectorySpinBox.value != value:
                self.trajectorySpinBox.blockSignals(True)
                self.trajectorySpinBox.setValue(value)
                self.trajectorySpinBox.blockSignals(False)

    def onExecuteButton(self) -> None:
        if self.motionControlNode is None:
            print("No motion control node available. Please use the robot first.")
            return
        if self.trajectoryData is None:
            print("No trajectory planned. Please plan a trajectory first.")
            return
        group = (
            self._parameterNode.planningGroup
            if self._parameterNode is not None
            else self.ui.planGroupComboBox.currentText
        )
        # Always execute from the stored vtk trajectory (works for both MoveIt and Simple generators).
        # ExecuteMoveItTrajectoryAsync returns True as soon as the thread is launched — the actual
        # MoveIt result is not known here. We therefore keep the plan loaded so the user can
        # see what was sent and re-execute if needed.
        success = self.motionControlNode.ExecuteMoveItTrajectoryAsync(group, self.trajectoryData)

        if success:
            print("Trajectory sent to controller (async). Check robot / MoveIt output for result.")
        else:
            print("Failed to send trajectory (pre-flight check failed — see console).")



#
# ROS2MotionControlLogic
#


class ROS2MotionControlLogic(ScriptedLoadableModuleLogic):
    """This class should implement all the actual
    computation done by your module.  The interface
    should be such that other python code can import
    this class and make use of the functionality without
    requiring an instance of the Widget.
    Uses ScriptedLoadableModuleLogic base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self) -> None:
        """Called when the logic class is instantiated. Can be used for initializing member variables."""
        ScriptedLoadableModuleLogic.__init__(self)
        self.MOVEIT_COLLISION_OBJECT_TOPIC = "/collision_object"
        self.MOVEIT_OBSTACLE_ATTRIBUTE = "ROS2MotionControl.MoveItObstacle"
        self.MOVEIT_OBSTACLE_FRAME_ATTRIBUTE = "ROS2MotionControl.MoveItObstacleFrame"
        self.obsTag = None
        self.obsNode = None
        self.callback = None
        self.viewObserverTags = []
        self.isInteracting = False
        self.toNode = None
        self.tipLink = None  # Will be set from widget
        self.last_ik_solution = []  # Will be sized based on actual joint count
        self.joint_names = []  # Will be populated from URDF
        self.joint_state_subscriber = None
        self.joint_state_subscriber_observer = None
        self.joint_state_last_message = None
        self.joint_state_topic = None
        self.joint_state_ros2_node = None
        self.joint_state_subscriber_owned = False
        self.useMoveItIK = False  # Runtime flag; kept in sync with param node by widget
        self._last_moveit_obstacle_publish_time = 0.0

    def _NormalizeJointStateTopic(self, topic_name: str) -> str:
        topic = (topic_name or "").strip()
        if not topic:
            topic = "joint_states"
        if not topic.startswith("/"):
            topic = f"/{topic}"
        return topic

    def _getObstacleROS2Node(self, robotNode=None):
        if robotNode is not None and robotNode.GetNodeReference("node"):
            return robotNode.GetNodeReference("node")
        return slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLROS2NodeNode")

    def _getCollisionObjectPublisher(self, robotNode=None, create=True):
        ros2Node = self._getObstacleROS2Node(robotNode)
        if not ros2Node:
            return None

        pub = ros2Node.GetPublisherNodeByTopic(self.MOVEIT_COLLISION_OBJECT_TOPIC)
        if pub:
            return pub

        if not create:
            return None
        return ros2Node.CreateAndAddPublisherNode("CollisionObject", self.MOVEIT_COLLISION_OBJECT_TOPIC)

    def SetupMoveItPlanningGroup(self, robotNode, groupName) -> bool:
        if not robotNode or not groupName:
            return False
        return bool(robotNode.SetupIKMoveIt(groupName))

    def CreateMoveGroupSRDFParameterNode(self, robotNode):
        if not robotNode:
            return None

        ros2Node = robotNode.GetNodeReference("node")
        if ros2Node is None:
            ros2Node = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLROS2NodeNode")
        if ros2Node is None:
            return None

        paramNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLROS2ParameterNode")
        paramNode.SetName("_moveGroupSRDFParam")
        paramNode.AddToROS2Node(ros2Node.GetID(), "/move_group")
        paramNode.AddParameter("robot_description_semantic")
        return paramNode

    def ParseMoveGroupSRDF(self, srdf_xml):
        if not srdf_xml:
            return [], []

        try:
            root = ET.fromstring(srdf_xml)
        except Exception as e:
            print(f"Warning: Could not parse SRDF: {e}")
            return [], []

        groups = []
        for group in root.findall("group"):
            name = group.get("name")
            if name:
                groups.append(name)

        end_effectors = []
        for ee in root.findall("end_effector"):
            name = ee.get("name")
            parent_link = ee.get("parent_link")
            if name and parent_link:
                end_effectors.append({"name": name, "parent_link": parent_link})

        return groups, end_effectors

    def AddMoveItObstacle(self, modelNode, frameId="world", robotNode=None) -> bool:
        if not modelNode:
            print("Add obstacle: model node is invalid")
            return False

        frameId = (frameId or "world").strip() or "world"
        modelNode.SetAttribute(self.MOVEIT_OBSTACLE_ATTRIBUTE, "1")
        modelNode.SetAttribute(self.MOVEIT_OBSTACLE_FRAME_ATTRIBUTE, frameId)
        if not self.PublishMoveItObstacle(modelNode, frameId, robotNode):
            return False

        for delay_ms in (250, 1000):
            qt.QTimer.singleShot(
                delay_ms,
                lambda node=modelNode, frame=frameId, robot=robotNode: self.PublishMoveItObstacle(node, frame, robot)
            )
        return True

    def PublishMoveItObstacle(self, modelNode, frameId=None, robotNode=None) -> bool:
        if not modelNode:
            return False

        pub = self._getCollisionObjectPublisher(robotNode, create=True)
        if not pub:
            print("Add obstacle: no ROS2 node found to host the CollisionObject publisher")
            return False

        frameId = frameId or modelNode.GetAttribute(self.MOVEIT_OBSTACLE_FRAME_ATTRIBUTE) or "world"
        pub.SetFrameId(frameId)
        pub.SetSourceNodeID(modelNode.GetID())
        pub.Publish()
        return True

    def PublishAllMoveItObstacles(self, robotNode=None) -> None:
        for _modelID, modelNode, frameId in self.GetMoveItObstacles():
            self.PublishMoveItObstacle(modelNode, frameId, robotNode)

    def RemoveMoveItObstacle(self, modelNode, robotNode=None) -> bool:
        if not modelNode:
            return False

        pub = self._getCollisionObjectPublisher(robotNode, create=False)
        if pub:
            pub.SetFrameId(modelNode.GetAttribute(self.MOVEIT_OBSTACLE_FRAME_ATTRIBUTE) or "world")
            pub.PublishRemove(modelNode)
            qt.QTimer.singleShot(250, lambda node=modelNode, publisher=pub: publisher.PublishRemove(node))

        modelNode.SetAttribute(self.MOVEIT_OBSTACLE_ATTRIBUTE, None)
        modelNode.SetAttribute(self.MOVEIT_OBSTACLE_FRAME_ATTRIBUTE, None)
        return True

    def GetMoveItObstacles(self):
        by_id = {}
        for modelNode in slicer.util.getNodesByClass("vtkMRMLModelNode"):
            if modelNode.GetAttribute(self.MOVEIT_OBSTACLE_ATTRIBUTE):
                by_id[modelNode.GetID()] = (
                    modelNode,
                    modelNode.GetAttribute(self.MOVEIT_OBSTACLE_FRAME_ATTRIBUTE) or "world"
                )

        for pub in slicer.util.getNodesByClass("vtkMRMLROS2PublisherCollisionObjectNode"):
            if pub.GetTopic() != self.MOVEIT_COLLISION_OBJECT_TOPIC:
                continue
            modelNode = pub.GetSourceNode()
            if not modelNode:
                continue
            if modelNode.GetID() not in by_id:
                frameId = pub.GetFrameId() or "world"
                modelNode.SetAttribute(self.MOVEIT_OBSTACLE_ATTRIBUTE, "1")
                modelNode.SetAttribute(self.MOVEIT_OBSTACLE_FRAME_ATTRIBUTE, frameId)
                by_id[modelNode.GetID()] = (modelNode, frameId)

        return [(modelID, modelNode, frameId) for modelID, (modelNode, frameId) in by_id.items()]

    def SetupRobotForMotionControl(self, parameterNode) -> bool:
        if not parameterNode or not parameterNode.robotNodeID:
            print("SetupRobotForMotionControl: Invalid parameter node or robotNodeID")
            return False

        robotNode = slicer.mrmlScene.GetNodeByID(parameterNode.robotNodeID)
        if not robotNode:
            print("SetupRobotForMotionControl: Could not find robot node in scene")
            return False

        # Auto-detect Root and Tip Links
        rootandtip = robotNode.FindRootAndTipLinks()
        if not rootandtip or len(rootandtip) < 2:
            print("Error: Could not auto-detect root and tip links from URDF.")
            return False

        # Load in goal robot if it does not already exist
        if robotNode.GetNumberOfNodeReferences("goal_model") == 0:
            if not robotNode.CreateGoalStateRobot(robotNode):
                print("Error: Failed to create goal state robot.")
                return False
            print("Goal robot created successfully.")

        # Get joint names
        self.joint_names = robotNode.GetJoints()

        # Create or update MotionControl node
        motionControlNode = slicer.mrmlScene.GetNodeByID(parameterNode.motionControlNodeID)
        if not motionControlNode:
            motionControlNode = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLROS2MotionControlNode")
            parameterNode.motionControlNodeID = motionControlNode.GetID()

        ros2NodeRef = robotNode.GetNodeReference("node")
        if ros2NodeRef is None:
            ros2NodeRef = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLROS2NodeNode")
        if ros2NodeRef is not None:
            motionControlNode.SetROS2NodeID(ros2NodeRef.GetID())
        motionControlNode.SetRobotNodeID(robotNode.GetID())

        topic_name = parameterNode.jointStateTopic if parameterNode.jointStateTopic else "joint_states"
        if not self.ConfigureJointStateSubscriber(robotNode, topic_name):
            print("Warning: Failed to configure JointState subscriber.")

        if parameterNode.moveGroupExists and parameterNode.planningGroup:
            self.SetupMoveItPlanningGroup(robotNode, parameterNode.planningGroup)

        return True

    def _onJointStateModified(self, caller=None, event=None):
        if self.joint_state_subscriber is None:
            return
        try:
            self.joint_state_last_message = self.joint_state_subscriber.GetLastMessage()
        except Exception as e:
            print(f"Joint state: failed to cache last message: {e}")

    def ConfigureJointStateSubscriber(self, robotNode, topic_name: str) -> bool:
        if robotNode is None:
            print("Joint state: robot node is invalid")
            return False

        ros2_node = robotNode.GetNodeReference("node")
        if ros2_node is None:
            print("Joint state: robot has no ROS2 node reference")
            return False

        topic = self._NormalizeJointStateTopic(topic_name)
        if (self.joint_state_subscriber is not None and
                self.joint_state_topic == topic and
                self.joint_state_ros2_node == ros2_node):
            return True

        self.ClearJointStateSubscriber()

        subscriber = ros2_node.GetSubscriberNodeByTopic(topic)
        self.joint_state_subscriber_owned = False
        if subscriber is None:
            for class_name in ("JointState", "vtkMRMLROS2SubscriberJointStateNode"):
                subscriber = ros2_node.CreateAndAddSubscriberNode(class_name, topic)
                if subscriber:
                    self.joint_state_subscriber_owned = True
                    break

        if subscriber is None:
            print(f"Joint state: failed to create subscriber for topic '{topic}'")
            print(f"Joint state: registered subscriber types: {ros2_node.RegisteredROS2SubscriberNodes()}")
            return False

        self.joint_state_subscriber = subscriber
        self.joint_state_topic = topic
        self.joint_state_ros2_node = ros2_node
        self.joint_state_last_message = None
        self.joint_state_subscriber_observer = self.joint_state_subscriber.AddObserver("ModifiedEvent", self._onJointStateModified)
        self._onJointStateModified()

        print(f"Joint state: subscribed to topic '{topic}'")
        return True

    def ClearJointStateSubscriber(self):
        if self.joint_state_subscriber and self.joint_state_subscriber_observer is not None:
            try:
                self.joint_state_subscriber.RemoveObserver(self.joint_state_subscriber_observer)
            except Exception:
                pass

        if self.joint_state_ros2_node and self.joint_state_topic and self.joint_state_subscriber_owned:
            try:
                self.joint_state_ros2_node.RemoveAndDeleteSubscriberNode(self.joint_state_topic)
            except Exception:
                pass

        self.joint_state_subscriber = None
        self.joint_state_subscriber_observer = None
        self.joint_state_last_message = None
        self.joint_state_topic = None
        self.joint_state_ros2_node = None
        self.joint_state_subscriber_owned = False

    def _extract_sequence(self, message, field_name: str):
        if message is None:
            return []

        accessors = [
            f"Get{field_name}",
            f"Get{field_name.capitalize()}",
        ]
        data = None
        for accessor in accessors:
            getter = getattr(message, accessor, None)
            if getter is None:
                continue
            try:
                data = getter()
                break
            except Exception:
                continue

        if data is None:
            return []

        if isinstance(data, (list, tuple)):
            return list(data)

        size_getter = getattr(data, "GetNumberOfValues", None)
        value_getter = getattr(data, "GetValue", None)
        if size_getter and value_getter:
            try:
                return [value_getter(i) for i in range(size_getter())]
            except Exception:
                return []

        tuple_count_getter = getattr(data, "GetNumberOfTuples", None)
        tuple_value_getter = getattr(data, "GetTuple1", None)
        if tuple_count_getter and tuple_value_getter:
            try:
                return [tuple_value_getter(i) for i in range(tuple_count_getter())]
            except Exception:
                return []

        try:
            return list(data)
        except Exception:
            return []

    def GetCurrentJointState(self, joint_names):
        if self.joint_state_subscriber is None:
            print("Current state: JointState subscriber is not configured")
            return []

        message = self.joint_state_last_message
        if message is None:
            try:
                message = self.joint_state_subscriber.GetLastMessage()
            except Exception as e:
                print(f"Current state: unable to retrieve JointState message ({e})")
                return []

        names_raw = self._extract_sequence(message, "name")
        positions_raw = self._extract_sequence(message, "position")
        if not names_raw or not positions_raw:
            print("Current state: JointState message missing name/position arrays")
            return []

        names = [str(n) for n in names_raw]
        try:
            positions = [float(p) for p in positions_raw]
        except Exception:
            print("Current state: JointState position array contains invalid values")
            return []

        if len(names) != len(positions):
            print("Current state: JointState name/position length mismatch")
            return []

        by_name = {n: p for n, p in zip(names, positions)}
        missing = [n for n in joint_names if n not in by_name]
        if missing:
            print(f"Current state: JointState missing joints: {missing}")
            return []

        return [by_name[n] for n in joint_names]

    def getParameterNode(self):
        return ROS2MotionControlParameterNode(super().getParameterNode())

    def createSphereModel(self, name="ProbeSphere", radius_mm=20.0):
        r = radius_mm
        src = vtk.vtkSphereSource()
        src.SetRadius(r); src.SetThetaResolution(40); src.SetPhiResolution(40); src.Update()

        model = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", name)
        model.SetAndObservePolyData(src.GetOutput())

        disp = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelDisplayNode", name+"_Display")
        disp.SetOpacity(0.6); disp.SetBackfaceCulling(0); disp.SetVisibility3D(True); disp.SetVisibility2D(True)
        disp.SetColor(0.9,0.3,0.3)
        model.SetAndObserveDisplayNodeID(disp.GetID())
        return model

    def createLinearTransform(self, name="ProbeSphere_Transform", showAxes=True):
        t = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", name)
        if not t.GetDisplayNode():
            tdisp = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformDisplayNode", name+"_Display")
            tdisp.SetVisibility(True)  # “eye” in Data
            tdisp.SetVisibility2D(True)
            t.SetAndObserveDisplayNodeID(tdisp.GetID())
        if showAxes:
            t.GetDisplayNode().SetEditorVisibility(True)  # show 3D gizmo on selection
        return t

    def applyTransformToModel(self, modelNode, transformNode):

        if modelNode is None or transformNode is None:
            raise ValueError("modelNode and transformNode are required")

        # Link the model to the transform in the MRML hierarchy
        modelNode.SetAndObserveTransformNodeID(transformNode.GetID())

        # Nudge MRML/3D view to update
        modelNode.Modified()

    def EnterControlMode(self, robotmodel, rootlink, tiplink, goaltiplink, baseGoalColor=None):
        if not robotmodel or not rootlink:
            return None

        try:
            totransform = self.findRobotTransforms(rootlink, goal=True)
        except RuntimeError:
            print("Error: Could not find goal robot root transform.")
            return None

        try:
            model = slicer.util.getNode("ProbeSphere")
        except Exception:
            model = None

        if model is None:
            model = self.createSphereModel()
            fromtransform = self.createLinearTransform()
            if fromtransform.GetDisplayNode():
                fromtransform.GetDisplayNode().SetEditorVisibility(True)
            self.applyTransformToModel(model, fromtransform)
        else:
            try:
                fromtransform = slicer.util.getNode("ProbeSphere_Transform")
            except Exception:
                fromtransform = self.createLinearTransform()
                self.applyTransformToModel(model, fromtransform)
            if fromtransform.GetDisplayNode():
                fromtransform.GetDisplayNode().SetEditorVisibility(True)

        try:
            if goaltiplink:
                tip_transform_node = self.findRobotTransforms(goaltiplink, goal=True)
                if tip_transform_node:
                    tip_matrix = vtk.vtkMatrix4x4()
                    tip_transform_node.GetMatrixTransformToWorld(tip_matrix)
                    fromtransform.SetMatrixTransformToParent(tip_matrix)
        except Exception as e:
            print(f"Warning: Could not snap sphere to goal tip. Error: {e}")

        self.setIKSourceTransforms(fromtransform.GetName(), totransform.GetName())

        if DEBUG:
            print(f"\n=== TIP LINK CONFIGURATION ===")
            print(f"rootlink (base): {rootlink}")
            print(f"tiplink (target): {tiplink}")
            print(f"goaltiplink: {goaltiplink}")
            print(f"================================\n")

        self.tipLink = tiplink
        self.addObserverComputeIK(robotmodel, baseGoalColor=baseGoalColor)
        return {"fromTransform": fromtransform, "toTransform": totransform}

    def ExitControlMode(self, fromtransform=None):
        self.removeObserver()

        if fromtransform:
            slicer.mrmlScene.RemoveNode(fromtransform)

        try:
            model = slicer.util.getNode("ProbeSphere")
            if model:
                slicer.mrmlScene.RemoveNode(model)
        except Exception:
            pass

    def _allTransforms(self):
        s = slicer.mrmlScene
        return [s.GetNthNodeByClass(i, "vtkMRMLTransformNode")
                for i in range(s.GetNumberOfNodesByClass("vtkMRMLTransformNode"))]

    def attachProbeTransformUnderLeaf(self, probeTransformName="ProbeSphere_Transform",
                                    prefix="ros2:tf2lookup:"):
        """
        Find robot leaf transform and parent `probeTransformName` under it.
        Creates a display node for the probe transform if needed.
        """
        # 1) find (or get) the probe transform node
        probeT = slicer.util.getNode(probeTransformName)
        if probeT is None:
            # create if missing
            probeT = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", probeTransformName)
        if not probeT.GetDisplayNode():
            d = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformDisplayNode", probeTransformName + "_Display")
            d.SetVisibility(True)
            probeT.SetAndObserveDisplayNodeID(d.GetID())

        # 2) find robot leaf
        leaf = self.findLeafRobotTransform(prefix=prefix)

        # 3) parent probe under the leaf
        probeT.SetAndObserveTransformNodeID(leaf.GetID())

        print(f"Attached '{probeTransformName}' under leaf transform '{leaf.GetName()}'")
        return dict(leafTransform=leaf, probeTransform=probeT)

    def findRobotTransforms(self, link_name, goal=False):
        """
        Locate the transform for a link/model name using multi-part model naming only:
        - live: <link>_model_<index>
        - goal: <link>_model_<index>_goal
        """
        scene = slicer.mrmlScene

        # If an explicit model node name is provided, honor it directly.
        if re.fullmatch(r".+_model_\d+(_goal)?", link_name):
            model_name = link_name if (not goal or link_name.endswith("_goal")) else f"{link_name}_goal"
            try:
                model_node = slicer.util.getNode(model_name)
                parent = model_node.GetParentTransformNode()
                if parent:
                    return parent
            except Exception:
                pass

        # Otherwise treat input as a link name and pick the lowest-index model part.
        prefix = f"{link_name}_model_"
        candidates = []
        for i in range(scene.GetNumberOfNodesByClass("vtkMRMLModelNode")):
            model = scene.GetNthNodeByClass(i, "vtkMRMLModelNode")
            if not model:
                continue
            model_name = model.GetName() or ""
            if goal:
                if model_name.startswith(prefix) and model_name.endswith("_goal"):
                    index_text = model_name[len(prefix):-5]
                    if re.fullmatch(r"\d+", index_text):
                        candidates.append((int(index_text), model))
            else:
                if model_name.startswith(prefix):
                    index_text = model_name[len(prefix):]
                    if re.fullmatch(r"\d+", index_text):
                        candidates.append((int(index_text), model))

        if candidates:
            candidates.sort(key=lambda x: x[0])
            parent = candidates[0][1].GetParentTransformNode()
            if parent:
                return parent

        if goal:
            try:
                goal_transform = slicer.util.getNode(f"{link_name}_goal_transform")
                if goal_transform:
                    return goal_transform
            except Exception:
                pass

        # Fallback: search for TF lookup nodes (for links without meshes like base_link)
        if not goal:
            scene = slicer.mrmlScene
            for i in range(scene.GetNumberOfNodesByClass("vtkMRMLROS2Tf2LookupNode")):
                lookup = scene.GetNthNodeByClass(i, "vtkMRMLROS2Tf2LookupNode")
                if lookup:
                    child_id = lookup.GetChildID() or ""
                    # Match either exact link name or suffix after "/" (handles prefixed names)
                    if child_id == link_name or child_id.endswith(f"/{link_name}"):
                        return lookup

        raise RuntimeError(f"Could not find transform for link/model '{link_name}' (goal={goal})")

    def addObserver(self, fromTransformName, toTransformName, robotmodel=None):
        """
        Observe 'fromTransformName' and print its XYZ (origin) w.r.t. 'toTransformName'
        whenever the FROM transform is modified.
        """
        fromNode = slicer.util.getNode(fromTransformName)
        toNode   = slicer.util.getNode(toTransformName)

        if fromNode is None or toNode is None:
            raise RuntimeError("Transform nodes not found in scene.")

        # Print once initially
        self.printLocation(fromNode, toNode)

        # Remove a previous observer if any
        self.removeObserver()

        # Define the callback and keep a reference to it
        def onModified(caller, eventId):
            if self.isInteracting:
                return # Skip if interaction event already handling this
            self.printLocation(fromNode, toNode)
            if self.obsRobotNode:
                self.computeIK(self.obsRobotNode)

        self.callback = onModified
        # Prefer TransformModifiedEvent for transforms; ModifiedEvent also works
        eventId = slicer.vtkMRMLTransformNode.TransformModifiedEvent
        self.obsTag  = fromNode.AddObserver(eventId, self.callback)
        self.obsNode = fromNode
        self.toNode = toNode
        self.obsRobotNode = robotmodel
        return self.obsTag

    def printLocation(self, fromNode, toNode):
        m = vtk.vtkMatrix4x4()
        ok = slicer.vtkMRMLTransformNode.GetMatrixTransformBetweenNodes(fromNode, toNode, m)
        if not ok:
            print("Could not compute transform between nodes.")
            return
        x, y, z = m.GetElement(0, 3), m.GetElement(1, 3), m.GetElement(2, 3)
        if DEBUG:
            print(f"{fromNode.GetName()} wrt {toNode.GetName()}: x={x:.2f}, y={y:.2f}, z={z:.2f} mm")

    def removeObserver(self):
        if self.viewObserverTags:
            for observed_object, observed_tag in self.viewObserverTags:
                try:
                    observed_object.RemoveObserver(observed_tag)
                except Exception:
                    pass
            self.viewObserverTags = []

        if self.obsNode and self.obsTag is not None:
            try:
                self.obsNode.RemoveObserver(self.obsTag)
            except Exception as e:
                print(f"[ROS2MotionControlLogic] Error removing observer: {e}")
        else:
            if self.obsTag is not None:
                print(f"[ROS2MotionControlLogic] No obsNode to remove observer from (tag={self.obsTag})")
        self.obsNode = None
        self.obsTag = None
        self.callback = None
        self.isInteracting = False

    def computeIK(self, robotmodel, baseGoalColor=None):
        if robotmodel is None:
            return
        solution = None
        if self.useMoveItIK:
            solution = self.computeIKWithMoveIt(robotmodel=robotmodel, tipLink=self.tipLink)
            if DEBUG and solution:
                print(f"[MoveIt IK] Solution: {solution}")
        else:
            solution = self.computeIKWithKDL(robotmodel=robotmodel)

        # Resolve base color: prefer caller-supplied value, fall back to reading from the robot
        if baseGoalColor is None:
            baseGoalColor = self.GetGoalRobotColor(robotmodel)

        self._updateRobotColorForIKResult(robotmodel, solution is not None, baseGoalColor)

    def _updateRobotColorForIKResult(self, robotNode, success, baseGoalColor=None):
        """Changes goal robot color to complementary if IK fails, restores it on success."""
        if baseGoalColor is None:
            baseGoalColor = self.GetGoalRobotColor(robotNode)
        if baseGoalColor is None:
            return
        if success:
            self.setGoalRobotColorRGB(robotNode, baseGoalColor)
        else:
            r, g, b = baseGoalColor
            self.setGoalRobotColorRGB(robotNode, (1.0 - r, 1.0 - g, 1.0 - b))

    def setGoalRobotColorRGB(self, robotNode, rgb):
        if robotNode is None:
            return
        r, g, b = rgb
        goal_models = self._collectGoalModelNodes(robotNode)
        for modelNode in goal_models:
            displayNode = modelNode.GetDisplayNode()
            if displayNode:
                displayNode.SetColor(r, g, b)

    def _addViewInteractionObservers(self, robotmodel, baseGoalColor=None):
        layout_manager = slicer.app.layoutManager()
        if layout_manager is None:
            return

        def onInteractionEvent(caller, eventId):
            if eventId == vtk.vtkCommand.InteractionEvent:
                self.isInteracting = True
                self.computeIK(robotmodel, baseGoalColor=baseGoalColor)
            elif eventId == vtk.vtkCommand.EndInteractionEvent:
                self.isInteracting = False
                self.computeIK(robotmodel, baseGoalColor=baseGoalColor)

        interaction_events = (vtk.vtkCommand.InteractionEvent, vtk.vtkCommand.EndInteractionEvent)

        # Observe 3D view interactions.
        try:
            for view_index in range(layout_manager.threeDViewCount):
                three_d_widget = layout_manager.threeDWidget(view_index)
                if three_d_widget is None:
                    continue
                interactor = three_d_widget.threeDView().interactor()
                if interactor is None:
                    continue
                style = interactor.GetInteractorStyle()
                if style is None:
                    continue
                for interaction_event in interaction_events:
                    tag = style.AddObserver(interaction_event, onInteractionEvent)
                    self.viewObserverTags.append((style, tag))
        except Exception:
            pass

        # Observe slice view interactions.
        try:
            for slice_name in layout_manager.sliceViewNames():
                slice_widget = layout_manager.sliceWidget(slice_name)
                if slice_widget is None:
                    continue
                interactor = slice_widget.sliceView().interactor()
                if interactor is None:
                    continue
                style = interactor.GetInteractorStyle()
                if style is None:
                    continue
                for interaction_event in interaction_events:
                    tag = style.AddObserver(interaction_event, onInteractionEvent)
                    self.viewObserverTags.append((style, tag))
        except Exception:
            pass


    def computeIKWithMoveIt(self, robotmodel, tipLink):

    # --- Get Slicer transform nodes ---
        fromNode = self.obsNode
        toNode   = self.toNode

        # --- Compute 4×4 transform between nodes ---
        targetPose = vtk.vtkMatrix4x4()
        if not slicer.vtkMRMLTransformNode.GetMatrixTransformBetweenNodes(fromNode, toNode, targetPose):
            raise RuntimeError("Could not compute transform between nodes.")

        if DEBUG:
            print(f"\n[MoveIt IK] Target Pose Matrix (4x4):")
            print(f"  [{targetPose.GetElement(0,0):.3f}, {targetPose.GetElement(0,1):.3f}, {targetPose.GetElement(0,2):.3f}, {targetPose.GetElement(0,3):.2f}]")
            print(f"  [{targetPose.GetElement(1,0):.3f}, {targetPose.GetElement(1,1):.3f}, {targetPose.GetElement(1,2):.3f}, {targetPose.GetElement(1,3):.2f}]")
            print(f"  [{targetPose.GetElement(2,0):.3f}, {targetPose.GetElement(2,1):.3f}, {targetPose.GetElement(2,2):.3f}, {targetPose.GetElement(2,3):.2f}]")
            print(f"  [{targetPose.GetElement(3,0):.3f}, {targetPose.GetElement(3,1):.3f}, {targetPose.GetElement(3,2):.3f}, {targetPose.GetElement(3,3):.3f}]")

        now = time.time()
        if now - self._last_moveit_obstacle_publish_time > 1.0:
            self.PublishAllMoveItObstacles(robotmodel)
            self._last_moveit_obstacle_publish_time = now

        seed = self.last_ik_solution
        result_str = robotmodel.FindIKMoveIt(targetPose, tipLink, seed, 0.05)

        if result_str and result_str.strip():
            # Parse comma-separated string into list of floats
            try:
                data = [float(x) for x in result_str.split(",")]
                if DEBUG:
                    print(f"[IK] Joint Solution: {data}")
                self.last_ik_solution = data
                # Publish the joint state solution
                self.updategoalTransformsFromJointsKDL(robotmodel, data)
                return data

            except ValueError as e:
                print(f"[IK] Failed to parse solution: {e}")
                return None
        else:
            if DEBUG:
                print(f"[IK] Empty result from FindIK")
            return None


    def computeIKWithKDL(self, robotmodel):
            # --- Get Slicer transform nodes ---
            fromNode = self.obsNode
            toNode   = self.toNode

            if fromNode is None or toNode is None:
                return None

            if DEBUG:
                print(f"\n[IK] Computing transform from '{fromNode.GetName()}' to '{toNode.GetName()}'")

            # --- Compute 4×4 transform between nodes ---
            targetPose = vtk.vtkMatrix4x4()
            success = slicer.vtkMRMLTransformNode.GetMatrixTransformBetweenNodes(fromNode, toNode, targetPose)

            if not success:
                raise RuntimeError("Could not compute transform between nodes.")

            # If we have no seed or a bad seed, try all zeros first
            seed = self.last_ik_solution if self.last_ik_solution and len(self.last_ik_solution) > 0 else []

            # call KDL IK
            result_str = robotmodel.FindKDLIK(targetPose, seed)

            if result_str and result_str.strip():
                try:
                    data = [float(x) for x in result_str.split(",")]
                    if DEBUG:
                        print(f"[IK] Solution found: {data}")
                    self.last_ik_solution = data
                    self.updategoalTransformsFromJointsKDL(robotmodel, data)
                    return data
                except ValueError as e:
                    print(f"[IK] Failed to parse solution: {e}")
                    return None
            else:
                if DEBUG:
                    print(f"[IK] Empty result from FindKDLIK")
                return None

    def addObserverComputeIK(self, robotmodel=None, baseGoalColor=None):
            """
            Observe transform changes. Uses self.obsNode and self.toNode that should be
            set by setupikforRobot(). Each transform update triggers IK computation.
            Uses either KDL (default) or MoveIt IK based on useMoveItIK flag.

            Args:
                robotmodel: The robot model for IK
                baseGoalColor: (r, g, b) tuple used to color-code IK success/failure.
                               Comes from the module parameter node via the widget.
            """
            fromNode = self.obsNode
            toNode   = self.toNode

            if fromNode is None or toNode is None:
                raise RuntimeError("Transform nodes not found. Call setupikforRobot() first.")

            # Remove previous observer if any to prevent duplicates
            # (but this will clear self.obsNode, so we restore it below)
            self.removeObserver()

            # Restore the node references that removeObserver() cleared
            self.obsNode = fromNode
            self.toNode = toNode

            def onModified(caller, eventId):
                # Update XYZ label/print even during interaction
                self.printLocation(fromNode, toNode)

                # Only compute IK if NOT interacting via view.
                # If interacting, the InteractorStyle observers handle _runIk.
                # If NOT interacting (e.g. typing values), handle it here.
                if self.isInteracting:
                    return
                self.computeIK(robotmodel, baseGoalColor=baseGoalColor)

            self.callback = onModified
            eventId = slicer.vtkMRMLTransformNode.TransformModifiedEvent
            self.obsTag  = fromNode.AddObserver(eventId, self.callback)
            self._addViewInteractionObservers(robotmodel, baseGoalColor=baseGoalColor)

            return self.obsTag

    # Set robot opacity
    def setOpacity(self, robotmodel, opacity):

        # Get number of model nodes under robotmodel
        numModels = robotmodel.GetNumberOfNodeReferences("model")

        # Loop through each model node and set opacity
        for i in range(numModels):
            modelNode = robotmodel.GetNthNodeReference("model", i)
            displayNode = modelNode.GetDisplayNode()
            if displayNode:
                displayNode.SetOpacity(opacity)

    # Set robot color
    def setGoalRobotColor(self, robotNode, color):
        if robotNode is None:
            return

        r = color.red() / 255.0
        g = color.green() / 255.0
        b = color.blue() / 255.0

        goal_models = self._collectGoalModelNodes(robotNode)
        for modelNode in goal_models:
            displayNode = modelNode.GetDisplayNode()
            if displayNode:
                displayNode.SetColor(r, g, b)

    def GetGoalRobotColor(self, robotNode):
        if robotNode is None:
            return None

        goal_models = self._collectGoalModelNodes(robotNode)
        for modelNode in goal_models:
            displayNode = modelNode.GetDisplayNode()
            if displayNode:
                color = displayNode.GetColor()
                if color and len(color) >= 3:
                    return (float(color[0]), float(color[1]), float(color[2]))
        return None

    def _collectGoalModelNodes(self, robotNode):
        goal_nodes = []
        seen_names = set()

        # First pass: gather all goal model nodes directly referenced by robot node.
        numModels = robotNode.GetNumberOfNodeReferences("model")
        for i in range(numModels):
            modelNode = robotNode.GetNthNodeReference("model", i)
            if modelNode is None:
                continue
            modelName = modelNode.GetName() or ""
            if modelName.endswith("_goal") and modelName not in seen_names:
                goal_nodes.append(modelNode)
                seen_names.add(modelName)

        # Second pass: map each live model reference to its goal counterpart name.
        for i in range(numModels):
            modelNode = robotNode.GetNthNodeReference("model", i)
            if modelNode is None:
                continue
            modelName = modelNode.GetName() or ""
            if modelName.endswith("_goal"):
                continue
            goalName = f"{modelName}_goal"
            if goalName in seen_names:
                continue
            try:
                goalModel = slicer.util.getNode(goalName)
                if goalModel is not None:
                    goal_nodes.append(goalModel)
                    seen_names.add(goalName)
            except Exception:
                pass

        # Fallback: if still empty, scan scene for all goal models.
        if not goal_nodes:
            scene = slicer.mrmlScene
            count = scene.GetNumberOfNodesByClass("vtkMRMLModelNode")
            for i in range(count):
                modelNode = scene.GetNthNodeByClass(i, "vtkMRMLModelNode")
                if modelNode is None:
                    continue
                modelName = modelNode.GetName() or ""
                if modelName.endswith("_goal") and modelName not in seen_names:
                    goal_nodes.append(modelNode)
                    seen_names.add(modelName)

        return goal_nodes

    def setJointSlidersFromUrdfLimits(self, limits_rad, sliders):

        if len(sliders) != len(limits_rad):
            print(
                f"[ROS2MotionControl] Slider count ({len(sliders)}) "
                f"!= joint count ({len(limits_rad)})"
            )

        for slider, (jointName, limit_info) in zip(sliders, limits_rad.items()):
            lo_rad = limit_info[0]
            hi_rad = limit_info[1]
            jtype = limit_info[2] if len(limit_info) > 2 else "revolute"

            if jtype == "prismatic":
                lo_ui = int(round(lo_rad * 1000.0))
                hi_ui = int(round(hi_rad * 1000.0))
                unit_str = "mm"
            else:
                lo_ui = int(round(math.degrees(lo_rad)))
                hi_ui = int(round(math.degrees(hi_rad)))
                unit_str = "deg"

            if lo_ui > hi_ui:
                lo_ui, hi_ui = hi_ui, lo_ui

            slider.setRange(lo_ui, hi_ui)
            slider.setValue(0)

            print(f"[ROS2MotionControl] {jointName}: {lo_ui}..{hi_ui} {unit_str}")

    def setIKSourceTransforms(self, fromtransformname, totransformname):
            """
            Links the specific visual nodes (Sphere Transform -> Robot Root Transform)
            so the observer knows what to calculate IK for.
            """
            fromNode = slicer.util.getNode(fromtransformname)
            toNode   = slicer.util.getNode(totransformname)

            if fromNode and toNode:
                self.obsNode = fromNode
                self.toNode = toNode
                if DEBUG:
                    print(f"Logic Linked: '{fromtransformname}' -> '{totransformname}'")
            else:
                print("Error: Could not link IK transforms (Nodes missing)")

    def updategoalTransformsFromJointsKDL(self, robotmodel, joint_values):
        """
        Update all goal robot link transforms using KDL FK computation.
        For each link, calls ComputeKDLFK to get the transform and applies it to the goal link.

        Args:
            robotmodel: The robot model node with ComputeLocalTransform method
            joint_values: List of joint angles in radians
        """
        if not robotmodel or not joint_values:
            if DEBUG:
                print("[updategoalTransformsFromJointsKDL] No robot model or joint values")
            return False

        seg = robotmodel.GetSegments()

        # For each link, compute FK and update goal transform
        for link_name in seg:
            try:
                # Create a matrix to hold the FK result
                fk_matrix = vtk.vtkMatrix4x4()

                # Call the C++ ComputeLocalTransform function
                # It takes: joint_values (as list), output matrix, and link name
                result = robotmodel.ComputeLocalTransform(joint_values, fk_matrix, link_name)

                if result is None:
                    print(f"[FK] Failed to compute FK for link '{link_name}'")
                    continue

                # Find the goal link's transform node
                try:
                    goal_transform = self.findRobotTransforms(link_name, goal=True)
                    if goal_transform:
                        # Apply the FK matrix to the goal transform
                        goal_transform.SetMatrixTransformToParent(fk_matrix)
                        if DEBUG:
                            print(f"[FK] Updated goal transform for '{link_name}'")
                except Exception:
                    print(f"[FK] Could not find or update goal transform for '{link_name}'")

            except Exception as e:
                print(f"[FK] Error computing FK for link '{link_name}': {e}")

        return True
