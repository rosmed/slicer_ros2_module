import logging
import os, time
import subprocess
from typing import Annotated, Optional
import xml.etree.ElementTree as ET
import math
import qt
import json
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
    WithinRange,
)





from slicer import vtkMRMLScalarVolumeNode


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
        self.parent.title = _("ROS2 Motion Control")
        self.parent.categories = [translate("qSlicerAbstractCoreModule", "IGT")]
        self.parent.dependencies = ["ROS2"]
        self.parent.contributors = ["Kaito Hara-Lee", "Anton Deguet"]
        self.parent.helpText = _("""
    Scripted module for robot control and motion planning workflows integrated with SlicerROS2.
""")
        self.parent.acknowledgementText = _("""
    Integrated from the MedicalRobotMotionPlanner project and adapted for this repository.
""")

        # Additional initialization step after application startup is complete
        slicer.app.connect("startupCompleted()", registerSampleData)


#
# Register sample data sets in Sample Data module
#


def registerSampleData():
    """Add data sets to Sample Data module."""
    # It is always recommended to provide sample data for users to make it easy to try the module,
    # but if no sample data is available then this method (and associated startupCompeted signal connection) can be removed.

    import SampleData

    iconsPath = os.path.join(os.path.dirname(__file__), "Resources/Icons")

    # To ensure that the source code repository remains small (can be downloaded and installed quickly)
    # it is recommended to store data sets that are larger than a few MB in a Github release.

    # ROS2MotionControl1
    SampleData.SampleDataLogic.registerCustomSampleDataSource(
        # Category and sample name displayed in Sample Data module
        category="ROS2MotionControl",
        sampleName="ROS2MotionControl1",
        # Thumbnail should have size of approximately 260x280 pixels and stored in Resources/Icons folder.
        # It can be created by Screen Capture module, "Capture all views" option enabled, "Number of images" set to "Single".
        thumbnailFileName=os.path.join(iconsPath, "ROS2MotionControl1.png"),
        # Download URL and target file name
        uris="https://github.com/Slicer/SlicerTestingData/releases/download/SHA256/998cb522173839c78657f4bc0ea907cea09fd04e44601f17c82ea27927937b95",
        fileNames="ROS2MotionControl1.nrrd",
        # Checksum to ensure file integrity. Can be computed by this command:
        #  import hashlib; print(hashlib.sha256(open(filename, "rb").read()).hexdigest())
        checksums="SHA256:998cb522173839c78657f4bc0ea907cea09fd04e44601f17c82ea27927937b95",
        # This node name will be used when the data set is loaded
        nodeNames="ROS2MotionControl1",
    )

    # ROS2MotionControl2
    SampleData.SampleDataLogic.registerCustomSampleDataSource(
        # Category and sample name displayed in Sample Data module
        category="ROS2MotionControl",
        sampleName="ROS2MotionControl2",
        thumbnailFileName=os.path.join(iconsPath, "ROS2MotionControl2.png"),
        # Download URL and target file name
        uris="https://github.com/Slicer/SlicerTestingData/releases/download/SHA256/1a64f3f422eb3d1c9b093d1a18da354b13bcf307907c66317e2463ee530b7a97",
        fileNames="ROS2MotionControl2.nrrd",
        checksums="SHA256:1a64f3f422eb3d1c9b093d1a18da354b13bcf307907c66317e2463ee530b7a97",
        # This node name will be used when the data set is loaded
        nodeNames="ROS2MotionControl2",
    )


#
# ROS2MotionControlParameterNode
#


@parameterNodeWrapper
class ROS2MotionControlParameterNode:
    """
    The parameters needed by module.

    inputVolume - The volume to threshold.
    imageThreshold - The value at which to threshold the input volume.
    invertThreshold - If true, will invert the threshold.
    thresholdedVolume - The output volume that will contain the thresholded volume.
    invertedVolume - The output volume that will contain the inverted thresholded volume.
    """

    inputVolume: vtkMRMLScalarVolumeNode
    imageThreshold: Annotated[float, WithinRange(-100, 500)] = 100
    invertThreshold: bool = False
    thresholdedVolume: vtkMRMLScalarVolumeNode
    invertedVolume: vtkMRMLScalarVolumeNode


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
        self.ghosttiplink = None
        self.isRobotLoaded = False
        self.trajectoryTimer = None
        self.trajectoryData = None
        self.trajectoryIndex = 0
        self.trajectorySlider = None

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

        # Create logic class. Logic implements all computations that should be possible to run
        # in batch mode, without a graphical user interface.
        self.logic = ROS2MotionControlLogic()

        # Connections

        # These connections ensure that we update parameter node when scene is closed
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.StartCloseEvent, self.onSceneStartClose)
        self.addObserver(slicer.mrmlScene, slicer.mrmlScene.EndCloseEvent, self.onSceneEndClose)
        self.ui.tabWidget.currentChanged.connect(self.onTabChanged)
        
        # Buttons
        self.ui.applyButton.connect("clicked(bool)", self.onApplyButton)
        self.ui.usebutton.connect("clicked(bool)", self.onusebutton)
        self.ui.opacitypushButton.connect("clicked(bool)", self.onopacitybutton)
        self.ui.robotColorButton.connect("colorChanged(QColor)", self.onRobotColorChanged)
        self.ui.zeropushButton.connect("clicked(bool)", self.onzerobutton)
        self.ui.checkBox.connect("toggled(bool)", self.onMoveGroupToggled)
        self.ui.planbutton.connect("clicked(bool)", self.onPlanButton)
        self.ui.previewbutton.connect("clicked(bool)", self.onpreviewButton)
        self.ui.executebutton.connect("clicked(bool)", self.onExecuteButton)    
                
        # Set appearence collapsible button to be collapsed and disabled initially
        self.ui.appCollapsibleButton.collapsed = True
        self.ui.appCollapsibleButton.enabled = False
        self.ui.checkBox.enabled = False
        self.ui.planbutton.enabled = False
        self.ui.previewbutton.enabled = False
        self.ui.executebutton.enabled = False
        self.ui.plangroup.enabled = False
        self.ui.plangroupline.enabled = False
        
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
        self.removeObservers()
        
        # Remove probe sphere and transform
        try:
            probe_transform = slicer.util.getNode("ProbeSphere_Transform")
            if probe_transform:
                slicer.mrmlScene.RemoveNode(probe_transform)
        except:
            pass
        
        try:
            probe_model = slicer.util.getNode("ProbeSphere")
            if probe_model:
                slicer.mrmlScene.RemoveNode(probe_model)
        except:
            pass

    def enter(self) -> None:
        """Called each time the user opens this module."""
        # Make sure parameter node exists and observed
        self.initializeParameterNode()

    def exit(self) -> None:
        """Called each time the user opens a different module."""
        # Stop streaming when exiting module
        if self.logic:
            self.logic.removeObserver()
        # Do not react to parameter node changes (GUI will be updated when the user enters into the module)
        if self._parameterNode:
            self._parameterNode.disconnectGui(self._parameterNodeGuiTag)
            self._parameterNodeGuiTag = None
            self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)

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
        # Parameter node stores all user choices in parameter values, node selections, etc.
        # so that when the scene is saved and reloaded, these settings are restored.

        self.setParameterNode(self.logic.getParameterNode())

        # Select default input nodes if nothing is selected yet to save a few clicks for the user
        if not self._parameterNode.inputVolume:
            firstVolumeNode = slicer.mrmlScene.GetFirstNodeByClass("vtkMRMLScalarVolumeNode")
            if firstVolumeNode:
                self._parameterNode.inputVolume = firstVolumeNode

    def setParameterNode(self, inputParameterNode: Optional[ROS2MotionControlParameterNode]) -> None:
        """
        Set and observe parameter node.
        Observation is needed because when the parameter node is changed then the GUI must be updated immediately.
        """

        if self._parameterNode:
            self._parameterNode.disconnectGui(self._parameterNodeGuiTag)
            self.removeObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)
        self._parameterNode = inputParameterNode
        if self._parameterNode:
            # Note: in the .ui file, a Qt dynamic property called "SlicerParameterName" is set on each
            # ui element that needs connection.
            self._parameterNodeGuiTag = self._parameterNode.connectGui(self.ui)
            self.addObserver(self._parameterNode, vtk.vtkCommand.ModifiedEvent, self._checkCanApply)
            self._checkCanApply()

    def _checkCanApply(self, caller=None, event=None) -> None:
        if self._parameterNode and self._parameterNode.inputVolume and self._parameterNode.thresholdedVolume:
            self.ui.applyButton.toolTip = _("Compute output volume")
            self.ui.applyButton.enabled = True
        else:
            self.ui.applyButton.toolTip = _("Select input and output volume nodes")
            self.ui.applyButton.enabled = False

    def onApplyButton(self) -> None:
        """Run processing when user clicks "Apply" button."""
        with slicer.util.tryWithErrorDisplay(_("Failed to compute results."), waitCursor=True):
            # Compute output
            self.logic.process(self.ui.inputSelector.currentNode(), self.ui.outputSelector.currentNode(),
                               self.ui.imageThresholdSliderWidget.value, self.ui.invertOutputCheckBox.checked)

            # Compute inverted output (if needed)
            if self.ui.invertedOutputSelector.currentNode():
                # If additional output volume is selected then result with inverted threshold is written there
                self.logic.process(self.ui.inputSelector.currentNode(), self.ui.invertedOutputSelector.currentNode(),
                                   self.ui.imageThresholdSliderWidget.value, not self.ui.invertOutputCheckBox.checked, showResult=False)
   
    def onusebutton(self) -> None:
            
            # Stop any prior streaming callbacks
            self.logic.removeObserver()
                
            # Get robot node
            robotNode = self.ui.ikrobotcombobox.currentNode()
            if not robotNode:
                print("Error: No robot selected.")
                return
            
            # Print selected robot name
            print("Selected Robot:", robotNode.GetName())

            # Extract URDF XML
            pnode = robotNode.GetNthNodeReference("parameter", 0)
            if not pnode:
                print("Error: No parameter node found for robot.")
                return
            urdf_xml = pnode.GetParameterAsString("robot_description")

            # Auto-detect Root and Tip Links
            alllink = self.logic.parse_all_link_names_from_urdf(urdf_xml)
            if not alllink: 
                print("Error: No links found in URDF.")
                return
            
            # Check if ghost model exists, if so store ghost tip
            ghost_name = alllink[-1] + "_model_ghost"
            try:
                ghost_model = slicer.util.getNode(ghost_name)
                ghost_loaded = ghost_model is not None and ghost_model.GetParentTransformNode() is not None
                if ghost_loaded:
                    self.ghosttiplink = ghost_name
                    print(f"Ghost robot loaded: True")
                else:
                    print(f"Ghost robot NOT loaded or Transform missing")
                    return
            except slicer.util.MRMLNodeNotFoundException:
                print(f"Ghost robot NOT loaded")
                return
        
            # Print current postiion
            currentjointpos =self.logic.getcurrentjointpositions(robotNode)

            # Get joint names & store initial joint positions
            joint_names = robotNode.GetJoints()
            self.logic.joint_names = joint_names
            self.logic.last_ik_solution = currentjointpos
            self.jointPositionsRad = [0.0] * len(joint_names)
            self.robot = robotNode
            self.rootlink = alllink[0] 
            self.tiplink = alllink[-1]
            
            # Print results
            print(f"CURRENT: rootlink={self.rootlink}, tiplink={self.tiplink}, ghosttiplink={self.ghosttiplink}")
            print(f"Current Joint Positions (rad): {[f'{j:.4f}' for j in currentjointpos]}")
            
            # Enable buttons
            self.ui.appCollapsibleButton.collapsed = False
            self.ui.appCollapsibleButton.enabled = True
            
            # Check if /move_group node exists and ghost robot in ROS
            # if so enable MoveIt buttons
            is_running = _check_ros2_node_running("/move_group")
            if is_running:
                self.ui.planbutton.enabled = True
                self.ui.previewbutton.enabled = True
                self.ui.checkBox.enabled = True
                self.ui.plangroup.enabled = True
                self.ui.plangroupline.enabled = True
                print(f"/move_group node is running")
                print(f"User must enter planning group name before using MoveIt IK")
            else:
                if not is_running:
                    print(f"/move_group node is NOT running")
            
            # Create Joint Sliders Dynamically (only if ghost model exists)
            self.ui.zeropushButton.enabled = True
            limits = self.logic.parse_joint_limits_from_urdf(urdf_xml)
            container = self.ui.Jointtab.layout()
            if container is not None:
                # FIX: Iterate backwards to delete dynamic items but KEEP the zero button
                for i in reversed(range(container.count())):
                    item = container.itemAt(i)
                    widget = item.widget()
                    
                    # If this is your specific button, skip it!
                    if widget == self.ui.zeropushButton:
                        continue
                        
                    # Otherwise, remove from layout and destroy
                    if widget is not None:
                        container.takeAt(i)
                        widget.deleteLater()

                # Create sliders dynamically
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
                        lo_deg = int(round(math.degrees(lo_hi[0])))
                        hi_deg = int(round(math.degrees(lo_hi[1])))
                        if lo_deg > hi_deg:
                            lo_deg, hi_deg = hi_deg, lo_deg
                    else:
                        lo_deg, hi_deg = -180, 180

                    # --- 4. CONFIGURE SLIDER ---
                    joint_slider.setMinimum(lo_deg)
                    joint_slider.setMaximum(hi_deg)
                    joint_slider.setValue(0)
                    joint_slider.setTickInterval(10)
                    joint_slider.setTickPosition(qt.QSlider.TicksBelow)

                    # --- 5. CONFIGURE SPINBOX ---
                    joint_spinbox.setMinimum(lo_deg)
                    joint_spinbox.setMaximum(hi_deg)
                    joint_spinbox.setSingleStep(1.0) 
                    joint_spinbox.setValue(0)
                    joint_spinbox.setSuffix(" deg")

                    # --- 6. SYNC LOGIC ---
                    # A. Slider moves -> Update Spinbox
                    joint_slider.valueChanged.connect(lambda val, sb=joint_spinbox: sb.setValue(val))

                    # B. Spinbox changes -> Update Slider
                    joint_spinbox.valueChanged.connect(lambda val, sl=joint_slider: sl.setValue(int(val)))

                    # C. Slider moves -> Trigger your IK Logic
                    joint_slider.valueChanged.connect(lambda value, idx=i: self.onJointSliderChanged(idx, value))

                    # --- 7. ADD TO LAYOUTS ---
                    
                    # Add Slider + Spinbox to the Horizontal controls layout
                    controls_layout.addWidget(joint_slider)
                    controls_layout.addWidget(joint_spinbox)
                    
                    # Add Label and the Controls Layout to the Main Vertical Block
                    joint_block_layout.addWidget(joint_label)
                    joint_block_layout.addLayout(controls_layout)
                    
                    # Add the whole block to your main container
                    container.addWidget(joint_block_widget)
            
            # Set robot true
            self.isRobotLoaded = True

    # Opacity button handler        
    def onopacitybutton(self) -> None:
        opacity = self.ui.spinBox.value / 100.0
        robot = self.ui.robotcomboBox.currentNode()
        self.logic.setopacity(robot, opacity)
    
    # Robot color button handler    
    def onRobotColorChanged(self) -> None:
        color = self.ui.robotColorButton.color    
        robotNode = self.ui.robotcomboBox.currentNode()
        self.logic.setRobotColor(robotNode, color)
        
    # Joint slider change handler
    def onJointSliderChanged(self, idx: int, valueDeg: int) -> None:
        # Ensure array is large enough
        while len(self.jointPositionsRad) <= idx:
            self.jointPositionsRad.append(0.0)
        
        # store as radians
        self.jointPositionsRad[idx] = math.radians(valueDeg)

        # Update ghost robot transforms with new joint positions
        if self.logic is not None and self.robot is not None:
            self.logic.updateGhostTransformsFromJointsKDL(self.robot, self.jointPositionsRad)
        
        print(f"All joint values (rad): {[f'{j:.4f}' for j in self.jointPositionsRad]}")
    
    # Zero button handler
    def onzerobutton(self) -> None:
        
        print("Resetting joint sliders to zero.")

        container = self.ui.Jointtab.layout()
        if container is None:
            return

        sliders_found = []
        
        # 1. Iterate through the main layout to find the Row Widgets
        for i in range(container.count()):
            item = container.itemAt(i)
            widget = item.widget()
            
            # Skip empty items or the zero button itself
            if widget is None or widget == self.ui.zeropushButton:
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

        # Update ghost robot with zero positions
        if self.logic is not None and self.robot is not None:
            self.logic.updateGhostTransformsFromJointsKDL(self.robot, self.jointPositionsRad)
            self.logic.last_ik_solution = self.jointPositionsRad.copy()

    def onMoveGroupToggled(self, toggled: bool) -> None:
        if toggled:
            print("Enabling MoveIt IK")
            self.logic.useMoveItIK = True
            
            if self.ui.plangroupline.text == "":
                print("Warning: No MoveIt planning group specified.")
            self.robot.setupIKmoveit(self.ui.plangroupline.text)
        else:
            print("Disabling MoveIt IK")
            self.logic.useMoveItIK = False

            
    def onTabChanged(self, index):
            if not self.isRobotLoaded:
                print("Robot not loaded yet; ignoring tab change.")
                return
            
            # 1. Get the widget that is currently visible
            current_widget = self.ui.tabWidget.widget(index)
            
            # 2. Check: Is this widget MY control tab?
            if current_widget == self.ui.controltab:
                print("Detected: controltab is now OPEN")
                # Call your start function here
                self.enterControlMode()
                
            else:
                print("Detected: controltab is now HIDDEN (User went somewhere else)")
                self.exitControlMode()

    def enterControlMode(self):
                # 1. Check if Robot is Loaded
                if not self.isRobotLoaded: return
                if not self.robot or not self.rootlink: return

                print(">> Enter Control Mode: Creating Sphere & Starting IK...")

                # 2. Find Robot Root Transform (Target for IK calculation)
                try:
                    self.totransform = self.logic.findRobotTransforms(self.rootlink)
                except RuntimeError:
                    print("Error: Could not find robot root transform.")
                    return

                # 3. Create Sphere (Probe) if missing
                try:
                    model = slicer.util.getNode("ProbeSphere")
                except slicer.util.MRMLNodeNotFoundException:
                    model = None

                if model is None:
                    model = self.logic.createSphereModel()
                    self.fromtransform = self.logic.createLinearTransform()
                    self.logic.applyTransformToModel(model, self.fromtransform)
                else:
                    self.fromtransform = slicer.util.getNode("ProbeSphere_Transform")

                # --- UPDATED: SNAP SPHERE TO TIP ON START ---
                # This aligns the probe (Pos + Rot) with the robot immediately, 
                # solving the "Impossible Orientation" issue at startup.
                try:
                    # A. Decide which tip to follow: Ghost (Preferred) -> Real (Fallback)
                    target_tip_link = self.ghosttiplink if self.ghosttiplink else self.tiplink
                    use_ghost = (target_tip_link == self.ghosttiplink)

                    if target_tip_link:
                        # B. Find the Transform Node
                        tip_transform_node = self.logic.findRobotTransforms(target_tip_link, ghost=use_ghost)
                        
                        if tip_transform_node:
                            # C. Get the Full 4x4 Matrix (Pos + Rot) in World Coordinates
                            tipMatrix = vtk.vtkMatrix4x4()
                            tip_transform_node.GetMatrixTransformToWorld(tipMatrix)
                            
                            # D. Apply to Probe
                            self.fromtransform.SetMatrixTransformToParent(tipMatrix)
                            print(f"✅ Snapped Probe Pose to {target_tip_link} (Pos + Rot)")
                            
                            # DEBUG check
                            # print(f"   Initial Rot: {tipMatrix.GetElement(0,0):.2f}...")
                    else:
                        print("Warning: No tip link found to snap to.")
                        
                except Exception as e:
                    print(f"Warning: Could not snap sphere to tip. Error: {e}")
                # ----------------------------------------

                # 4. LINK VISUALS TO LOGIC
                self.logic.setIKSourceTransforms(
                    self.fromtransform.GetName(), 
                    self.totransform.GetName()
                )
                
                # 5. PASS TIP LINK TO LOGIC
                print(f"\n=== TIP LINK CONFIGURATION ===")
                print(f"rootlink (base): {self.rootlink}")
                print(f"tiplink (target): {self.tiplink}")
                print(f"ghosttiplink: {self.ghosttiplink}")
                print(f"================================\n")
                self.logic.tipLink = self.tiplink
                
                # 6. START OBSERVER
                self.logic.addObserverComputeIK(self.robot)
            
    def exitControlMode(self):
        if not self.isRobotLoaded: return
        
        print(">> Exit Control Mode: Stopping IK & Cleaning up...")
        
        # 1. Stop Observer (Stop calculating IK)
        # This effectively replaces the "Stop" button
        if self.logic:
            self.logic.removeObserver()
        
        # 2. Delete Sphere Transform
        if self.fromtransform:
            slicer.mrmlScene.RemoveNode(self.fromtransform)
            self.fromtransform = None
        
        # 3. Delete Sphere Model
        try:
            model = slicer.util.getNode("ProbeSphere")
            if model: slicer.mrmlScene.RemoveNode(model)
        except: pass

    def onPlanButton(self) -> None:
        sol = self.robot.PlanMoveItTrajectoryJSON(self.ui.plangroupline.text,self.logic.last_ik_solution)
        print(f"Plan button clicked - received solution: {sol}")
        
        # Parse and store the trajectory
        if sol:
            self.ui.previewbutton.enabled = True
            self.ui.executebutton.enabled = True
            try:
                trajectory = json.loads(sol)
                if "points" in trajectory:
                    self.trajectoryData = trajectory
                    num_points = len(trajectory['points'])
                    print(f"Trajectory stored with {num_points} points")
                    
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
                    moveitLayout = self.ui.moveittab.layout()
                    if moveitLayout:
                        moveitLayout.addWidget(self.trajectorySliderWidget)
                    
                    # Show first point of trajectory
                    if self.robot and num_points > 0:
                        positions = trajectory['points'][0]['positions']
                        self.logic.updateGhostTransformsFromJointsKDL(self.robot, positions)
                else:
                    print("Error: No points found in trajectory")
            except json.JSONDecodeError as e:
                print(f"Error parsing trajectory JSON: {e}")
        
    def onpreviewButton(self) -> None:
        """Preview the planned trajectory on the ghost robot"""
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
        print(f"Starting trajectory preview with {len(self.trajectoryData['points'])} points")
    
    def animateTrajectoryStep(self):
        """Animate one step of the trajectory"""
        if not self.trajectoryData or self.trajectoryIndex >= len(self.trajectoryData['points']):
            # Animation complete
            if self.trajectoryTimer:
                self.trajectoryTimer.stop()
            print("Trajectory preview complete")
            return
        
        # Get current point
        point = self.trajectoryData['points'][self.trajectoryIndex]
        positions = point['positions']
        
        # Apply to ghost robot
        if self.robot:
            self.logic.updateGhostTransformsFromJointsKDL(self.robot, positions)
            print(f"Point {self.trajectoryIndex}/{len(self.trajectoryData['points'])-1}: {[f'{p:.3f}' for p in positions]}")
        
        self.trajectoryIndex += 1
    
    def onTrajectorySliderChanged(self, value):
        """Called when trajectory slider is moved"""
        if not self.trajectoryData or not self.robot:
            return
        
        # Stop any running animation
        if self.trajectoryTimer:
            self.trajectoryTimer.stop()
        
        # Get the trajectory point at this index
        if 0 <= value < len(self.trajectoryData['points']):
            point = self.trajectoryData['points'][value]
            positions = point['positions']
            
            # Apply to ghost robot
            self.logic.updateGhostTransformsFromJointsKDL(self.robot, positions)
            
            # Update spinbox if it's not the source of the change
            if self.trajectorySpinBox.value != value:
                self.trajectorySpinBox.blockSignals(True)
                self.trajectorySpinBox.setValue(value)
                self.trajectorySpinBox.blockSignals(False)
                
    def onExecuteButton(self) -> None:
        success = self.robot.ExecuteCachedMoveItTrajectory(self.ui.plangroupline.text)
        
        if success:
            print("Execute command sent successfully.")
        else:
            print("Failed to send execute command.")
        
    

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
        self.obsTag = None
        self.obsNode = None
        self.callback = None  
        self.toNode = None
        self.plangroup = None
        self.tipLink = None  # Will be set from widget
        self.last_ik_solution = []  # Will be sized based on actual joint count
        self.joint_names = []  # Will be populated from URDF
        self.joint_state_publisher = None
        self.useMoveItIK = False  # Flag to toggle between KDL and MoveIt IK
        
    def getParameterNode(self):
        return ROS2MotionControlParameterNode(super().getParameterNode())

    def process(self,
                inputVolume: vtkMRMLScalarVolumeNode,
                outputVolume: vtkMRMLScalarVolumeNode,
                imageThreshold: float,
                invert: bool = False,
                showResult: bool = True) -> None:
        """
        Run the processing algorithm.
        Can be used without GUI widget.
        :param inputVolume: volume to be thresholded
        :param outputVolume: thresholding result
        :param imageThreshold: values above/below this threshold will be set to 0
        :param invert: if True then values above the threshold will be set to 0, otherwise values below are set to 0
        :param showResult: show output volume in slice viewers
        """

        if not inputVolume or not outputVolume:
            raise ValueError("Input or output volume is invalid")

        import time

        startTime = time.time()
        logging.info("Processing started")

        # Compute the thresholded output volume using the "Threshold Scalar Volume" CLI module
        cliParams = {
            "InputVolume": inputVolume.GetID(),
            "OutputVolume": outputVolume.GetID(),
            "ThresholdValue": imageThreshold,
            "ThresholdType": "Above" if invert else "Below",
        }
        cliNode = slicer.cli.run(slicer.modules.thresholdscalarvolume, None, cliParams, wait_for_completion=True, update_display=showResult)
        # We don't need the CLI module node anymore, remove it to not clutter the scene with it
        slicer.mrmlScene.RemoveNode(cliNode)

        stopTime = time.time()
        logging.info(f"Processing completed in {stopTime-startTime:.2f} seconds")
        
    
    def createSphereModel(self, name="ProbeSphere", radius_mm=20.0):
        r = radius_mm
        src = vtk.vtkSphereSource()
        src.SetRadius(r); src.SetThetaResolution(40); src.SetPhiResolution(40); src.Update()

        model = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelNode", name)
        model.SetAndObservePolyData(src.GetOutput())

        disp = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLModelDisplayNode", name+"_Display")
        disp.SetOpacity(0.6); disp.SetBackfaceCulling(0); disp.SetVisibility3D(True)
        disp.SetColor(0.9,0.3,0.3)
        model.SetAndObserveDisplayNodeID(disp.GetID())
        return model
    
    def createLinearTransform(self, name="ProbeSphere_Transform", showAxes=True):
        t = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLLinearTransformNode", name)
        if not t.GetDisplayNode():
            tdisp = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLTransformDisplayNode", name+"_Display")
            tdisp.SetVisibility(True)  # “eye” in Data
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
    
    def findRobotTransforms(self, link_name, ghost=False):
        """
        Locates the Slicer Transform node for a given link by looking for 
        the visual model and getting its parent.
        """
        # Construct expected model name
        model_name = f"{link_name}_model"
        
        if ghost:
            model_name = link_name

        
        # Find model node, if exsits, get its parent transform
        model_node = slicer.util.getNode(model_name)
        if model_node is None:
            raise RuntimeError(f"Model for link '{link_name}' not found as '{model_name}'")
        parent = model_node.GetParentTransformNode()
        if parent:
            print(f"Found transform for '{link_name}' via model '{model_name}': {parent.GetName()}")
            return parent
        else:
            raise RuntimeError(f"Could not find Transform for link '{link_name}'")

    def addObserver(self, fromTransformName, toTransformName):
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
            self.printLocation(fromNode, toNode)

        self.callback = onModified
        # Prefer TransformModifiedEvent for transforms; ModifiedEvent also works
        eventId = slicer.vtkMRMLTransformNode.TransformModifiedEvent
        self.obsTag  = fromNode.AddObserver(eventId, self.callback)
        self.obsNode = fromNode
        self.toNode = toNode
        return self.obsTag
        
    def printLocation(self, fromNode, toNode):
        m = vtk.vtkMatrix4x4()
        ok = slicer.vtkMRMLTransformNode.GetMatrixTransformBetweenNodes(fromNode, toNode, m)
        if not ok:
            print("Could not compute transform between nodes.")
            return
        x, y, z = m.GetElement(0, 3), m.GetElement(1, 3), m.GetElement(2, 3)
        print(f"{fromNode.GetName()} wrt {toNode.GetName()}: x={x:.2f}, y={y:.2f}, z={z:.2f} mm")

    def removeObserver(self):
        if self.obsNode and self.obsTag is not None:
            try:
                self.obsNode.RemoveObserver(self.obsTag)
                print(f"[ROS2MotionControlLogic] Removed observer (tag={self.obsTag}) from {self.obsNode.GetName()}")
            except Exception as e:
                print(f"[ROS2MotionControlLogic] Error removing observer: {e}")
        else:
            if self.obsTag is not None:
                print(f"[ROS2MotionControlLogic] No obsNode to remove observer from (tag={self.obsTag})")
        self.obsNode = None
        self.obsTag = None
        self.callback = None
    
    
    def computeIKWithMoveIt(self, robotmodel, tipLink):

    # --- Get Slicer transform nodes ---
        fromNode = self.obsNode
        toNode   = self.toNode
        
        # --- Compute 4×4 transform between nodes ---
        targetPose = vtk.vtkMatrix4x4()
        if not slicer.vtkMRMLTransformNode.GetMatrixTransformBetweenNodes(fromNode, toNode, targetPose):
            raise RuntimeError("Could not compute transform between nodes.")
        
        # DEBUG: Print full 4x4 matrix
        print(f"\n[MoveIt IK] Target Pose Matrix (4x4):")
        print(f"  [{targetPose.GetElement(0,0):.3f}, {targetPose.GetElement(0,1):.3f}, {targetPose.GetElement(0,2):.3f}, {targetPose.GetElement(0,3):.2f}]")
        print(f"  [{targetPose.GetElement(1,0):.3f}, {targetPose.GetElement(1,1):.3f}, {targetPose.GetElement(1,2):.3f}, {targetPose.GetElement(1,3):.2f}]")
        print(f"  [{targetPose.GetElement(2,0):.3f}, {targetPose.GetElement(2,1):.3f}, {targetPose.GetElement(2,2):.3f}, {targetPose.GetElement(2,3):.2f}]")
        print(f"  [{targetPose.GetElement(3,0):.3f}, {targetPose.GetElement(3,1):.3f}, {targetPose.GetElement(3,2):.3f}, {targetPose.GetElement(3,3):.3f}]")
        
        seed = self.last_ik_solution
        result_str = robotmodel.FindIKmoveit(targetPose, tipLink, seed, 0.05)

        if result_str and result_str.strip():
            # Parse comma-separated string into list of floats
            try:
                data = [float(x) for x in result_str.split(",")]
                print(f"[IK] Joint Solution: {data}")
                self.last_ik_solution = data
                # Publish the joint state solution
                self.updateGhostTransformsFromJointsKDL(robotmodel, data)
                return data

            except ValueError as e:
                print(f"[IK] Failed to parse solution: {e}")
                return None
        else:
            print(f"[IK] Empty result from FindIK")
            return None
    
    
    def compute_ik_once(self, robotmodel):            
            # --- Get Slicer transform nodes ---
            fromNode = self.obsNode
            toNode   = self.toNode
            
            if fromNode is None or toNode is None:
                return None

            # DEBUG: Log which transforms we're computing between
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
                    print(f"[IK] Solution found: {data}")
                    self.last_ik_solution = data
                    self.updateGhostTransformsFromJointsKDL(robotmodel, data)
                    return data
                except ValueError as e:
                    print(f"[IK] Failed to parse solution: {e}")
                    return None
            else:
                print(f"[IK] Empty result from FindKDLIK")
                return None
    
    def addObserverComputeIK(self, robotmodel=None):
            """
            Observe transform changes. Uses self.obsNode and self.toNode that should be
            set by setupikforRobot(). Each transform update triggers IK computation.
            Uses either KDL (default) or MoveIt IK based on useMoveItIK flag.
            
            Args:
                robotmodel: The robot model for KDL IK
                Planning group name is read from self.plangroup (updated by widget)
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
                # Get the target pose
                targetPose = vtk.vtkMatrix4x4()
                success = slicer.vtkMRMLTransformNode.GetMatrixTransformBetweenNodes(
                    self.obsNode, self.toNode, targetPose
                )
                
                if not success:
                    return
                
                # Choose IK solver based on flag
                if self.useMoveItIK:
                    # Use MoveIt IK via robotmodel integration
                    solution = self.computeIKWithMoveIt(robotmodel=robotmodel, tipLink=self.tipLink)
                    if solution:
                        print(f"[MoveIt IK] Solution: {solution}")
                else:
                    # Use KDL IK (original method)
                    self.compute_ik_once(robotmodel=robotmodel)

            self.callback = onModified
            eventId = slicer.vtkMRMLTransformNode.TransformModifiedEvent
            self.obsTag  = fromNode.AddObserver(eventId, self.callback)

            return self.obsTag
    
    # Set robot opacity
    def setopacity(self, robotmodel, opacity):
        
        # Get number of model nodes under robotmodel
        numModels = robotmodel.GetNumberOfNodeReferences("model")  

        # Loop through each model node and set opacity
        for i in range(numModels):  
            modelNode = robotmodel.GetNthNodeReference("model", i)  
            displayNode = modelNode.GetDisplayNode()  
            if displayNode:  
                displayNode.SetOpacity(opacity)

    # Set robot color
    def setRobotColor(self, robotNode, color):
        
        # Get number of model nodes under robotmodel
        numModels = robotNode.GetNumberOfNodeReferences("model")  

        # Loop through each model node and set color
        for i in range(numModels):  
            modelNode = robotNode.GetNthNodeReference("model", i)  
            displayNode = modelNode.GetDisplayNode()  
            if displayNode:  
                r = color.red() / 255.0
                g = color.green() / 255.0
                b = color.blue() / 255.0
                displayNode.SetColor(r, g, b)
    
    # Parse urdf to get joint limits. parses only non-fixed joints
    def parse_joint_limits_from_urdf(self, urdf_xml: str):        
        root = ET.fromstring(urdf_xml)
        limits = {} 

        for joint in root.findall("joint"):
            jtype = joint.get("type", "")
            name = joint.get("name", "")
            if jtype == "fixed" or not name:
                continue

            limit = joint.find("limit")
            if limit is None:
                continue

            lo = limit.get("lower")
            hi = limit.get("upper")
            if lo is None or hi is None:
                continue

            limits[name] = (float(lo), float(hi))

        return limits
    
    # Parse urdf to get joint names. parses only non-fixed joints
    def parse_joint_names_from_urdf(self, urdf_xml: str):
        root = ET.fromstring(urdf_xml)
        names = []

        for joint in root.findall("joint"):
            jtype = joint.get("type", "")
            name = joint.get("name", "")
            if not name:
                continue
            if jtype == "fixed":
                continue
            names.append(name)

        return names
    
    # Parse urdf to get all joint names and types
    def parse_all_joint_types_from_urdf(self, urdf_xml: str):
        root = ET.fromstring(urdf_xml)
        name_to_type = {}

        for joint in root.findall("joint"):
            name = joint.get("name", "")
            if not name:
                continue
            jtype = joint.get("type", "")
            name_to_type[name] = jtype

        return name_to_type
    
    # Parse urdf to get all link names
    def parse_all_link_names_from_urdf(self, urdf_xml: str):
        root = ET.fromstring(urdf_xml)
        names = []

        for link in root.findall("link"):
            name = link.get("name", "")
            if not name:
                continue
            names.append(name)

        return names
    
    def parse_joint_structure_from_urdf(self, urdf_xml: str):
            """
            Returns a dict mapping joint_name -> {'parent', 'child', 'axis', 'type', 'origin_rpy'}
            """
            import xml.etree.ElementTree as ET
            root = ET.fromstring(urdf_xml)
            joint_info = {}

            for joint in root.findall("joint"):
                name = joint.get("name", "")
                jtype = joint.get("type", "")
                
                # Skip if no name or fixed (unless you need fixed joints later)
                if not name or jtype == "fixed":
                    continue

                parent_elem = joint.find("parent")
                child_elem = joint.find("child")
                axis_elem = joint.find("axis")
                origin_elem = joint.find("origin")  # <--- NEW: Find the origin tag
                
                if parent_elem is None or child_elem is None:
                    continue
                    
                parent_link = parent_elem.get("link", "")
                child_link = child_elem.get("link", "")
                
                # Default axis
                axis = [0.0, 0.0, 1.0]
                if axis_elem is not None:
                    xyz = axis_elem.get("xyz", "0 0 1")
                    try:
                        axis = [float(x) for x in xyz.split()]
                    except ValueError:
                        pass
                
                # --- NEW: Extract Origin RPY ---
                origin_rpy = [0.0, 0.0, 0.0]
                if origin_elem is not None:
                    rpy_str = origin_elem.get("rpy", "0 0 0")
                    try:
                        origin_rpy = [float(x) for x in rpy_str.split()]
                    except ValueError:
                        pass
                # -------------------------------
                
                joint_info[name] = {
                    'parent': parent_link,
                    'child': child_link,
                    'axis': axis,
                    'type': jtype,
                    'origin_rpy': origin_rpy  # <--- Store it here
                }

            return joint_info

    
    def setJointSlidersFromUrdfLimits(self, limits_rad, sliders):

        if len(sliders) != len(limits_rad):
            print(
                f"[ROS2MotionControl] Slider count ({len(sliders)}) "
                f"!= joint count ({len(limits_rad)})"
            )

        for slider, (jointName, (lo_rad, hi_rad)) in zip(sliders, limits_rad.items()):
            lo_deg = int(round(math.degrees(lo_rad)))
            hi_deg = int(round(math.degrees(hi_rad)))

            if lo_deg > hi_deg:
                lo_deg, hi_deg = hi_deg, lo_deg

            slider.setRange(lo_deg, hi_deg)
            slider.setValue(0)

            print(f"[ROS2MotionControl] {jointName}: {lo_deg}..{hi_deg} deg")
            
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
                print(f"Logic Linked: '{fromtransformname}' -> '{totransformname}'")
            else:
                print("Error: Could not link IK transforms (Nodes missing)")
                
    def getcurrentjointpositions(self, robotmodel):
            """
            Calculate current joint values by subtracting the fixed URDF origin 
            from the measured link transforms.
            """

            if not robotmodel:
                print("Error: No robot model provided")
                return []
            
            # Get joint names
            joint_names = robotmodel.GetJoints()
            if not joint_names:
                print("Error: No joints found in robot model")
                return []
            
            # Get URDF to parse joint structure
            pnode = robotmodel.GetNthNodeReference("parameter", 0)
            if not pnode:
                print("Error: No parameter node found for robot")
                return []
            urdf_xml = pnode.GetParameterAsString("robot_description")
            
            # Parse joint structure (Must include origin_rpy logic!)
            joint_info = self.parse_joint_structure_from_urdf(urdf_xml)
            
            joint_positions = []
            
            for joint_name in joint_names:
                try:
                    # 1. Validation
                    if joint_name not in joint_info:
                        print(f"Warning: Joint '{joint_name}' not found in URDF")
                        joint_positions.append(0.0)
                        continue
                    
                    info = joint_info[joint_name]
                    parent_link = info['parent']
                    child_link = info['child']
                    axis = info['axis']
                    # Default to [0,0,0] if not found
                    origin_rpy = info.get('origin_rpy', [0.0, 0.0, 0.0]) 
                    
                    # 2. Find Slicer Nodes for Parent and Child
                    real_parent_model = f"{parent_link}"
                    real_child_model = f"{child_link}"
                    
                    try:
                        parent_transform = self.findRobotTransforms(real_parent_model, ghost=False)
                        child_transform = self.findRobotTransforms(real_child_model, ghost=False)
                    except Exception:
                        # If transforms aren't in the scene, assume 0.0
                        joint_positions.append(0.0)
                        continue
                    
                    # 3. Get Measured Matrix (Total Transform = Origin * JointRotation)
                    # This calculates the transform of Child relative to Parent
                    measured_matrix = vtk.vtkMatrix4x4()
                    slicer.vtkMRMLTransformNode.GetMatrixTransformBetweenNodes(
                        child_transform, parent_transform, measured_matrix
                    )
                    
                    # 4. Create the Structural Origin Matrix
                    # We apply rotations in Z, Y, X order which corresponds to URDF Euler RPY
                    origin_transform = vtk.vtkTransform()
                    origin_transform.RotateZ(math.degrees(origin_rpy[2]))
                    origin_transform.RotateY(math.degrees(origin_rpy[1]))
                    origin_transform.RotateX(math.degrees(origin_rpy[0]))
                    
                    origin_matrix = origin_transform.GetMatrix()
                    
                    # 5. Isolate the Joint Rotation
                    # Math: T_rotation = (T_origin)^-1 * T_measured
                    origin_matrix.Invert() 
                    
                    pure_joint_matrix = vtk.vtkMatrix4x4()
                    vtk.vtkMatrix4x4.Multiply4x4(origin_matrix, measured_matrix, pure_joint_matrix)
                    
                    # 6. Extract the Angle from the Pure Matrix
                    final_transform = vtk.vtkTransform()
                    final_transform.SetMatrix(pure_joint_matrix)
                    rotation = final_transform.GetOrientation()  # Returns [rx, ry, rz] in degrees
                    
                    # 7. Select the correct axis
                    # Find the dominant axis (largest absolute value in the axis vector)
                    abs_axis = [abs(a) for a in axis]
                    max_idx = abs_axis.index(max(abs_axis))
                    
                    angle_deg = rotation[max_idx]
                    
                    # Flip sign if the axis definition is negative (e.g., [0, 0, -1])
                    if axis[max_idx] < 0:
                        angle_deg = -angle_deg
                        
                    # Clamp tiny noise to 0.0 to prevent flickering
                    if abs(angle_deg) < 0.1:
                        angle_deg = 0.0

                    angle_rad = math.radians(angle_deg)
                    
                    # Debug print (Optional)
                    # print(f"Joint {joint_name}: raw={rotation}, fixed={angle_deg}")

                    joint_positions.append(angle_rad)
                        
                except Exception as e:
                    print(f"Error reading real robot joint '{joint_name}': {e}")
                    joint_positions.append(0.0)
            
            return joint_positions
    
    def getcurrentghostjointpositions(self, robotmodel):
        """
        Calculate current joint values from the ghost robot's link transforms.
        Returns a list of joint angles in radians.
        """
        if not robotmodel:
            print("Error: No robot model provided")
            return []
        
        # Get joint names
        joint_names = robotmodel.GetJoints()
        if not joint_names:
            print("Error: No joints found in robot model")
            return []
        
        # Get URDF to parse joint structure
        pnode = robotmodel.GetNthNodeReference("parameter", 0)
        if not pnode:
            print("Error: No parameter node found for robot")
            return []
        urdf_xml = pnode.GetParameterAsString("robot_description")
        
        # Parse joint structure from URDF
        joint_info = self.parse_joint_structure_from_urdf(urdf_xml)
        
        joint_positions = []
        
        # For each joint, compute the relative rotation between parent and child links
        # Ghost links have "_ghost" suffix in their model names
        for joint_name in joint_names:
            try:
                if joint_name not in joint_info:
                    print(f"Warning: Joint '{joint_name}' not found in URDF")
                    joint_positions.append(0.0)
                    continue
                
                info = joint_info[joint_name]
                parent_link = info['parent']
                child_link = info['child']
                axis = info['axis']
                
                # Ghost links have model names with "_ghost" suffix
                # When ghost=True, findRobotTransforms expects the full model node name
                ghost_parent_model = f"{parent_link}_model_ghost"
                ghost_child_model = f"{child_link}_model_ghost"
                
                # Get transform nodes for ghost parent and child links
                try:
                    parent_transform = self.findRobotTransforms(ghost_parent_model, ghost=True)
                    child_transform = self.findRobotTransforms(ghost_child_model, ghost=True)
                except RuntimeError as e:
                    print(f"Warning: Could not find ghost transforms for joint '{joint_name}': {e}")
                    joint_positions.append(0.0)
                    continue
                
                # Compute relative transform from child to parent
                rel_matrix = vtk.vtkMatrix4x4()
                slicer.vtkMRMLTransformNode.GetMatrixTransformBetweenNodes(
                    child_transform, parent_transform, rel_matrix
                )
                
                # Extract rotation angle around the joint axis
                transform = vtk.vtkTransform()
                transform.SetMatrix(rel_matrix)
                rotation = transform.GetOrientation()  # Returns [rx, ry, rz] in degrees
                
                # Determine which axis component to use based on joint axis
                # Find the dominant axis (largest absolute value)
                abs_axis = [abs(a) for a in axis]
                max_idx = abs_axis.index(max(abs_axis))
                
                angle_deg = rotation[max_idx]
                # If axis is negative, flip the angle
                if axis[max_idx] < 0:
                    angle_deg = -angle_deg
                    
                angle_rad = math.radians(angle_deg)
                print(f"[Ghost] {joint_name}: axis={axis}, rotation_deg={[f'{r:.1f}' for r in rotation]}, extracted_deg={angle_deg:.1f}, rad={angle_rad:.4f}")
                joint_positions.append(angle_rad)
                    
            except Exception as e:
                print(f"Error reading ghost joint '{joint_name}': {e}")
                joint_positions.append(0.0)
        
        return joint_positions
    
    def updateGhostTransformsFromJointsKDL(self, robotmodel, joint_values):
        """
        Update all ghost robot link transforms using KDL FK computation.
        For each link, calls ComputeKDLFK to get the transform and applies it to the ghost link.
        
        Args:
            robotmodel: The robot model node with ComputeLocalTransform method
            joint_values: List of joint angles in radians
        """
        if not robotmodel or not joint_values:
            print("[updateGhostTransformsFromJointsKDL] No robot model or joint values")
            return False
        
        seg = robotmodel.GetSegments()
        
        # For each link, compute FK and update ghost transform
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
                
                # Find the ghost link's transform node
                ghost_link_name = link_name + "_model_ghost"
                try:
                    ghost_model = slicer.util.getNode(ghost_link_name)
                    if ghost_model:
                        ghost_transform = ghost_model.GetParentTransformNode()
                        if ghost_transform:
                            # Apply the FK matrix to the ghost transform
                            ghost_transform.SetMatrixTransformToParent(fk_matrix)
                            print(fk_matrix)
                            print(f"[FK] Updated ghost transform for '{link_name}'")
                except:
                    print(f"[FK] Could not find or update ghost model for '{link_name}'")
                    
            except Exception as e:
                print(f"[FK] Error computing FK for link '{link_name}': {e}")
        
        return True
        


#
# ROS2MotionControlTest
#


class ROS2MotionControlTest(ScriptedLoadableModuleTest):
    """
    This is the test case for your scripted module.
    Uses ScriptedLoadableModuleTest base class, available at:
    https://github.com/Slicer/Slicer/blob/main/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def setUp(self):
        """Do whatever is needed to reset the state - typically a scene clear will be enough."""
        slicer.mrmlScene.Clear()

    def runTest(self):
        """Run as few or as many tests as needed here."""
        self.setUp()
        self.test_ROS2MotionControl1()

    def test_ROS2MotionControl1(self):
        """Ideally you should have several levels of tests.  At the lowest level
        tests should exercise the functionality of the logic with different inputs
        (both valid and invalid).  At higher levels your tests should emulate the
        way the user would interact with your code and confirm that it still works
        the way you intended.
        One of the most important features of the tests is that it should alert other
        developers when their changes will have an impact on the behavior of your
        module.  For example, if a developer removes a feature that you depend on,
        your test should break so they know that the feature is needed.
        """

        self.delayDisplay("Starting the test")

        # Get/create input data

        import SampleData

        registerSampleData()
        inputVolume = SampleData.downloadSample("ROS2MotionControl1")
        self.delayDisplay("Loaded test data set")

        inputScalarRange = inputVolume.GetImageData().GetScalarRange()
        self.assertEqual(inputScalarRange[0], 0)
        self.assertEqual(inputScalarRange[1], 695)

        outputVolume = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLScalarVolumeNode")
        threshold = 100

        # Test the module logic

        logic = ROS2MotionControlLogic()

        # Test algorithm with non-inverted threshold
        logic.process(inputVolume, outputVolume, threshold, True)
        outputScalarRange = outputVolume.GetImageData().GetScalarRange()
        self.assertEqual(outputScalarRange[0], inputScalarRange[0])
        self.assertEqual(outputScalarRange[1], threshold)

        # Test algorithm with inverted threshold
        logic.process(inputVolume, outputVolume, threshold, False)
        outputScalarRange = outputVolume.GetImageData().GetScalarRange()
        self.assertEqual(outputScalarRange[0], inputScalarRange[0])
        self.assertEqual(outputScalarRange[1], inputScalarRange[1])

        self.delayDisplay("Test passed")
