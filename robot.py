import sys
import wpilib
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.controller import PIDController

from AutoSequencerV2.autoSequencer import AutoSequencer
from drivetrain.controlStrategies.autoDrive import AutoDrive
from drivetrain.controlStrategies.trajectory import Trajectory
from drivetrain.drivetrainControl import DrivetrainControl
from humanInterface.driverInterface import DriverInterface

# from humanInterface.ledControl import LEDControl


# from subsystems.presets import Presets
from utils.crashLogger import CrashLogger
from utils.powerMonitor import PowerMonitor
from utils.rioMonitor import RIOMonitor
from utils.segmentTimeTracker import SegmentTimeTracker
from utils.signalLogging import logUpdate
from utils.singleton import destroyAllSingletonInstances
from webserver.webserver import Webserver

kLEDBuffer = 150


class MyRobot(wpilib.TimedRobot):
    #########################################################
    ## Common init/update for all modes
    def robotInit(self):
        # Since we're defining a bunch of new things here, tell pylint
        # to ignore these instantiations in a method.
        # pylint: disable=attribute-defined-outside-init
        self.enableLiveWindowInTest(True)

        #########################################################

        self.timer = wpilib.Timer()

        wpilib.CameraServer.launch()

        remoteRIODebugSupport()

        self.crashLogger = CrashLogger()

        # We do our own logging, we don't need additional logging in the background.
        # Both of these will increase CPU load by a lot, and we never use their output.
        wpilib.LiveWindow.disableAllTelemetry()

        self.webserver = Webserver()

        self.driveTrain = DrivetrainControl()
        self.autodrive = AutoDrive()

        self.stt = SegmentTimeTracker()

        self.dInt = DriverInterface()
        self.stick = self.dInt.ctrl
        self.stick2 = wpilib.XboxController(1)

        self.autoSequencer = AutoSequencer()

        self.rioMonitor = RIOMonitor()
        self.pwrMon = PowerMonitor()

        self.autoHasRun = False

    def robotPeriodic(self):
        pass

        #########################################################

        self.stt.start()

        self.dInt.update()
        self.stt.mark("Driver Interface")

        self.driveTrain.update()
        self.stt.mark("Drivetrain")

        self.autodrive.updateTelemetry()
        self.driveTrain.poseEst._telemetry.setCurAutoDriveWaypoints(
            self.autodrive.getWaypoints()
        )
        self.driveTrain.poseEst._telemetry.setCurObstacles(
            self.autodrive.rfp.getObstacleStrengths()
        )
        self.stt.mark("Telemetry")

        logUpdate()
        self.stt.end()

    def autonomousInit(self):
        self.timer.start()
        self.driveTrain.resetGyro()
        # Start up the autonomous sequencer
        self.autoSequencer.initialize()

        # Use the autonomous routines starting pose to init the pose estimator
        self.driveTrain.poseEst.setKnownPose(self.autoSequencer.getStartingPose())
        # self.driveTrain.poseEst.setKnownPose(Pose2d(1.0, 1.0, Rotation2d(0.0)))

        # Mark we at least started autonomous
        self.autoHasRun = True

        self.timer.restart()

    def autonomousPeriodic(self):
        self.autoSequencer.update()

    def autonomousExit(self):
        self.autoSequencer.end()
        # print("auto done.")

    ## Teleop-Specific init and update
    def teleopInit(self):
        # clear existing telemetry trajectory
        self.driveTrain.poseEst._telemetry.setCurAutoTrajectory(None)

        # If we're starting teleop but haven't run auto, set a nominal default pose
        # This is needed because initial pose is usually set by the autonomous routine
        if not self.autoHasRun:
            self.driveTrain.poseEst.setKnownPose(Pose2d(1.0, 1.0, Rotation2d(0.0)))
            print("not running.")
        self.timer.restart()

    def teleopPeriodic(self):
        # LIMELIGHT ALIGN (Left Side Values: TX: 27.58 TY: 5.74) (Right Side Values: TX: -10.6 TY: 8.37)
        self.driveTrain.setManualCmd(self.dInt.getCmd())

        if self.stick.getRawButton(7):
            self.driveTrain.resetGyro()

        # No trajectory in Teleop
        Trajectory().setCmd(None)

        #########################################################

    ## Disabled-Specific init and update
    def disabledPeriodic(self):
        self.autoSequencer.updateMode()
        Trajectory().trajHDC.updateCals()

    def disabledInit(self):
        self.autoSequencer.updateMode(True)

    ## Cleanup
    def endCompetition(self):
        # Sometimes `robopy test pyfrc_test.py` will invoke endCompetition() without completing robotInit(),
        # this will create a confusing exception here because we can reach self.rioMonitor.stopThreads()
        # when self.rioMonitor does not exist.
        # To prevent the exception and confusion, we only call self.rioMonitor.stopThreads() when exists.
        rioMonitorExists = getattr(self, "rioMonitor", None)
        if rioMonitorExists is not None:
            self.rioMonitor.stopThreads()

        destroyAllSingletonInstances()
        super().endCompetition()


def remoteRIODebugSupport():
    if __debug__ and "run" in sys.argv:
        print("Starting Remote Debug Support....")
        try:
            import debugpy  # pylint: disable=import-outside-toplevel
        except ModuleNotFoundError:
            pass
        else:
            debugpy.listen(("0.0.0.0", 5678))
            debugpy.wait_for_client()
