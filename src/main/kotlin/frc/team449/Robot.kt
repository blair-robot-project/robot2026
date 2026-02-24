package frc.team449

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Threads
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team449.RobotContainer.limelightl
import frc.team449.RobotContainer.limelightr
import frc.team449.subsystems.vision.LimelightHelpers
import frc.team449.subsystems.vision.VisionConstants
import frc.team449.util.PhoenixUtil
import limelight.networktables.*
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import java.util.*
import kotlin.math.pow

/** The main class of the robot, constructs all the subsystems
 * and initializes default commands . */
class Robot : LoggedRobot() {
    init {
        println("Initializing Robot!")

        HAL.report(FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin)
        DriverStation.silenceJoystickConnectionWarning(true)

        when (Constants.CURRENT_MODE) {
            Constants.Mode.REAL -> {
                Logger.addDataReceiver(WPILOGWriter())
                Logger.addDataReceiver(NT4Publisher())
            }

            Constants.Mode.SIM -> {
                Logger.addDataReceiver(NT4Publisher())
            }

            Constants.Mode.REPLAY -> {
                this.setUseTiming(false) // run as fast as possible
                val logPath: String = LogFileUtil.findReplayLog()
                Logger.setReplaySource(WPILOGReader(logPath))
                Logger.addDataReceiver(WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")))
            }
        }

        SignalLogger.enableAutoLogging(false)
        Logger.start()
    }

    private val robotContainer = RobotContainer

    override fun driverStationConnected() {
        robotContainer.drive.setOperatorPerspectiveForward()
    }

    override fun robotInit() {
        robotContainer.bindings.setDefaultCommands()
        robotContainer.bindings.bindControls()

        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            limelightr!!.settings
                .withLimelightLEDMode(LimelightSettings.LEDMode.PipelineControl)
                .withCameraOffset(VisionConstants.robotToCameraRight)
                .save()

            limelightl!!.settings
                .withLimelightLEDMode(LimelightSettings.LEDMode.PipelineControl)
                .withCameraOffset(VisionConstants.robotToCameraLeft)
                .save()

            limelightr.settings
                .withRobotOrientation(
                    Orientation3d(
                        Rotation3d(Rotation2d(robotContainer.drive.inputs.gyroAngle)),
                        AngularVelocity3d(
                            DegreesPerSecond.of(robotContainer.driveIOHardware.getPitchVelocity()),
                            DegreesPerSecond.of(robotContainer.driveIOHardware.getRollVelocity()),
                            DegreesPerSecond.of(robotContainer.driveIOHardware.getYawVelocity())
                        )
                    )
                )
                .save()
            limelightl.settings
                .withRobotOrientation(
                    Orientation3d(
                        Rotation3d(Rotation2d(robotContainer.drive.inputs.gyroAngle)),
                        AngularVelocity3d(
                            DegreesPerSecond.of(robotContainer.driveIOHardware.getPitchVelocity()),
                            DegreesPerSecond.of(robotContainer.driveIOHardware.getRollVelocity()),
                            DegreesPerSecond.of(robotContainer.driveIOHardware.getYawVelocity())
                        )
                    )
                )
                .save()

            LimelightHelpers.SetIMUMode("limelight-right", 1)
            LimelightHelpers.SetIMUMode("limelight-left", 1)
        }
    }

    override fun robotPeriodic() {
        PhoenixUtil.refreshAll()

        // high priority (real-time) thread for loop timing
        Threads.setCurrentThreadPriority(true, 99)
        CommandScheduler.getInstance().run()

        // return thread to low priority (standard)
        Threads.setCurrentThreadPriority(false, 10)

        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            limelightr.settings
                .withRobotOrientation(
                    Orientation3d(
                        Rotation3d(Rotation2d(robotContainer.drive.inputs.gyroAngle)),
                        AngularVelocity3d(
                            DegreesPerSecond.of(robotContainer.driveIOHardware.getPitchVelocity()),
                            DegreesPerSecond.of(robotContainer.driveIOHardware.getRollVelocity()),
                            DegreesPerSecond.of(robotContainer.driveIOHardware.getYawVelocity())
                        )
                    )
                )
                .save()
            limelightl.settings
                .withRobotOrientation(
                    Orientation3d(
                        Rotation3d(Rotation2d(robotContainer.drive.inputs.gyroAngle)),
                        AngularVelocity3d(
                            DegreesPerSecond.of(robotContainer.driveIOHardware.getPitchVelocity()),
                            DegreesPerSecond.of(robotContainer.driveIOHardware.getRollVelocity()),
                            DegreesPerSecond.of(robotContainer.driveIOHardware.getYawVelocity())
                        )
                    )
                )
                .save()

            val visionEstimateRight: Optional<PoseEstimate> =
                limelightr.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG2).poseEstimate

            visionEstimateRight.ifPresent { poseEstimate: PoseEstimate ->

                if (poseEstimate.tagCount > 0 && poseEstimate.maxTagAmbiguity < VisionConstants.maxAmbiguity) {
                    robotContainer.drive.estimator.addVisionMeasurement(
                        poseEstimate.pose.toPose2d(),
                        poseEstimate.timestampSeconds
                    )
                }

                Logger.recordOutput(
                    "Vision/CameraRight/NewestPoseEstimate/Accepted",
                    (poseEstimate.tagCount > 0 && poseEstimate.maxTagAmbiguity < VisionConstants.maxAmbiguity)
                )
                Logger.recordOutput("Vision/CameraRight/NewestPoseEstimate/Pose", poseEstimate.pose.toPose2d())
                Logger.recordOutput("Vision/CameraRight/NewestPoseEstimate/TagCount", poseEstimate.tagCount)
                Logger.recordOutput("Vision/CameraRight/NewestPoseEstimate/AverageTagDistance", poseEstimate.avgTagDist)
                Logger.recordOutput("Vision/CameraRight/NewestPoseEstimate/AverageTagAmbiguity", poseEstimate.avgTagAmbiguity)
                Logger.recordOutput("Vision/CameraRight/NewestPoseEstimate/MaxiumumTagAmbiguity", poseEstimate.maxTagAmbiguity)
                Logger.recordOutput("Vision/CameraRight/NewestPoseEstimate/MinimumTagAmbiguity", poseEstimate.minTagAmbiguity)
                val stdDevFactor: Double =
                    poseEstimate.avgTagDist.pow(2.0) / poseEstimate.tagCount
                var linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor
                var angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor
                linearStdDev *= VisionConstants.linearStdDevMegatag2Factor
                angularStdDev *= VisionConstants.angularStdDevMegatag2Factor
                Logger.recordOutput("Vision/CameraRight/NewestPoseEstimate/StandardDeviationFactor", stdDevFactor)
                Logger.recordOutput("Vision/CameraRight/NewestPoseEstimate/LinearStandardDeviation", linearStdDev)
                Logger.recordOutput("Vision/CameraRight/NewestPoseEstimate/AngularStandardDeviation", angularStdDev)
            }

            val visionEstimateLeft: Optional<PoseEstimate> =
                limelightl.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG2).poseEstimate

            visionEstimateLeft.ifPresent { poseEstimate: PoseEstimate ->
                if (poseEstimate.tagCount > 0 && poseEstimate.maxTagAmbiguity < VisionConstants.maxAmbiguity) {
                    robotContainer.drive.estimator.addVisionMeasurement(
                        poseEstimate.pose.toPose2d(),
                        poseEstimate.timestampSeconds
                    )
                }
                Logger.recordOutput(
                    "Vision/CameraLeft/NewestPoseEstimate/Accepted",
                    (poseEstimate.tagCount > 0 && poseEstimate.maxTagAmbiguity < VisionConstants.maxAmbiguity)
                )
                Logger.recordOutput("Vision/CameraLeft/NewestPoseEstimate/Pose", poseEstimate.pose.toPose2d())
                Logger.recordOutput("Vision/CameraLeft/NewestPoseEstimate/TagCount", poseEstimate.tagCount)
                Logger.recordOutput("Vision/CameraLeft/NewestPoseEstimate/AverageTagDistance", poseEstimate.avgTagDist)
                Logger.recordOutput("Vision/CameraLeft/NewestPoseEstimate/AverageTagAmbiguity", poseEstimate.avgTagAmbiguity)
                Logger.recordOutput("Vision/CameraLeft/NewestPoseEstimate/MaxiumumTagAmbiguity", poseEstimate.maxTagAmbiguity)
                Logger.recordOutput("Vision/CameraLeft/NewestPoseEstimate/MinimumTagAmbiguity", poseEstimate.minTagAmbiguity)
                val stdDevFactor: Double =
                    poseEstimate.avgTagDist.pow(2.0) / poseEstimate.tagCount
                var linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor
                var angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor
                linearStdDev *= VisionConstants.linearStdDevMegatag2Factor
                angularStdDev *= VisionConstants.angularStdDevMegatag2Factor
                Logger.recordOutput("Vision/CameraLeft/NewestPoseEstimate/StandardDeviationFactor", stdDevFactor)
                Logger.recordOutput("Vision/CameraLeft/NewestPoseEstimate/LinearStandardDeviation", linearStdDev)
                Logger.recordOutput("Vision/CameraLeft/NewestPoseEstimate/AngularStandardDeviation", angularStdDev)
            }
        }
    }

    override fun autonomousInit() {
        CommandScheduler.getInstance().schedule(robotContainer.autonomousCommand)

        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            LimelightHelpers.SetIMUMode("limelight-right", 4)
            LimelightHelpers.SetIMUMode("limelight-left", 4)
        }
    }

    override fun autonomousPeriodic() {}

    override fun teleopInit() {}

    override fun teleopPeriodic() {}

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun testInit() {}

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}
