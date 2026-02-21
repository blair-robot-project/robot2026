package frc.team449

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Threads
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team449.util.PhoenixUtil
import frc.team449.subsystems.vision.LimelightHelpers
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter

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
            LimelightHelpers.SetRobotOrientation(
                "limelight-right",
                robotContainer.drive.inputs.gyroAngle,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            LimelightHelpers.SetRobotOrientation(
                "limelight-left",
                robotContainer.drive.inputs.gyroAngle,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
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
    }

    override fun autonomousInit() {
        CommandScheduler.getInstance().schedule(robotContainer.autonomousCommand)

        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            LimelightHelpers.SetIMUMode("limelight-right", 4)
            LimelightHelpers.SetIMUMode("limelight-left", 4)
        }
    }

    override fun autonomousPeriodic() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            LimelightHelpers.SetRobotOrientation(
                "limelight-right",
                robotContainer.drive.inputs.gyroAngle,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            LimelightHelpers.SetRobotOrientation(
                "limelight-left",
                robotContainer.drive.inputs.gyroAngle,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
        }
    }

    override fun teleopInit() {}

    override fun teleopPeriodic() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            LimelightHelpers.SetRobotOrientation(
                "limelight-right",
                robotContainer.drive.inputs.gyroAngle,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            LimelightHelpers.SetRobotOrientation(
                "limelight-left",
                robotContainer.drive.inputs.gyroAngle,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
        }
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun testInit() {}

    override fun testPeriodic() {}

    override fun simulationInit() {
        Logger.recordOutput("ZeroedComponentPoses", *Array(3) { Pose3d() })
    }

    override fun simulationPeriodic() {
        Logger.recordOutput(
            "FinalComponentPoses",
            *arrayOf(
                Pose3d(0.3, 0.0, 0.2, Rotation3d(0.0, robotContainer.intake.intakeSimAngle, 0.0)),
                Pose3d(
                    MathUtil.inverseInterpolate(
                        Constants.IntakeConstants.STOW_POS_RADS,
                        Constants.IntakeConstants.DEPLOY_POS_RADS,
                        robotContainer.intake.intakeSimAngle
                    ) * 0.3,
                    0.0,
                    0.0,
                    Rotation3d()
                ),
                Pose3d(
                    -0.1,
                    0.0,
                    0.4,
                    Rotation3d(0.0, robotContainer.shooter.hoodSimAngle, 0.0)
                )
            )
        )
    }
}
