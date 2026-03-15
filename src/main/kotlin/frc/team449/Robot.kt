package frc.team449

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team449.util.FieldUtil
import frc.team449.util.PhoenixUtil
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter

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

    override fun driverStationConnected() {}

    override fun robotInit() {
        FieldUtil.initialize()
        robotContainer.bLineRoutines.addAutoOptions(robotContainer.autoChooser)

        robotContainer.bindings.setDefaultCommands()
        robotContainer.bindings.bindControls()
    }

    override fun robotPeriodic() {
        Logger.recordOutput("Robot/Mode", Constants.CURRENT_MODE.name)
        Logger.recordOutput("Match Time", DriverStation.getMatchTime())
        PhoenixUtil.refreshAll()

        // high priority (real-time) thread for loop timing
//        Threads.setCurrentThreadPriority(true, 99)
        CommandScheduler.getInstance().run()

        // return thread to low priority (standard)
//        Threads.setCurrentThreadPriority(false, 10)

        Logger.recordOutput(
            "FinalComponentPoses",
            *arrayOf(
                Pose3d(0.3, 0.0, 0.2, Rotation3d(0.0, robotContainer.intake.pivotAngle, 0.0)),
                Pose3d(
                    MathUtil.inverseInterpolate(
                        Constants.IntakeConstants.STOW_POS_RADS,
                        Constants.IntakeConstants.DEPLOY_POS_RADS,
                        robotContainer.intake.pivotAngle,
                    ) * 0.3,
                    0.0,
                    0.0,
                    Rotation3d(),
                ),
                Pose3d(
                    -0.1,
                    0.0,
                    0.4,
                    Rotation3d(0.0, robotContainer.shooter.hoodAngle + 0.2591940418, 0.0),
                ),
            ),
        )
    }

    override fun autonomousInit() {
        robotContainer.drive.setOperatorPerspectiveForward()
        robotContainer.autonomousCommand = robotContainer.autoChooser.get()
        CommandScheduler.getInstance().schedule(robotContainer.autonomousCommand)
        FieldUtil.updateKeyPositions()
        robotContainer.shooter.homeHood()
    }

    override fun autonomousPeriodic() {
        robotContainer.bLineRoutines.logBLineAuto()
    }

    override fun teleopInit() {
        robotContainer.autonomousCommand.cancel()
        robotContainer.drive.setOperatorPerspectiveForward()

        FieldUtil.updateAutoWinner()
        FieldUtil.updateKeyPositions()

        robotContainer.actions.stopAllAndHomeHood()
    }

    override fun teleopPeriodic() {}

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun testInit() {}

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}
