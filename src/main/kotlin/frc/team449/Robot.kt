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
    private val robotContainer = RobotContainer

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

    override fun robotInit() {
        FieldUtil.initialize()

        robotContainer.bLineRoutines.addAutoOptions(robotContainer.autoChooser)
        robotContainer.bindings.setDefaultCommands()
        robotContainer.bindings.bindControls()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()
        PhoenixUtil.refreshAll()

        Logger.recordOutput("Robot/Mode", Constants.CURRENT_MODE.name)
        Logger.recordOutput("MatchTime", DriverStation.getMatchTime())

        logComponentPoses()
    }

    override fun autonomousInit() {
        robotContainer.drive.setOperatorPerspectiveForward()
        FieldUtil.updateKeyPositions()
        CommandScheduler.getInstance().schedule(robotContainer.actions.stopAllAndHomeHood())

        robotContainer.autonomousCommand = robotContainer.autoChooser.get()
        CommandScheduler.getInstance().schedule(robotContainer.autonomousCommand)
    }

    override fun autonomousPeriodic() {
        robotContainer.bLineRoutines.logBLineAuto()
    }

    override fun teleopInit() {
        robotContainer.autonomousCommand.cancel()
        robotContainer.drive.setOperatorPerspectiveForward()
        FieldUtil.updateKeyPositions()

        CommandScheduler.getInstance().schedule(robotContainer.actions.stopAllAndHomeHood())
    }

    override fun teleopPeriodic() {
        if (!FieldUtil.autoWinnerLogged) FieldUtil.autoWinnerLogged = FieldUtil.updateAutoWinner()
    }

    private fun logComponentPoses() {
        val pivotAngle = robotContainer.intake.pivotAngle
        val hoodAngle = robotContainer.shooter.hoodAngle

        val hopperTranslationX = MathUtil.inverseInterpolate(
            Constants.IntakeConstants.STOW_POS_RADS,
            Constants.IntakeConstants.DEPLOY_POS_RADS,
            pivotAngle
        ) * 0.3

        Logger.recordOutput(
            "FinalComponentPoses",
            Pose3d(0.3, 0.0, 0.2, Rotation3d(0.0, pivotAngle, 0.0)),
            Pose3d(hopperTranslationX, 0.0, 0.0, Rotation3d()),
            Pose3d(-0.1, 0.0, 0.4, Rotation3d(0.0, hoodAngle + 0.2591940418, 0.0))
        )
    }
}
