package frc.team449

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.PowerDistribution
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team449.util.FieldUtil
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedPowerDistribution
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
                Logger.addDataReceiver(WPILOGWriter("/u/logs"))
                Logger.addDataReceiver(NT4Publisher())
                DataLogManager.start("/u/logs")
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

        // TODO: correct PDH CAN ID
        LoggedPowerDistribution.getInstance(0, PowerDistribution.ModuleType.kRev)

        SignalLogger.enableAutoLogging(false)
        Logger.start()
    }

    override fun robotInit() {
        RobotController.setBrownoutVoltage(6.3)

        FieldUtil.initializeAutoWinnerField()

        robotContainer.bindings.setDefaultCommands()
        robotContainer.bindings.bindControls()
    }

    override fun robotPeriodic() {
        CommandScheduler.getInstance().run()

        Logger.recordOutput("Robot/Mode", Constants.CURRENT_MODE.name)
        Logger.recordOutput("MatchTime", DriverStation.getMatchTime())

        logComponentPoses()
    }

    override fun autonomousInit() {
        robotContainer.drive.setOperatorPerspectiveForward()

        robotContainer.autonomousCommand = robotContainer.autoChooser.get()
        CommandScheduler.getInstance().schedule(robotContainer.autonomousCommand)
    }

    override fun autonomousPeriodic() {}

    override fun teleopInit() {
        robotContainer.autonomousCommand.cancel()
        robotContainer.drive.setOperatorPerspectiveForward()
    }

    override fun teleopPeriodic() {
        if (!FieldUtil.autoWinnerLogged) FieldUtil.autoWinnerLogged = FieldUtil.updateAutoWinner()
    }

    private fun logComponentPoses() {
    }
}
