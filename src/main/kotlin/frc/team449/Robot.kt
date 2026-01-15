package frc.team449

import choreo.auto.AutoChooser
import com.ctre.phoenix6.SignalLogger
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Threads
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import frc.team449.auto.bLineRoutines
import frc.team449.auto.choreoRoutines
import frc.team449.auto.pathRoutines
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

        // CanBridge.runTCP()

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

        print(Constants.CURRENT_MODE)
    }

    private val robotContainer = RobotContainer
    private val bindings = Bindings(robotContainer)

    override fun driverStationConnected() {
        robotContainer.drive.setOperatorPerspectiveForward()
    }

    override fun robotInit() {
        bindings.setDefaultCommands()
        bindings.bindControls()

      routines.addOptions(autoChooser)
        SmartDashboard.putData("Auto Chooser", autoChooser)
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler())


    }

    override fun robotPeriodic() {
        // high priority (real-time) thread for loop timing
        Threads.setCurrentThreadPriority(true, 99)

        CommandScheduler.getInstance().run()

        // return thread to low priority (standard)
        Threads.setCurrentThreadPriority(false, 10)
    }

    override fun autonomousInit() {
        CommandScheduler.getInstance().schedule(robotContainer.autonomousCommand)
    }

    override fun autonomousPeriodic() {}

    override fun teleopInit() {}

    override fun teleopPeriodic() {
    }

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun testInit() {}

    override fun testPeriodic() {}

    override fun simulationInit() {}

    override fun simulationPeriodic() {}
}
