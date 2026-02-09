package frc.team449

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Threads
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import frc.team449.RobotContainer.drive
import frc.team449.RobotContainer.fuelSim

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

    override fun teleopPeriodic() {}

    override fun disabledInit() {
    }

    override fun disabledPeriodic() {}

    override fun testInit() {}

    override fun testPeriodic() {}

    override fun simulationInit() {
        fuelSim.spawnStartingFuel()
        fuelSim.registerRobot(
            Constants.Dimensions.FULL_WIDTH.`in`(Meters),
            Constants.Dimensions.FULL_LENGTH.`in`(Meters),
            Constants.Dimensions.BUMPER_HEIGHT.`in`(Meters),
            { drive.getPose() },
            drive::getFieldRelativeSpeeds,
        )

        fuelSim.registerIntake(
            Constants.Dimensions.FULL_LENGTH
                .div(2.0)
                .`in`(Meters),
            Constants.Dimensions.FULL_LENGTH
                .div(2.0)
                .plus(Inches.of(3.0))
                .`in`(Meters),
            -Constants.Dimensions.FULL_WIDTH
                .div(2.0)
                .minus(Inches.of(2.0))
                .`in`(Meters),
            Constants.Dimensions.FULL_WIDTH
                .div(2.0)
                .minus(Inches.of(5.0))
                .`in`(Meters),
        )

        fuelSim.start()
        SmartDashboard.putData(
            Commands
                .runOnce(
                    {
                        fuelSim.clearFuel()
                        fuelSim.spawnStartingFuel()
                    },
                ).withName("Reset Fuel")
                .ignoringDisable(true),
        )
    }

    override fun simulationPeriodic() {
        fuelSim.updateSim()
    }
}
