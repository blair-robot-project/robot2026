package frc.team449

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Threads
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import frc.team449.RobotContainer.drive
import frc.team449.RobotContainer.fuelSim
import frc.team449.util.PhoenixUtil
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
            { drive.pose },
            drive::getFieldRelativeSpeeds,
        )

        fuelSim.registerIntake(
            (Constants.Dimensions.FULL_LENGTH + Inches.of(12.0))
                .div(2.0)
                .`in`(Meters),
            (Constants.Dimensions.FULL_LENGTH + Inches.of(12.0))
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
            robotContainer.intake.isSimIntaking(),
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
        Logger.recordOutput("isSimintaking", robotContainer.intake.isSimIntaking())

        Logger.recordOutput("ZeroedComponentPoses", *Array(3) { Pose3d() })
        Logger.recordOutput(
            "FinalComponentPoses",
            *arrayOf(
                Pose3d(0.3, 0.0, 0.2, Rotation3d(0.0, robotContainer.intake.intakeSimAngle, 0.0)),
                Pose3d(
                    MathUtil.inverseInterpolate(
                        Constants.IntakeConstants.STOW_POS_RADS,
                        Constants.IntakeConstants.DEPLOY_POS_RADS,
                        robotContainer.intake.intakeSimAngle,
                    ) * 0.3,
                    0.0,
                    0.0,
                    Rotation3d(),
                ),
                Pose3d(
                    -0.1,
                    0.0,
                    0.4,
                    Rotation3d(0.0, robotContainer.shooter.hoodSimAngle, 0.0),
                ),
            ),
        )
    }
}
