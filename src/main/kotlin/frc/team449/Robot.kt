package frc.team449

import com.ctre.phoenix6.SignalLogger
import edu.wpi.first.hal.FRCNetComm
import edu.wpi.first.hal.HAL
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Threads
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team449.firecontrol.FuelPhysicsSim
import org.littletonrobotics.junction.LogFileUtil
import org.littletonrobotics.junction.LoggedRobot
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.NT4Publisher
import org.littletonrobotics.junction.wpilog.WPILOGReader
import org.littletonrobotics.junction.wpilog.WPILOGWriter
import java.util.function.Supplier
import frc.team449.firecontrol.*

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
    private lateinit var ballSim: FuelPhysicsSim
    private val robotContainer = RobotContainer

    override fun driverStationConnected() {
        robotContainer.drive.setOperatorPerspectiveForward()
    }

    override fun robotInit() {
        robotContainer.bindings.setDefaultCommands()
        robotContainer.bindings.bindControls()
    }

    override fun robotPeriodic() {
        Threads.setCurrentThreadPriority(true, 99)
        CommandScheduler.getInstance().run()
        Threads.setCurrentThreadPriority(false, 10)

        val speeds = RobotContainer.drive.getRobotRelativeSpeeds()
        val pose = RobotContainer.drive.getPose()
        val robotVel = Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
        val fieldVel = robotVel.rotateBy(pose.rotation)

        val shotInputs = ShotCalculator.ShotInputs(
            pose,
            fieldVel,
            robotVel,
            RobotContainer.hubCenter,
            RobotContainer.hubForward,
            0.9,
            0.0,
            0.0
        )

        val shot = RobotContainer.shotCalc.calculate(shotInputs)
        if (shot.isValid && shot.confidence() > 50) {
            RobotContainer.shooter.setRPM(shot.rpm())
            RobotContainer.drive.aimAt(shot.driveAngle())
        }
    }
    override fun autonomousInit() {
        CommandScheduler.getInstance().schedule(robotContainer.autonomousCommand)
    }

    override fun autonomousPeriodic() {}

    override fun teleopInit() {}

    override fun teleopPeriodic() {}

    override fun disabledInit() {}

    override fun disabledPeriodic() {}

    override fun testInit() {}

    override fun testPeriodic() {}

    override fun simulationInit() {
        ballSim = FuelPhysicsSim("Sim/Fuel")
        ballSim.enable()
        ballSim.placeFieldBalls()
        //these parameters need to be changed
        ballSim.configureRobot(robotWidth, robotLength, bumperHeight,
            Supplier { robotContainer.drive.pose },
            Supplier { robotContainer.drive.chassisSpeeds }
        )
    }

    override fun simulationPeriodic() {
        ballSim.tick()
    }
}
