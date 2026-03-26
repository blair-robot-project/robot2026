package frc.team449

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.Constants.Mode
import frc.team449.Constants.VisionConstants
import frc.team449.auto.BLineRoutines
import frc.team449.generated.TunerConstants
import frc.team449.subsystems.RobotActions
import frc.team449.subsystems.drive.DriveIO
import frc.team449.subsystems.drive.DriveIOHardware
import frc.team449.subsystems.drive.DriveIOSim
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.indexer.IndexerIO
import frc.team449.subsystems.indexer.IndexerIOHardware
import frc.team449.subsystems.indexer.IndexerIOSim
import frc.team449.subsystems.indexer.IndexerSubsystem
import frc.team449.subsystems.intake.IntakeIO
import frc.team449.subsystems.intake.IntakeIOHardware
import frc.team449.subsystems.intake.IntakeIOSim
import frc.team449.subsystems.intake.IntakeSubsystem
import frc.team449.subsystems.shooter.ShooterIO
import frc.team449.subsystems.shooter.ShooterIOHardware
import frc.team449.subsystems.shooter.ShooterIOSim
import frc.team449.subsystems.shooter.ShooterSubsystem
import frc.team449.subsystems.vision.VisionIO
import frc.team449.subsystems.vision.VisionIOLimelight
import frc.team449.subsystems.vision.VisionSubsystem
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import mrSchaferSim.*

object RobotContainer {
    // driver/op controllers
    val driveController: CommandXboxController = CommandXboxController(0)
    val opController: CommandXboxController = CommandXboxController(1)

    var autonomousCommand: Command = PrintCommand("If you see this, you probably didn't run an auto.")

    val drive: DriveSubsystem =
        DriveSubsystem(
            when (Constants.CURRENT_MODE) {
                Mode.REAL -> {
                    DriveIOHardware(
                        TunerConstants.DrivetrainConstants,
                        arrayOf(
                            TunerConstants.FrontLeft,
                            TunerConstants.FrontRight,
                            TunerConstants.BackLeft,
                            TunerConstants.BackRight,
                        ),
                    )
                }

                Mode.SIM -> {
                    DriveIOSim(
                        TunerConstants.DrivetrainConstants,
                        arrayOf(
                            TunerConstants.FrontLeft,
                            TunerConstants.FrontRight,
                            TunerConstants.BackLeft,
                            TunerConstants.BackRight,
                        ),
                    )
                }

                Mode.REPLAY -> {
                    object : DriveIO {}
                }
            },
        )

    val vision: VisionSubsystem =
        when (Constants.CURRENT_MODE) {
            Mode.REAL ->
                VisionSubsystem(
                    drive::addVisionMeasurement,
                    VisionIOLimelight(
                        VisionConstants.CAMERA_RIGHT_NAME,
                        { drive.pose.rotation },
                        drive::getAngularVelocity,
                        VisionConstants.ROBOT_TO_CAMERA_RIGHT
                    ),
                    VisionIOLimelight(
                        VisionConstants.CAMERA_LEFT_NAME,
                        { drive.pose.rotation },
                        drive::getAngularVelocity,
                        VisionConstants.ROBOT_TO_CAMERA_LEFT
                    )
                )
//            Mode.SIM ->
//                VisionSubsystem(
//                    drive::addVisionMeasurement,
//                    VisionIOPhotonVisionSim("camera1", VisionConstants.ROBOT_TO_CAMERA_RIGHT, { drive.pose }),
//                    VisionIOPhotonVisionSim("camera2", VisionConstants.ROBOT_TO_CAMERA_LEFT, { drive.pose }),
//                )
            else -> VisionSubsystem(
                drive::addVisionMeasurement,
                object : VisionIO {},
                object : VisionIO {}
            ).also { vision = it }
        }

    val intake: IntakeSubsystem =
        IntakeSubsystem(
            when (Constants.CURRENT_MODE) {
                Mode.REAL -> IntakeIOHardware()
                Mode.SIM -> IntakeIOSim()
                Mode.REPLAY -> object : IntakeIO {}
            },
        )

    val indexer: IndexerSubsystem =
        IndexerSubsystem(
            when (Constants.CURRENT_MODE) {
                Mode.REAL -> IndexerIOHardware()
                Mode.SIM -> IndexerIOSim()
                Mode.REPLAY -> object : IndexerIO {}
            },
        )

    val shooter: ShooterSubsystem =
        ShooterSubsystem(
            when (Constants.CURRENT_MODE) {
                Mode.REAL -> ShooterIOHardware()
                Mode.SIM -> ShooterIOSim()
                Mode.REPLAY -> object : ShooterIO {}
            },
        )
    val bindings = Bindings(this)
    val simParams = ProjectileSimulator.SimParameters(
        0.215,   // ball mass kg
        0.1501,  // ball diameter m
        0.47,    // drag coeff
        0.2,     // Magnus coeff
        1.225,   // air density
        0.43,    // exit height m
        0.1016,  // flywheel diameter m
        1.83,    // target height m
        0.6,     // slip factor
        45.0,    // launch angle degrees
        0.001,   // sim timestep
        1500.0, 6000.0, 25, 5.0  // RPM range, iterations, max sim time
    )
    val sim = ProjectileSimulator(simParams)
    val lut = sim.generateLUT()
    val config = ShotCalculator.Config().apply {
        launcherOffsetX = 0.23
        launcherOffsetY = 0.0
        phaseDelayMs = 30.0
        mechLatencyMs = 20.0
        maxTiltDeg = 5.0
        headingSpeedScalar = 1.0
        headingReferenceDistance = 2.5
    }

    val shotCalc = ShotCalculator(config)

    val hubCenter = Translation2d(4.6, 4.0)
    val hubForward = Translation2d(1.0, 0.0)

    init {
        // optional: print the table to see what the sim produced
        shotCalc.loadLUTEntry(1.0, 2000.0, 0.45)
        shotCalc.loadLUTEntry(2.0, 2800.0, 0.62)
        shotCalc.loadLUTEntry(3.0, 3500.0, 0.78)
        /*lut.put(1.0, 2000.0, 45.0, 0.45)
        lut.put(2.0, 2800.0, 42.0, 0.62)
        lut.put(3.0, 3500.0, 38.0, 0.78)
        shotCalc.loadShotLUT(lut)*/
        //im not completely sure but I'm pretty sure it depends on if the hood angle moves.
        for (entry in lut.entries()) {
            if (entry.reachable()) {
                println(
                    "%.2fm -> %.0f RPM, %.3fs TOF".format(
                        entry.distanceM(), entry.rpm(), entry.tof()
                    )
                )
            }
        }

        // load into shotCalc
        for (entry in lut.entries()) {
            if (entry.reachable()) {
                shotCalc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof())
            }
        }
    }
    val actions = RobotActions(this)

    val bLineRoutines = BLineRoutines(drive, actions)
    val autoChooser = LoggedDashboardChooser<Command>("Auto Routines")
}
