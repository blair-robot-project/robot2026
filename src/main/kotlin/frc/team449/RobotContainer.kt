package frc.team449

import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.Constants.Mode
import frc.team449.generated.TunerConstants
import frc.team449.subsystems.RobotActions
import frc.team449.subsystems.drive.DriveIO
import frc.team449.subsystems.drive.DriveIOHardware
import frc.team449.subsystems.drive.DriveIOSim
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.vision.Vision
import frc.team449.subsystems.vision.VisionConstants
import frc.team449.subsystems.vision.VisionIO
import frc.team449.subsystems.vision.VisionIOLimelight
import frc.team449.subsystems.vision.VisionIOPhotonVisionSim
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

object RobotContainer {
    // driver/op controllers
    val driveController: CommandXboxController = CommandXboxController(0)
    val opController: CommandXboxController = CommandXboxController(1)

    val autonomousCommand = PrintCommand("This is the autonomous command!")

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
    val vision: Vision =
        when (Constants.CURRENT_MODE) {
            Mode.REAL ->
                Vision(
                    drive::addVisionMeasurement,
                    VisionIOLimelight("limelight-right", drive.rotation),
                    VisionIOLimelight("limelight-left", drive.rotation)
                )
            Mode.SIM ->
                Vision(
                    drive::addVisionMeasurement,
                    VisionIOPhotonVisionSim("camera1", VisionConstants.robotToCamera0, drive.pose),
                    VisionIOPhotonVisionSim("camera2", VisionConstants.robotToCamera1, drive.pose),
                )
            Mode.REPLAY -> Vision(
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

    val actions = RobotActions(this)

    val bindings = Bindings(this)
}
