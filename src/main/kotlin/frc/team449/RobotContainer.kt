package frc.team449

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
import frc.team449.subsystems.vision.QuestNav
import frc.team449.subsystems.vision.VisionIO
import frc.team449.subsystems.vision.VisionIOLimelight
import frc.team449.subsystems.vision.VisionIOPhotonVisionSim
import frc.team449.subsystems.vision.VisionSubsystem
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object RobotContainer {
    // driver controller
    val driveController: CommandXboxController = CommandXboxController(0)
    val operatorController: CommandXboxController = CommandXboxController(1)
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
                    { questNav.getIsDisconnected },

                    VisionIOLimelight(
                        VisionConstants.CAMERA_RIGHT_NAME,
                        { drive.pose },
                        { drive.getAngularVelocity() },
                        VisionConstants.ROBOT_TO_CAMERA_RIGHT
                    ),
//                    VisionIOLimelight(
//                        VisionConstants.CAMERA_LEFT_NAME,
//                        { drive.pose },
//                        { drive.getAngularVelocity() },
//                        VisionConstants.ROBOT_TO_CAMERA_LEFT
//                    ),
                )
            Mode.SIM ->
                VisionSubsystem(
                    drive::addVisionMeasurement,
                    { questNav.getIsDisconnected },
                    VisionIOPhotonVisionSim("camera1", VisionConstants.ROBOT_TO_CAMERA_RIGHT) { drive.pose },
//                    VisionIOPhotonVisionSim("camera2", VisionConstants.ROBOT_TO_CAMERA_LEFT) { drive.pose },
                )
            else -> VisionSubsystem(
                drive::addVisionMeasurement,
                { questNav.getIsDisconnected },
//                object : VisionIO {},
                object : VisionIO {},
            )
        }

    val questNav: QuestNav =
        QuestNav(
            VisionConstants.ROBOT_TO_VISIONQUEST,
            drive::addVisionMeasurement,
        )

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

    val bLineRoutines = BLineRoutines(drive, actions)
    val autoChooser = LoggedDashboardChooser<Command>("Auto Routines")
}
