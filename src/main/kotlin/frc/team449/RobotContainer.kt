package frc.team449

import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.Constants.Mode
import frc.team449.generated.TunerConstants
import frc.team449.subsystems.drive.DriveIO
import frc.team449.subsystems.drive.DriveIOHardware
import frc.team449.subsystems.drive.DriveIOSim
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.vision.Vision
import frc.team449.subsystems.vision.VisionIOLimelight

object RobotContainer {
    // driver/op controllers
    val driveController: CommandXboxController = CommandXboxController(0)
    val opController: CommandXboxController = CommandXboxController(1)

    val autonomousCommand = PrintCommand("This is the autonomous command!")

    val drive: DriveSubsystem = DriveSubsystem(
        when (Constants.CURRENT_MODE) {
            Mode.REAL -> DriveIOHardware(
                TunerConstants.DrivetrainConstants,
                arrayOf(TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight)
            )
            Mode.SIM -> DriveIOSim(
                TunerConstants.DrivetrainConstants,
                arrayOf(TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight)
            )
            Mode.REPLAY -> object : DriveIO {}
        }
    )
    val vision: Vision =
        when (Constants.CURRENT_MODE) {
            Mode.REAL ->
                Vision(
                    drive::addVisionMeasurement,
                    VisionIOLimelight("limelight1", drive.rotation),
                    VisionIOLimelight("limelight2", drive.rotation)
                )
            Mode.SIM -> TODO()
            Mode.REPLAY -> TODO()
        }

    val bindings = Bindings(this)
}
