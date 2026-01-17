package frc.team449

import choreo.auto.AutoChooser
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.Constants.Mode
import frc.team449.auto.Routines
import frc.team449.generated.TunerConstants
import frc.team449.subsystems.drive.DriveIO
import frc.team449.subsystems.drive.DriveIOHardware
import frc.team449.subsystems.drive.DriveIOSim
import frc.team449.subsystems.drive.DriveSubsystem

object RobotContainer {
    // driver/op controllers
    val driveController: CommandXboxController = CommandXboxController(0)
    val opController: CommandXboxController = CommandXboxController(1)

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

    val autoChooser = AutoChooser()
    val routines = Routines(this, true).apply {
        this.addOptions(autoChooser)
    }

    val bindings = Bindings(this)
}
