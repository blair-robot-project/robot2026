package frc.team449

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.Constants.Mode
import frc.team449.generated.TunerConstants
import frc.team449.subsystems.drive.DriveIO
import frc.team449.subsystems.drive.DriveIOHardware
import frc.team449.subsystems.drive.DriveSubsystem
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
                    object : DriveIO {}
                }

                Mode.REPLAY -> {
                    object : DriveIO {}
                }
            },
        )

    val bindings = Bindings(this)

    val autoChooser = LoggedDashboardChooser<Command>("Auto Routines")
}
