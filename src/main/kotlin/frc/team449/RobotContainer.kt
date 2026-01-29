package frc.team449

import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.team449.Constants.Mode
import frc.team449.generated.TunerConstants
import frc.team449.subsystems.climb.PivotIOHardware
import frc.team449.subsystems.drive.DriveIO
import frc.team449.subsystems.drive.DriveIOHardware
import frc.team449.subsystems.drive.DriveIOSim
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.intake.Intake
import frc.team449.subsystems.intake.pivot.PivotIO
import frc.team449.subsystems.intake.pivot.PivotIOSim
import frc.team449.subsystems.intake.roller.RollerIO
import frc.team449.subsystems.intake.roller.RollerIOHardware
import frc.team449.subsystems.intake.roller.RollerIOSim

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
                        arrayOf(TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight),
                    )
                }

                Mode.SIM -> {
                    DriveIOSim(
                        TunerConstants.DrivetrainConstants,
                        arrayOf(TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight),
                    )
                }

                Mode.REPLAY -> {
                    object : DriveIO {}
                }
            },
        )

    val intake: Intake =
        Intake(
            when (Constants.CURRENT_MODE) {
                Mode.REAL -> PivotIOHardware()
                Mode.SIM -> PivotIOSim()
                Mode.REPLAY -> object : PivotIO {}
            },
            when (Constants.CURRENT_MODE) {
                Mode.REAL -> RollerIOHardware()
                Mode.SIM -> RollerIOSim()
                Mode.REPLAY -> object : RollerIO {}
            },
        )

    val bindings = Bindings(this)
}
