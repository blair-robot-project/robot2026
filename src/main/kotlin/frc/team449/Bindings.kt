package frc.team449

import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team449.RobotContainer.drive
import frc.team449.commands.SwerveRequestCommand
import frc.team449.subsystems.drive.Intake

class Bindings(
    val robotContainer: RobotContainer,
) {
    val driveController = robotContainer.driveController
    val opController = robotContainer.opController

    val intake = Intake()

    fun setDefaultCommands() {
        // set default commands for systems here
        robotContainer.drive.defaultCommand =
            SwerveRequestCommand(
                robotContainer.drive,
                { -robotContainer.driveController.leftY },
                { -robotContainer.driveController.leftX },
                { robotContainer.driveController.rightX },
            )
    }

    fun bindControls() {
        driveController
            .x()
            .onTrue(
                PrintCommand("X Button Pressed!"),
            )

        driveController.y().onTrue(intake.runIntake())
        driveController.x().onTrue(intake.runIntake())

        opController.povUp().whileTrue(drive.sysIDTranslationRoutine.quasistatic(SysIdRoutine.Direction.kForward))
        opController.povUp().whileTrue(drive.sysIDTranslationRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
        opController.povUp().whileTrue(drive.sysIDTranslationRoutine.dynamic(SysIdRoutine.Direction.kForward))
        opController.povUp().whileTrue(drive.sysIDTranslationRoutine.dynamic(SysIdRoutine.Direction.kReverse))
    }
}
