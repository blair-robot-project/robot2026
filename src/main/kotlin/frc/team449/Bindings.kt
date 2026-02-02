package frc.team449

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team449.RobotContainer.drive
import frc.team449.commands.SwerveRequestCommand

class Bindings(
    val robotContainer: RobotContainer
) {

    val driveController = robotContainer.driveController
    val opController = robotContainer.opController

    fun setDefaultCommands() {
        // set default commands for systems here
        robotContainer.drive.defaultCommand =
            SwerveRequestCommand(
                robotContainer.drive,
                { -robotContainer.driveController.leftY },
                { -robotContainer.driveController.leftX },
                { robotContainer.driveController.rightX }
            )
    }

    fun bindControls() {
        if (Constants.RUNNING_SHOOTER_SIM) {
            driveController
                .y()
                .onTrue(
                    Commands.sequence(
                        PrintCommand("Y Button Pressed!"),
                        robotContainer.shooter.stop()
                    )
                )

            driveController
                .x()
                .onTrue(
                    Commands.sequence(
                        PrintCommand("X Button Pressed!"),
                        robotContainer.shooter.shoot()
                    )

                )

            driveController
                .a()
                .onTrue(
                    Commands.sequence(
                        PrintCommand("A Button Pressed!"),
                        robotContainer.shooter.setHood(Constants.ShooterConstants.HOOD_MAX_ANGLE),
                        WaitUntilCommand { robotContainer.shooter.atTolerance() },
                        robotContainer.shooter.holdHood()
                    )
                )

            driveController
                .b()
                .onTrue(
                    Commands.sequence(
                        PrintCommand("B Button Pressed!"),
                        robotContainer.shooter.setHood(Constants.ShooterConstants.HOOD_MIN_ANGLE),
                        WaitUntilCommand { robotContainer.shooter.atTolerance() },
                        robotContainer.shooter.holdHood()
                    )

                )
        }

        opController.povUp().whileTrue(drive.sysIDTranslationRoutine.quasistatic(SysIdRoutine.Direction.kForward))
        opController.povUp().whileTrue(drive.sysIDTranslationRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
        opController.povUp().whileTrue(drive.sysIDTranslationRoutine.dynamic(SysIdRoutine.Direction.kForward))
        opController.povUp().whileTrue(drive.sysIDTranslationRoutine.dynamic(SysIdRoutine.Direction.kReverse))
    }
}
