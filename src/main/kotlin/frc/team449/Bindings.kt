package frc.team449

import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team449.RobotContainer.drive
import frc.team449.commands.SwerveRequestCommand

class Bindings(
    val robotContainer: RobotContainer
) {

    val charController = robotContainer.charController
    val driveController = robotContainer.driveController
    val opController = robotContainer.opController

    fun setDefaultCommands() {
        // set default commands for systems here
        robotContainer.drive.defaultCommand =
            SwerveRequestCommand(
                robotContainer.drive,
                { -robotContainer.driveController.leftY },
                { robotContainer.driveController.leftX },
                { -robotContainer.driveController.rightX }
            )
    }




    fun bindControls() {
        driveController
            .x()
            .onTrue(
                PrintCommand("X Button Pressed!")
            )

        charController.povUp().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        charController.povDown().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        charController.povLeft().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        charController.povRight().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));




    }
}


