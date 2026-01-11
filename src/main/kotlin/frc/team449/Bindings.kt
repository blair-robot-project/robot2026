package frc.team449

import edu.wpi.first.wpilibj2.command.PrintCommand
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
                { -robotContainer.driveController.rightX }
            )
    }

    fun bindControls() {
        driveController
            .x()
            .onTrue(
                PrintCommand("X Button Pressed!")
            )
    }
}
