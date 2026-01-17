package frc.team449

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
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
        opController
            .x()
            .onTrue(
                robotContainer.drive.sysIdRoutineTranslation.quasistatic(SysIdRoutine.Direction.kForward)
            )

        opController
            .y()
            .onTrue(
                robotContainer.drive.sysIdRoutineTranslation.quasistatic(SysIdRoutine.Direction.kReverse)
            )

        opController
            .a()
            .onTrue(
                robotContainer.drive.sysIdRoutineTranslation.dynamic(SysIdRoutine.Direction.kForward)
            )

        opController
            .b()
            .onTrue(
                robotContainer.drive.sysIdRoutineTranslation.dynamic(SysIdRoutine.Direction.kReverse)
            )
    }
}
