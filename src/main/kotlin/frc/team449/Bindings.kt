package frc.team449

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.team449.RobotContainer.drive
import frc.team449.commands.PoseAlignCommand
import frc.team449.commands.SmartXLockCommand
import frc.team449.commands.SwerveRequestCommand

class Bindings(
    val robotContainer: RobotContainer
) {
    val driver = robotContainer.driveController
    val operator = robotContainer.opController

    fun setDefaultCommands() {
        // set default commands for systems here
        robotContainer.drive.defaultCommand =
            SwerveRequestCommand(
                robotContainer.drive,
                { -driver.leftY },
                { -driver.leftX },
                { driver.rightX },
            )
    }

    fun bindControls() {
        driver
            .rightTrigger()
            .onTrue(
                InstantCommand()
                // intake deploy THEN
                // run intake
            )

        driver
            .leftTrigger()
            .onTrue(
                InstantCommand()
                // intake retract
            )

        driver
            .rightBumper()
            .whileTrue(
                InstantCommand()
                // manual shoot
            )
            .onFalse(
                InstantCommand()
                // coast shooter
            )

        driver
            .x()
            .onTrue(
                PoseAlignCommand(
                    robotContainer.drive,
                    { Field.getClosestTrenchPose(robotContainer.drive.pose) },
                    { -driver.leftY },
                    { -driver.leftX },
                    { driver.rightX },
                )
                // shoot
            )

        driver
            .a()
            .onTrue(
                SmartXLockCommand(
                    robotContainer.drive,
                    { -driver.leftY },
                    { -driver.leftX },
                    { driver.rightX },
                )
            )

        driver
            .start()
            .onTrue(
                robotContainer.drive.seedFieldCentric()
            )
    }
}
