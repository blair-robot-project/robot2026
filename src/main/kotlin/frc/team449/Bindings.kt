package frc.team449

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.commands.AimAtTargetCommand
import frc.team449.commands.SwerveRequestCommand
import frc.team449.util.FieldUtil
import kotlin.math.abs

class Bindings(
    val robotContainer: RobotContainer
) {
    val driver = robotContainer.driveController
    val operator = robotContainer.opController
    val actions = robotContainer.actions

    val joysticksMovedPastDeadbandTrigger: Trigger =
        Trigger {
            abs(driver.leftY) > 0.25 || abs(driver.leftX) > 0.25 ||
                abs(driver.rightX) > 0.25
        }

    val shooterJamTrigger: Trigger = robotContainer.shooter.shooterJamTrigger

    fun setDefaultCommands() {
        robotContainer.drive.defaultCommand =
            SwerveRequestCommand(
                robotContainer.drive,
                { -driver.leftY },
                { -driver.leftX },
                { -driver.rightX },
            )
    }

    fun bindControls() {
        driver
            .rightTrigger()
            .whileTrue(
                actions.deployAndRunIntake()
            )
            .onFalse(
                actions.stopIntakeAndPivot()
            )

        driver
            .leftTrigger()
            .onTrue(
                actions.stopAndStow()
            )

        driver
            .rightBumper()
            .whileTrue(
                Commands.sequence(
                    Commands.deadline(
                        AimAtTargetCommand(
                            robotContainer.drive,
                            { -driver.leftY },
                            { -driver.leftX },
                            targetSupplier = { FieldUtil.HUB_TRANSLATION }
                        ),
                        actions.prepShotFromAnywhere { FieldUtil.distanceToHub }.repeatedly()
                    ),
                    Commands.parallel(
                        robotContainer.drive.xLock(),
                        actions.checkAndFeed()
                    )
                )
            )
            .onFalse(
                actions.stopAll()
            )

        driver
            .leftBumper()
            .whileTrue(
                SwerveRequestCommand(
                    robotContainer.drive,
                    { -driver.leftY },
                    { -driver.leftX },
                    { -driver.rightX },
                    Constants.DriveConstants.SLOW_LINEAR_SPEED_METERS_PER_SEC,
                    Constants.DriveConstants.SLOW_ANGULAR_SPEED_RADS_PER_SEC,
                ),
            )

        driver
            .a()
            .onTrue(
                robotContainer.drive
                    .xLock()
                    .until(joysticksMovedPastDeadbandTrigger),
            )

        driver
            .x()
            .whileTrue(
                actions.prepShotFromAnywhere { 3.43 }
                    .andThen(
                        robotContainer.drive.xLock()
                            .alongWith(actions.checkAndFeed())
                            .alongWith(WaitCommand(1.0).andThen(actions.shuffleIntakePivot()))
                    )
            )
            .onFalse(
                actions.stopAll()
            )

        driver
            .y()
            .whileTrue(
                actions.prepShotFromAnywhere { 1.3 }
                    .andThen(
                        robotContainer.drive.xLock()
                            .alongWith(actions.checkAndFeed())
                            .alongWith(WaitCommand(1.0).andThen(actions.shuffleIntakePivot()))
                    )
            )
            .onFalse(
                actions.stopAll()
            )

        driver
            .b()
            .whileTrue(
                actions.prepShotFromAnywhere { 2.92 }
                    .andThen(
                        robotContainer.drive.xLock()
                            .alongWith(actions.checkAndFeed())
                            .alongWith(WaitCommand(1.0).andThen(actions.shuffleIntakePivot()))
                    )
            )
            .onFalse(
                actions.stopAll()
            )

        driver
            .povDown()
            .whileTrue(
                actions.reverseAll(),
            )
            .onFalse(
                actions.stopAll()
            )

        driver
            .povLeft()
            .onTrue(
                actions.stopAllAndHomeHood()
            )

        driver
            .start()
            .onTrue(
                robotContainer.drive.seedFieldCentric()
            )

        shooterJamTrigger
            .onTrue(
                SequentialCommandGroup(
                    InstantCommand({ driver.setRumble(GenericHID.RumbleType.kBothRumble, 1.0) }),
                    actions.autoUnjam(),
                    InstantCommand({ driver.setRumble(GenericHID.RumbleType.kBothRumble, 0.0) }),
                    PrintCommand("RUMBLE COMPLETE."),
                    actions.checkAndFeed()
                )
            )
    }
}
