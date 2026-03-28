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

    val joysticksMovedPastDeadband: Trigger =
        Trigger {
            abs(driver.leftY) > 0.25 || abs(driver.leftX) > 0.25 ||
                abs(driver.rightX) > 0.25
        }

    val driverIdle: Trigger = Trigger {
        abs(driver.leftY) < Constants.DriveConstants.TRANSLATION_DEADBAND &&
            abs(driver.leftX) < Constants.DriveConstants.TRANSLATION_DEADBAND &&
            abs(driver.rightX) < Constants.DriveConstants.ANGULAR_DEADBAND
    }.debounce(0.1)

    val shooterJamTrigger: Trigger = robotContainer.shooter.shooterJamTrigger

    val autoAimCommand = AimAtTargetCommand(
        robotContainer.drive,
        robotContainer.shooter,
        { -driver.leftY },
        { -driver.leftX },
        targetSupplier = { FieldUtil.HUB }
    )

    val autoPassCommand = AimAtTargetCommand(
        robotContainer.drive,
        robotContainer.shooter,
        { -driver.leftY },
        { -driver.leftX },
        targetSupplier = { FieldUtil.getClosestFriendlyPass(robotContainer.drive.pose) }
    )

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
                    autoAimCommand.until { autoAimCommand.atHeadingSetpoint() && driverIdle.asBoolean },
                    Commands.parallel(
                        robotContainer.drive.xLock(),
                        actions.checkAndFeed()
                    )
                )
                    .withName("AutoAim")
            )
            .onFalse(
                actions.stopAll()
            )

        driver
            .leftBumper()
            .whileTrue(
                Commands.parallel(
                    autoPassCommand,
                    Commands.sequence(
                        Commands.waitUntil { autoPassCommand.atHeadingSetpoint() },
                        actions.checkAndFeed()
                    )
                )
                    .withName("AutoPass")
            )
            .onFalse(
                actions.stopAll()
            )

        driver
            .a()
            .onTrue(
                robotContainer.drive
                    .xLock()
                    .until(joysticksMovedPastDeadband),
            )

        driver
            .x()
            .whileTrue(
                actions.prepShotFromAnywhere(3.43)
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
                actions.prepShotFromAnywhere(1.3)
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
                actions.prepShotFromAnywhere(2.92)
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
