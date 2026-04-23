package frc.team449

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.commands.AimAtTargetCommand
import frc.team449.commands.SwerveRequestCommand
import frc.team449.util.FieldUtil
import kotlin.math.abs

class Bindings(
    val robotContainer: RobotContainer
) {
    val driver = robotContainer.driveController
    val operator = robotContainer.operatorController
    val actions = robotContainer.actions

    val joysticksMovedPastDeadband: Trigger =
        Trigger {
            abs(driver.leftY) > Constants.DriveConstants.INTERRUPT_DEADBAND ||
                abs(driver.leftX) > Constants.DriveConstants.INTERRUPT_DEADBAND ||
                abs(driver.rightX) > Constants.DriveConstants.INTERRUPT_DEADBAND
        }.debounce(0.1)

    val driverIdle: Trigger =
        Trigger {
            abs(driver.leftY) < Constants.DriveConstants.TRANSLATION_DEADBAND &&
                abs(driver.leftX) < Constants.DriveConstants.TRANSLATION_DEADBAND &&
                abs(driver.rightX) < Constants.DriveConstants.ANGULAR_DEADBAND
        }.debounce(0.1)

    private fun createAutoAimCommand(): AimAtTargetCommand =
        AimAtTargetCommand(
            robotContainer.drive,
            robotContainer.shooter,
            { -driver.leftY },
            { -driver.leftX },
            targetSupplier = { FieldUtil.HUB },
        )

    private fun createAutoPassCommand(): AimAtTargetCommand =
        AimAtTargetCommand(
            robotContainer.drive,
            robotContainer.shooter,
            { -driver.leftY },
            { -driver.leftX },
            targetSupplier = { FieldUtil.getClosestFriendlyPass(robotContainer.drive.pose.translation) },
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
                actions.deployAndIntake(),
            ).onFalse(
                actions.stopIntakeAndPivot(),
            )

        driver
            .leftTrigger()
            .onTrue(
                actions.stopAndStow(),
            )

        driver
            .rightBumper()
            .whileTrue(
                Commands.defer({
                    val autoAimCommand = createAutoAimCommand()
                    Commands.parallel(
                        autoAimCommand,
                        Commands.sequence(
                            Commands.waitUntil { autoAimCommand.atHeadingSetpoint() },
                            actions.checkAndFeed().andThen(actions.tuckAndClear())
                        )
                    )
                }, setOf(robotContainer.drive, robotContainer.shooter, robotContainer.indexer, robotContainer.intake))
                    .withName("AutoAim")
            ).onFalse(
                actions.stopAll(),
            )

        driver
            .leftBumper()
            .whileTrue(
                Commands.defer({
                    val autoPassCommand = createAutoPassCommand()
                    Commands.parallel(
                        autoPassCommand,
                        Commands.sequence(
                            Commands.waitUntil { autoPassCommand.atHeadingSetpoint() },
                            actions.checkAndFeed().andThen(actions.tuckAndClear())
                        )
                    )
                }, setOf(robotContainer.drive, robotContainer.shooter, robotContainer.indexer, robotContainer.intake))
                    .withName("AutoPass")
            ).onFalse(
                actions.stopAllAndZeroHood(),
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
                actions
                    .prepShotFromDistanceMeters(3.43)
                    .andThen(
                        robotContainer.drive
                            .xLock()
                            .alongWith(actions.checkAndFeed().andThen(actions.tuckAndClear()))
                    )
                    .withName("TrenchShot"),
            ).onFalse(
                actions.stopAll(),
            )

        driver
            .y()
            .whileTrue(
                actions
                    .prepShotFromDistanceMeters(1.4)
                    .andThen(
                        robotContainer.drive
                            .xLock()
                            .alongWith(actions.checkAndFeed().andThen(actions.tuckAndClear()))
                    )
                    .withName("HubShot"),
            ).onFalse(
                actions.stopAll(),
            )

        driver
            .b()
            .whileTrue(
                actions
                    .prepShotFromDistanceMeters(2.92)
                    .andThen(
                        robotContainer.drive
                            .xLock()
                            .alongWith(actions.checkAndFeed().andThen(actions.tuckAndClear()))
                    )
                    .withName("TowerShot"),
            ).onFalse(
                actions.stopAll(),
            )

        driver
            .povDown()
            .whileTrue(
                actions.reverseAll(),
            ).onFalse(
                actions.stopAll(),
            )

        driver
            .povLeft()
            .onTrue(
                actions.stopAllAndHomeHood(),
            )

        driver
            .povUp()
            .whileTrue(
                actions.checkAndFeed()
            )

        driver
            .start()
            .onTrue(
                robotContainer.drive.seedFieldCentric(),
            )
    }
}
