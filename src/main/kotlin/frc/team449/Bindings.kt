package frc.team449

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.Constants.ShooterConstants
import frc.team449.commands.AimAtTargetCommand
import frc.team449.commands.StatorStowCommand
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
            robotContainer.indexer,
            robotContainer.shooter,
            { -driver.leftY },
            { -driver.leftX },
            { FieldUtil.HUB },
        )

    private fun createAutoPassCommand(): AimAtTargetCommand =
        AimAtTargetCommand(
            robotContainer.drive,
            robotContainer.indexer,
            robotContainer.shooter,
            { -driver.leftY },
            { -driver.leftX },
            { FieldUtil.getClosestFriendlyPass(robotContainer.drive.pose.translation) },
            isScoring = false,
            flywheelVelocityMap = ShooterConstants.PASSING_FLYWHEEL_VELOCITY_MAP,
            hoodAngleMap = ShooterConstants.PASSING_HOOD_ANGLE_MAP,
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
                    val statorStowCommand = StatorStowCommand(robotContainer.intake) { autoAimCommand.readyToShoot() }
                    Commands.parallel(
                        autoAimCommand,
                        statorStowCommand,
                    )
                }, setOf(robotContainer.drive, robotContainer.shooter, robotContainer.indexer, robotContainer.intake))
                    .withName("AutoAim")
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
            ).onFalse(
                actions.stopShooterIndexer(),
            )

        driver
            .leftBumper()
            .whileTrue(
                Commands.defer({
                    val autoPassCommand = createAutoPassCommand()
                    autoPassCommand
                }, setOf(robotContainer.drive, robotContainer.shooter, robotContainer.indexer))
                    .withName("AutoPass")
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
            ).onFalse(
                Commands.sequence(
                    actions.stopShooterIndexer(),
                    robotContainer.shooter.setHoodAngle(Units.Radians.of(0.0))
                )
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
                robotContainer.indexer.setIndexerVoltage(12.0, 12.0)
            )
            .onFalse(
                robotContainer.indexer.stop()
            )

        driver
            .povRight()
            .whileTrue(
                robotContainer.shooter.setFlywheelVelocity(Units.RadiansPerSecond.of(100.0))
            )
            .onFalse(
                robotContainer.shooter.stopFlywheel()
            )

        driver
            .start()
            .onTrue(
                robotContainer.drive.seedFieldCentric(),
            )
    }
}
