package frc.team449

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.Constants.ShooterConstants
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

    private fun createAutoAimCommand(): AimAtTargetCommand =
        AimAtTargetCommand(
            robotContainer.drive,
            robotContainer.shooter,
            { -driver.leftY },
            { -driver.leftX },
            { FieldUtil.HUB },
        )

    private fun createAutoPassCommand(): AimAtTargetCommand =
        AimAtTargetCommand(
            robotContainer.drive,
            robotContainer.shooter,
            { -driver.leftY },
            { -driver.leftX },
            { FieldUtil.getClosestFriendlyPass(robotContainer.drive.pose.translation) },
            toleranceRadians = 0.15,
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
            .leftTrigger()
            .whileTrue(actions.deployAndIntake())
            .onFalse(actions.stopIntakeAndPivot())

        driver
            .leftBumper()
            .whileTrue(
                actions
                    .prepShotFromDistanceMeters(3.43)
                    .andThen(
                        robotContainer.drive
                            .xLock()
                            .alongWith(actions.checkAndFeed().andThen(actions.tuckAndClear())),
                    ).withName("TrenchShot"),
            ).onFalse(actions.stopAll())

        driver
            .rightTrigger()
            .whileTrue(
                Commands
                    .defer({
                        val autoAimCommand = createAutoAimCommand()
                        Commands.parallel(
                            autoAimCommand,
                            Commands.sequence(
                                Commands.waitUntil { autoAimCommand.readyToShoot() },
                                actions.checkAndFeed(),
                                actions.tuckAndClear().asProxy(),
                            ),
                        )
                    }, setOf(robotContainer.drive, robotContainer.shooter, robotContainer.indexer))
                    .withName("AutoAim")
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming),
            ).onFalse(actions.stopShooterIndexer())

        driver
            .rightBumper()
            .whileTrue(
                actions
                    .prepShotFromDistanceMeters(1.4)
                    .andThen(
                        robotContainer.drive
                            .xLock()
                            .alongWith(actions.checkAndFeed().andThen(actions.tuckAndClear())),
                    ).withName("HubShot"),
            ).onFalse(actions.stopAll())

        driver
            .a()
            .whileTrue(actions.reverseAll())
            .onFalse(actions.stopAll())

        driver
            .b()
            .onTrue(actions.stopAndStow())

        driver
            .x()
            .onTrue(
                robotContainer.drive
                    .xLock()
                    .until(joysticksMovedPastDeadband),
            )

        driver
            .y()
            .whileTrue(
                Commands
                    .defer({
                        val autoPassCommand = createAutoPassCommand()
                        Commands.parallel(
                            autoPassCommand,
                            Commands.sequence(
                                Commands.waitUntil { autoPassCommand.readyToShoot() },
                                actions.checkAndFeed(),
                            ),
                        )
                    }, setOf(robotContainer.drive, robotContainer.shooter, robotContainer.indexer))
                    .withName("AutoPass")
                    .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming),
            ).onFalse(
                Commands.sequence(
                    actions.stopShooterIndexer(),
                    robotContainer.shooter.setHoodAngle(Units.Radians.of(0.0)),
                ),
            )

        driver
            .povUp()
            .onTrue(robotContainer.drive.seedFieldCentric())

        driver
            .povLeft()
            .onTrue(actions.stopAllAndHomeHood())

        driver
            .povRight()
            .whileTrue(
                actions
                    .prepShotFromDistanceMeters(2.92)
                    .andThen(
                        robotContainer.drive
                            .xLock()
                            .alongWith(actions.checkAndFeed().andThen(actions.tuckAndClear())),
                    ).withName("TowerShot"),
            ).onFalse(actions.stopAll())

        driver
            .povDown()
            .whileTrue(robotContainer.indexer.setIndexerVoltage(12.0, 12.0))
            .onFalse(robotContainer.indexer.stop())
    }
}
