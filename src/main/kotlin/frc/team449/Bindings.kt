package frc.team449

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team449.Constants.IndexerConstants
import frc.team449.Constants.ShooterConstants
import frc.team449.commands.AimAtTargetCommand
import frc.team449.commands.PoseAlignCommand
import frc.team449.commands.SwerveRequestCommand
import frc.team449.util.FieldUtil
import kotlin.math.abs

class Bindings(
    val robotContainer: RobotContainer
) {
    val driver = robotContainer.driveController
    val operator = robotContainer.opController
    val actions = robotContainer.actions

    val joysticksMovedPastDeadband: Trigger = Trigger { abs(driver.leftY) > 0.25 || abs(driver.leftX) > 0.25 || abs(driver.rightX) > 0.25 }

    fun setDefaultCommands() {
        // set default commands for systems here
        robotContainer.drive.defaultCommand =
            SwerveRequestCommand(
                robotContainer.drive,
                { -driver.leftY },
                { -driver.leftX },
                { -driver.rightX },
            )
        // controls for simulation
    }

    fun bindControls() {
        driver
            .rightTrigger()
            .onTrue(
                actions.deployAndToggleIntake(),
            )

        driver
            .leftTrigger()
            .onTrue(
                actions.stopAndStow(),
            )

        driver
            .rightBumper()
            .whileTrue(
                ParallelCommandGroup(
                    AimAtTargetCommand(
                        robotContainer.drive,
                        { -robotContainer.driveController.leftY },
                        { -robotContainer.driveController.leftX },
                        { FieldUtil.HUB_TRANSLATION },
                    ),
                    actions.prepShotFromAnywhere { FieldUtil.HUB_TRANSLATION.getDistance(robotContainer.drive.pose.translation) },
                    actions.checkAndFeed(),
                ),
            ).onFalse(
                actions.stopFeedAndShooter(),
            )

        driver
            .leftBumper()
            .whileTrue(
                SwerveRequestCommand(
                    robotContainer.drive,
                    { -driver.leftY },
                    { -driver.leftX },
                    { -driver.rightX },
                    Constants.DriveConstants.SLOW_LINEAR_SPEED_METERS_PER_SECOND,
                    Constants.DriveConstants.SLOW_ANGULAR_SPEED_RADIANS_PER_SECOND,
                ),
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
            .onTrue(
                SequentialCommandGroup(
                    actions.prepTrenchShot(),
                    PoseAlignCommand(
                        robotContainer.drive,
                    ) { FieldUtil.getClosestTrenchPose(robotContainer.drive.pose) },
                    actions.checkAndFeed(),
                    robotContainer.drive.xLock(),
                ).until(joysticksMovedPastDeadband)
                    .finallyDo { _ -> CommandScheduler.getInstance().schedule(actions.stopFeedAndShooter()) },
            )

        // tower sequence on b()

        driver
            .povUp()
            .onTrue(
                actions.prepHubShot()
            )

        driver
            .povLeft()
            .onTrue(
                actions.prepTrenchShot()
            )

        driver
            .povRight()
            .onTrue(
                SequentialCommandGroup(
                    robotContainer.shooter.setHoodAngle(ShooterConstants.MAX_HOOD_ANGLE),
                    robotContainer.shooter.setFlywheelVelocity(Units.RadiansPerSecond.of(280.0))
                )
            )

        driver
            .start()
            .onTrue(
                robotContainer.drive.seedFieldCentric(),
            )

        driver
            .povDown()
            .whileTrue(
                robotContainer.indexer.index(IndexerConstants.SHOOTING_INDEXER_SPEED)
            )
            .onFalse(
                robotContainer.indexer.stop()
            )

        driver
            .povDownRight()
            .onTrue(
                actions.stopFeedAndShooter(),
            )

        operator
            .rightBumper()
            .whileTrue(
                actions.prepTrenchShot(),
            ).onFalse(
                actions.stopFeedAndShooter(),
            )

        operator
            .a()
            .onTrue(
                robotContainer.shooter.setHoodAngle(ShooterConstants.MIN_HOOD_ANGLE),
            )

        operator
            .y()
            .onTrue(
                robotContainer.shooter.setHoodAngle(ShooterConstants.MAX_HOOD_ANGLE),
            )

        operator
            .x()
            .onTrue(
                robotContainer.shooter.setHoodAngle(ShooterConstants.TRENCH_HOOD_ANGLE),
            )

        operator
            .povUp()
            .whileTrue(
                robotContainer.drive.sysIDTranslationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            )

        operator
            .povDown()
            .whileTrue(
                robotContainer.drive.sysIDTranslationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            )
    }
}
