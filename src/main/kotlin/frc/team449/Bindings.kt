package frc.team449

import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.Constants.IndexerConstants
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

        robotContainer.indexer.defaultCommand = robotContainer.indexer.stop()
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
                    ParallelCommandGroup(
                        robotContainer.drive.xLock(),
                        actions.shuffleIntakePivot()
                    ),
                ).until(joysticksMovedPastDeadband)
                    .finallyDo { _ -> CommandScheduler.getInstance().schedule(actions.stopFeedAndShooter(), actions.stopAndStow()) },
            )

        driver
            .b()
            .onTrue(
                SequentialCommandGroup(
                    actions.prepTowerShot(),
                    PoseAlignCommand(
                        robotContainer.drive,
                    ) { FieldUtil.TOWER_POSE },
                    actions.checkAndFeed(),
                    ParallelCommandGroup(
                        robotContainer.drive.xLock(),
                        actions.shuffleIntakePivot()
                    ),
                ).until(joysticksMovedPastDeadband)
                    .finallyDo { _ -> CommandScheduler.getInstance().schedule(actions.stopFeedAndShooter(), actions.stopAndStow()) },
            )

        driver
            .povUp()
            .onTrue(
                actions.prepHubShot(),
            )

        driver
            .povLeft()
            .onTrue(
                actions.prepTrenchShot(),
            )

        driver
            .povRight()
            .onTrue(
                actions.prepTowerShot()
            )

        driver
            .start()
            .onTrue(
                robotContainer.drive.seedFieldCentric(),
            )

        driver
            .povDown()
            .whileTrue(
                robotContainer.indexer.index(IndexerConstants.SHOOTING_INDEXER_SPEED),
            )

        driver
            .povDownRight()
            .onTrue(
                actions.stopFeedAndShooter(),
            )

        operator
            .a()
            .onTrue(
                actions.systemCheckCommand(),
            )
    }
}
