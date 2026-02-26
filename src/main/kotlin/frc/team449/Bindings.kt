package frc.team449

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.commands.AimAtTargetCommand
import frc.team449.commands.PoseAlignCommand
import frc.team449.commands.SwerveRequestCommand
import frc.team449.commands.XLockCommand
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
                { -driver.rightX }
            )
        // controls for simulation
    }

    fun bindControls() {
        driver
            .rightTrigger()
            .onTrue(
                actions.deployAndToggleIntake()
            )

        driver
            .leftTrigger()
            .onTrue(
                actions.stopAndStow()
            )

        driver
            .rightBumper()
            .whileTrue(
                actions.checkAndFeed()
            )
            .onFalse(
                actions.stopFeed()
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
                )
            )

        driver
            .a()
            .onTrue(
                XLockCommand(
                    robotContainer.drive
                )
                    .until(joysticksMovedPastDeadband)
            )

        driver
            .b()
            .onTrue(
                actions.prepTowerShot()
            )

        driver
            .x()
            .onTrue(
                SequentialCommandGroup(
                    actions.prepTrenchShot(),
                    PoseAlignCommand(
                        robotContainer.drive
                    ) { FieldUtil.getClosestTrenchPose(robotContainer.drive.pose) },
                    actions.checkAndFeed(),
                    XLockCommand(
                        robotContainer.drive
                    )
                )
                    .until(joysticksMovedPastDeadband)
                    .finallyDo { _ -> CommandScheduler.getInstance().schedule(actions.stopFeedAndShooter()) }
            )

        driver
            .y()
            .onTrue(
                actions.prepHubShot()
            )

        driver
            .povUp()
            .onTrue(
                actions.stopFeedAndShooter()
            )

        driver
            .start()
            .onTrue(
                robotContainer.drive.seedFieldCentric(),
            )

        operator.a().onTrue(
            ParallelCommandGroup(
                AimAtTargetCommand(
                    robotContainer.drive,
                    { -robotContainer.driveController.leftY },
                    { -robotContainer.driveController.leftX },
                    { FieldUtil.HUB_TRANSLATION },
                ).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf), // redundant but just doing this to be clear
                actions.prepShotFromAnywhere { FieldUtil.HUB_TRANSLATION.getDistance(robotContainer.drive.pose.translation) },
                actions.checkAndFeed()
            )
        ).onFalse(
            Commands.sequence(
                actions.stopFeedAndShooter(),
                runOnce({
                    CommandScheduler.getInstance().schedule(
                        SwerveRequestCommand(
                            robotContainer.drive,
                            { -robotContainer.driveController.leftY },
                            { -robotContainer.driveController.leftX },
                            { -driver.rightX }
                        )
                    )
                })
            )

        )
    }
}
