package frc.team449

import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team449.commands.PoseAlignCommand
import frc.team449.commands.SmartXLockCommand
import frc.team449.commands.SwerveRequestCommand
import frc.team449.util.FieldUtil
import kotlin.math.abs

class Bindings(
    val robotContainer: RobotContainer
) {
    val driver = robotContainer.driveController
    val operator = robotContainer.opController
    val actions = robotContainer.actions

    fun setDefaultCommands() {
        // set default commands for systems here
        robotContainer.drive.defaultCommand =
            SwerveRequestCommand(
                robotContainer.drive,
                { -driver.leftY },
                { -driver.leftX },
                { -driver.rightX },
                Constants.DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND,
                Constants.DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND
            )
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
            .onTrue(
                SwerveRequestCommand(
                    robotContainer.drive,
                    { -driver.leftY },
                    { -driver.leftX },
                    { -driver.rightX },
                    Constants.DriveConstants.SLOW_LINEAR_SPEED_METERS_PER_SECOND,
                    Constants.DriveConstants.SLOW_ANGULAR_SPEED_RADIANS_PER_SECOND,
                )
            )
            .onFalse(
                SwerveRequestCommand(
                    robotContainer.drive,
                    { -driver.leftY },
                    { -driver.leftX },
                    { -driver.rightX },
                    Constants.DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND,
                    Constants.DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND
                )
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
            .b()
            .onTrue(
                actions.prepTowerShot()
            )

        driver
            .x()
            .onTrue(
                SequentialCommandGroup(
                    ParallelCommandGroup(
                        PoseAlignCommand(
                            robotContainer.drive,
                            { FieldUtil.getClosestTrenchPose(robotContainer.drive.pose) },
                            { -driver.leftY },
                            { -driver.leftX },
                            { -driver.rightX },
                        ),
                        actions.prepTrenchShot()
                    ),
                    actions.checkAndFeed(),
                    SmartXLockCommand(
                        robotContainer.drive,
                        { -driver.leftY },
                        { -driver.leftX },
                        { driver.rightX },
                    )
                ).until {
                    val xLockDeadband = Constants.DriveConstants.X_LOCK_DEADBAND
                    abs(driver.leftY) > xLockDeadband ||
                        abs(driver.leftX) > xLockDeadband ||
                        abs(driver.rightX) > xLockDeadband
                }.finallyDo { end ->
                    CommandScheduler.getInstance().schedule(
                        actions.stopShooter()
                    )
                }
            )

        driver
            .y()
            .onTrue(
                actions.prepHubShot()
            )

        driver
            .povUp()
            .onTrue(
                actions.stopShooter()
            )

        driver
            .start()
            .onTrue(
                robotContainer.drive.seedFieldCentric(),
            )
    }
}
