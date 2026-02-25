package frc.team449

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team449.Constants.BLUE_GOAL_TRANSLATION
import frc.team449.Constants.RED_GOAL_TRANSLATION
import frc.team449.RobotContainer.drive
import frc.team449.commands.AimAtTargetCommand
import frc.team449.commands.PoseAlignCommand
import frc.team449.commands.SmartXLockCommand
import frc.team449.commands.SwerveRequestCommand
import frc.team449.util.FieldUtil
import java.util.function.Supplier
import kotlin.math.PI

class Bindings(
    val robotContainer: RobotContainer
) {
    val driver = robotContainer.driveController
    val operator = robotContainer.opController
    val actions = robotContainer.actions

    var hubPosition = RED_GOAL_TRANSLATION
    val robotHubDistanceSupplier: Supplier<Double> = { (robotContainer.drive.pose.translation - hubPosition).norm }

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
        // controls for simulation
    }

    fun setHubPosition() {
        hubPosition = if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) RED_GOAL_TRANSLATION else BLUE_GOAL_TRANSLATION
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

        // shoot from anywhere (not sure yet how this will get binded)
        operator.a().onTrue(
            Commands.parallel(
                AimAtTargetCommand(
                    robotContainer.drive,
                    { -robotContainer.driveController.leftY },
                    { -robotContainer.driveController.leftX },
                    { hubPosition }
                ),
                actions.prepShotFromAnywhere(
                    robotHubDistanceSupplier
                ),
                actions.checkAndFeed()
            )
        ).onFalse(
            Commands.sequence(
                robotContainer.shooter.stopFlywheel(),
                robotContainer.indexer.stop()
            )
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
                ).finallyDo { end ->
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
