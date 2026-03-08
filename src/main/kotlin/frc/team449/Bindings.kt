package frc.team449

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.commands.SwerveRequestCommand
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
    val shooterJamTrigger = robotContainer.shooter.shooterJamTrigger

    fun setDefaultCommands() {
        // set default commands for systems here
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
                actions.deployAndRunIntake(),
            ).onFalse(
                SequentialCommandGroup(
                    actions.stopIntake(),
                    actions.stopFeed(),
                ),
            )

        driver
            .leftTrigger()
            .onTrue(
                actions.stopAndStow(),
            )

//        driver
//            .rightBumper()
//            .whileTrue(
//                ParallelCommandGroup(
//                    AimAtTargetCommand(
//                        robotContainer.drive,
//                        { -robotContainer.driveController.leftY },
//                        { -robotContainer.driveController.leftX },
//                        { FieldUtil.HUB_TRANSLATION },
//                    ),
//                    actions.prepShotFromAnywhere { FieldUtil.HUB_TRANSLATION.getDistance(robotContainer.drive.pose.translation) },
//                    actions.checkAndFeed(),
//                ),
//            ).onFalse(
//                actions.stopFeedAndShooter(),
//            )

        driver
            .rightBumper()
            .whileTrue(
                SequentialCommandGroup(
                    actions.checkAndFeed(),
                    actions.shuffleIntakeRoller(),
                ),
            ).onFalse(
                SequentialCommandGroup(
                    actions.stopFeed(),
                    actions.stopIntake(),
                ),
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
                    .until(joysticksMovedPastDeadbandTrigger),
            )

        driver
            .y()
            .onTrue(
                actions.prepHubShot(),
            )

//        driver
//            .x()
//            .onTrue(
//                SequentialCommandGroup(
//                    actions.prepTrenchShot(),
//                    PoseAlignCommand(
//                        robotContainer.drive,
//                        { FieldUtil.getClosestTrenchPose(robotContainer.drive.pose) },
//                        driver.povUp()
//                    ),
//                    actions.checkAndFeed(),
//                    ParallelCommandGroup(
//                        robotContainer.drive.xLock(),
//                        actions.shuffleIntakePivot()
//                    ),
//                ).until(joysticksMovedPastDeadband)
//                    .finallyDo { _ -> CommandScheduler.getInstance().schedule(actions.stopFeedAndShooter(), actions.stopAndStow()) },
//            )

        driver
            .x()
            .onTrue(
                actions.prepTrenchShot(),
            )

        driver
            .b()
            .onTrue(
                actions.prepTowerShot(),
            )

//        driver
//            .b()
//            .onTrue(
//                SequentialCommandGroup(
//                    actions.prepTowerShot(),
//                    PoseAlignCommand(
//                        robotContainer.drive,
//                        { FieldUtil.TOWER_POSE },
//                        driver.povUp()
//                    ),
//                    actions.checkAndFeed(),
//                    ParallelCommandGroup(
//                        robotContainer.drive.xLock(),
//                        actions.shuffleIntakePivot()
//                    ),
//                ).until(joysticksMovedPastDeadband)
//                    .finallyDo { _ -> CommandScheduler.getInstance().schedule(actions.stopFeedAndShooter(), actions.stopAndStow()) },
//            )

        driver
            .povDown()
            .onTrue(
                actions.outtakeIntakeAndReverseIndex(),
            ).onFalse(
                actions.stopIntake(),
            )

        driver
            .povRight()
            .onTrue(
                robotContainer.shooter.stopFlywheel()
            )

        driver
            .start()
            .onTrue(
                robotContainer.drive.seedFieldCentric(),
            )

        shooterJamTrigger
            .onTrue(
                actions.autoUnjam(),
            )

        operator
            .leftBumper()
            .whileTrue(
                actions.outtakeIntakeAndReverseIndex(),
            )

        operator
            .a()
            .onTrue(
                actions.systemCheckCommand(),
            )

        operator
            .rightTrigger()
            .onTrue(
                actions.deployAndRunIntake(),
            )

        operator
            .leftTrigger()
            .onTrue(
                actions.stopAndStow(),
            )

        operator
            .rightBumper()
            .onTrue(
                actions.checkAndFeed(),
            )

        operator
            .x()
            .onTrue(
                robotContainer.shooter.setFlywheelVelocity(Constants.ShooterConstants.TRENCH_FLYWHEEL_VEL),
            )

        operator
            .y()
            .onTrue(
                robotContainer.shooter.setFlywheelVelocity(Constants.ShooterConstants.TEST_FLYWHEEL_VEL),
            )

        operator
            .povDown()
            .onTrue(
                actions.homeHood(),
            )

        operator
            .povLeft()
            .onTrue(
                robotContainer.shooter.setHoodAngle(Constants.ShooterConstants.TOWER_HOOD_ANGLE),
            )

        operator
            .povUp()
            .onTrue(
                robotContainer.shooter.setHoodAngle(Constants.ShooterConstants.TRENCH_HOOD_ANGLE),
            )

        operator
            .povRight()
            .onTrue(
                robotContainer.shooter.setHoodAngle(Constants.ShooterConstants.MAX_HOOD_ANGLE),
            )

        operator
            .povDownRight()
            .onTrue(
                actions.stopFeedAndShooter(),
            )
    }
}
