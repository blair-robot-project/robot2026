package frc.team449

import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team449.Constants.ShooterConstants
import frc.team449.commands.SwerveRequestCommand

class Bindings(
    val robotContainer: RobotContainer
) {
    val driver = robotContainer.driveController
    val operator = robotContainer.opController

    fun setDefaultCommands() {
        // set default commands for systems here
        robotContainer.drive.defaultCommand =
            SwerveRequestCommand(
                robotContainer.drive,
                { -driver.leftY },
                { -driver.leftX },
                { driver.rightX },
            )
    }

    fun bindControls() {
        driver
            .rightTrigger()
            .onTrue(
                SequentialCommandGroup(
                    robotContainer.intake.deploy(),
                    robotContainer.intake.intake(),
                ),
            )

        driver
            .leftTrigger()
            .onTrue(
                SequentialCommandGroup(
                    robotContainer.intake.stopRollers(),
                    robotContainer.intake.stow(),
                ),
            )

        driver
            .rightBumper()
            .whileTrue(
                robotContainer.shooter.setFlywheelVelocity(ShooterConstants.HUB_FLYWHEEL_VEL),
                // check tol and feed
            ).onFalse(
                robotContainer.shooter.stopFlywheel(),
                // coast hopper
            )
//
//        driver
//            .x()
//            .onTrue(
//                SequentialCommandGroup(
//                    ParallelCommandGroup(
//                        PoseAlignCommand(
//                            robotContainer.drive,
//                            { Field.getClosestTrenchPose(robotContainer.drive.pose) },
//                            { -driver.leftY },
//                            { -driver.leftX },
//                            { driver.rightX },
//                        ),
//                        robotContainer.shooter.setFlywheelVelocity(ShooterConstants.TRENCH_FLYWHEEL_VEL),
//                    ),
//                ),
//                // feed
//            )

//        driver
//            .a()
//            .onTrue(
//                SmartXLockCommand(
//                    robotContainer.drive,
//                    { -driver.leftY },
//                    { -driver.leftX },
//                    { driver.rightX },
//                )
//            )

        driver
            .start()
            .onTrue(
                robotContainer.drive.seedFieldCentric(),
            )

        driver
            .b()
            .onTrue(
                robotContainer.shooter.setHoodAngle(ShooterConstants.MAX_HOOD_ANGLE),
            )
//            .onFalse(
//                robotContainer.shooter.setHoodAngle(ShooterConstants.MIN_HOOD_ANGLE),
//            )

        driver
            .a()
            .onTrue(
                robotContainer.intake.deploy(),
            ).onFalse(
                robotContainer.intake.stow(),
            )

        driver
            .y()
            .onTrue(
                robotContainer.indexer.index(RadiansPerSecond.of(3.0)),
            ).onFalse(
                robotContainer.indexer.stop(),
            )

        driver
            .x()
            .onTrue(
                robotContainer.shooter.setFlywheelVelocity(RadiansPerSecond.of(130.0)),
            ).onFalse(robotContainer.shooter.stopFlywheel())
    }
}
