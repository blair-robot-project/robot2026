package frc.team449

import edu.wpi.first.units.Units.RPM
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
                { -driver.getRawAxis(2) },
            )
    }

    fun bindControls() {
        driver
            .a()
            .onTrue(
                SequentialCommandGroup(
                    robotContainer.intake.deploy(),
                    robotContainer.intake.intake(),
                ),
            )

        driver
            .b()
            .onTrue(
                SequentialCommandGroup(
                    robotContainer.intake.stopRollers(),
                    robotContainer.intake.stow(),
                ),
            )

        driver
            .x()
            .whileTrue(
                robotContainer.shooter.setFlywheelVelocity(RPM.of(3000.0)).andThen(robotContainer.shooter.setHoodAngle(
                    ShooterConstants.MIN_HOOD_ANGLE)),
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
//                            { -driver.rightX },
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
    }
}
