package frc.team449

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team449.Constants.ShooterConstants
import frc.team449.RobotContainer.drive
import frc.team449.commands.PoseAlignCommand
import frc.team449.commands.SwerveRequestCommand
import frc.team449.util.Field

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
                    robotContainer.intake.intake()
                )
            )

        driver
            .leftTrigger()
            .onTrue(
                SequentialCommandGroup(
                    robotContainer.intake.stopRollers(),
                    robotContainer.intake.stow()
                )
            )

        driver
            .rightBumper()
            .whileTrue(
                robotContainer.shooter.setFlywheelVelocity(ShooterConstants.HUB_FLYWHEEL_VEL)
                // check tol and feed
            )
            .onFalse(
                robotContainer.shooter.stopFlywheel()
                // coast hopper
            )

        driver
            .x()
            .onTrue(
                SequentialCommandGroup(
                    ParallelCommandGroup(
                        PoseAlignCommand(
                            robotContainer.drive,
                            { Field.getClosestTrenchPose(robotContainer.drive.pose) },
                            { -driver.leftY },
                            { -driver.leftX },
                            { driver.rightX },
                        ),
                        robotContainer.shooter.setFlywheelVelocity(ShooterConstants.TRENCH_FLYWHEEL_VEL)
                    )
                )
                // feed
            )

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

      //  driver
         //   .start()
       //     .onTrue(
      //          robotContainer.drive.seedFieldCentric()
      //      )

        driver
            .b()
            .onTrue(
                robotContainer.shooter.setHoodAngle(ShooterConstants.MAX_HOOD_ANGLE)
            )
            .onFalse(
                robotContainer.shooter.setHoodAngle(ShooterConstants.MIN_HOOD_ANGLE)
            )

        driver
            .a()
            .onTrue(
                robotContainer.intake.deploy()
            )
            .onFalse(
                robotContainer.intake.stow()
            )
    }
}
