package frc.team449

import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team449.Constants.ShooterConstants
import frc.team449.RobotContainer.drive
import frc.team449.commands.AimAtTargetCommand
import frc.team449.commands.SwerveRequestCommand
import java.util.function.Supplier
import kotlin.jvm.optionals.getOrDefault

class Bindings(
    val robotContainer: RobotContainer
) {
    val driver = robotContainer.driveController
    val operator = robotContainer.opController
    val driveController = robotContainer.driveController
    val opController = robotContainer.opController
    val HUB_POSITION = if (DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) Constants.RED_GOAL_POSE else Constants.BLUE_GOAL_POSE
    val ROBOT_DISTANCE_FROM_HUB: Supplier<Double> = { (robotContainer.drive.pose.translation - HUB_POSITION.translation).norm }

    fun setDefaultCommands() {
        // set default commands for systems here
        robotContainer.drive.defaultCommand =
            SwerveRequestCommand(
                robotContainer.drive,
                { -driver.leftY },
                { -driver.leftX },
                { driver.rightX },
            )
        // controls for simulation
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

//        driver
//            .rightBumper()
//            .whileTrue(
//                robotContainer.shooter.setFlywheelVelocity(ShooterConstants.HUB_FLYWHEEL_VEL),
//                // check tol and feed
//            ).onFalse(
//                robotContainer.shooter.stopFlywheel(),
//                // coast hopper
//            )

        // shoot from anywhere
        driveController.rightBumper().onTrue(
            Commands.sequence(
                runOnce({
                    CommandScheduler.getInstance().schedule(
                        AimAtTargetCommand(
                            robotContainer.drive,
                            { -robotContainer.driveController.leftY },
                            { -robotContainer.driveController.leftX },
                            { HUB_POSITION },
                        )
                    )
                }),
                robotContainer.shooter.setHoodAngle(
                    Degrees.of(
                        ShooterConstants.HOOD_ANGLE_MAP.get(
                            ROBOT_DISTANCE_FROM_HUB.get()
                        )
                    )
                ),
                robotContainer.shooter.setFlywheelVelocity(
                    RadiansPerSecond.of(
                        ShooterConstants.FLYWHEEL_VELOCITY_MAP.get(
                            ROBOT_DISTANCE_FROM_HUB.get()
                        )
                    )
                )
            )
        ).onFalse(
            runOnce({
                CommandScheduler.getInstance().schedule(drive.defaultCommand)
            })
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
            ).onFalse(
                robotContainer.shooter.setHoodAngle(ShooterConstants.MIN_HOOD_ANGLE),
            )

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
                robotContainer.shooter.setFlywheelVelocity(RadiansPerSecond.of(130.0))
            ).onFalse(robotContainer.shooter.stopFlywheel())
    }
}
