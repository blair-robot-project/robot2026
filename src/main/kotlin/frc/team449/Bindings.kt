package frc.team449

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team449.Constants.ShooterConstants
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ConditionalCommand
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
    var aimbotting = false
    val HUB_POSITION = if (DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) Constants.RED_GOAL_POSE else Constants.BLUE_GOAL_POSE
    val ROBOT_DISTANCE_FROM_HUB : Supplier<Double> = { (robotContainer.drive.pose.translation - HUB_POSITION.translation).norm }

    fun setDefaultCommands() {
        // set default commands for systems here
        robotContainer.drive.defaultCommand =
            SwerveRequestCommand(
                robotContainer.drive,
                { -driver.leftY },
                { -driver.leftX },
                { driver.rightX },
            )
        shootFromAnywhere()
        // controls for simulation
    }

    fun shootFromAnywhere() {
        driveController.a().onTrue (
            runOnce ({
                if (aimbotting) {
                    aimbotting = false
                    CommandScheduler.getInstance().schedule(drive.defaultCommand)
                } else {
                    aimbotting = true
                    CommandScheduler.getInstance().schedule(
                        AimAtTargetCommand(
                            robotContainer.drive,
                            { -robotContainer.driveController.leftY },
                            { -robotContainer.driveController.leftX },
                            {HUB_POSITION},
                        )
                    )
                }
            })
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
                ConditionalCommand(
                    Commands.sequence(
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
                    ),
                    robotContainer.shooter.setFlywheelVelocity(RadiansPerSecond.of(130.0)),
                    { aimbotting }
                )


            ).onFalse(robotContainer.shooter.stopFlywheel())
    }
}
