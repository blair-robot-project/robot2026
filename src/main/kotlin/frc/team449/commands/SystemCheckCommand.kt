package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.Constants.ShooterConstants
import frc.team449.Constants.ShooterConstants.HOOD_TOLERANCE_RAD
import frc.team449.RobotContainer

class SystemCheckCommand(
    private val robotContainer: RobotContainer
) : SequentialCommandGroup() {
    private val driveRequest = SwerveRequest.RobotCentric()

    init {
        addRequirements(
            robotContainer.drive,
            robotContainer.intake,
            robotContainer.indexer,
            robotContainer.shooter,
        )

        addCommands(
            robotContainer.drive.runOnce {
                robotContainer.drive.setControl(
                    driveRequest.withVelocityX(1.0).withVelocityY(0.0),
                )
            },
            WaitCommand(0.75),
            robotContainer.drive.runOnce {
                robotContainer.drive.setControl(
                    driveRequest.withVelocityX(0.0).withVelocityY(1.0),
                )
            },
            WaitCommand(0.75),
            robotContainer.drive.runOnce {
                robotContainer.drive.setControl(
                    driveRequest.withVelocityX(-1.0).withVelocityY(0.0),
                )
            },
            WaitCommand(0.75),
            robotContainer.drive.runOnce {
                robotContainer.drive.setControl(
                    driveRequest.withVelocityX(0.0).withVelocityY(-1.0),
                )
            },
            WaitCommand(0.75),
            robotContainer.drive.runOnce {
                robotContainer.drive.setControl(
                    SwerveRequest.SwerveDriveBrake(),
                )
            },
            // -----------INTAKE-----------
            robotContainer.intake.deploy(),
            robotContainer.intake
                .intake()
                .alongWith(
                    Commands
                        .waitUntil { robotContainer.intake.pivotAtTolerance() && robotContainer.intake.rollerAtTolerance() }
                        .withTimeout(
                            0.5,
                        ),
                    Commands.runOnce({ Alert("BAD PIVOT DEPLOY", Alert.AlertType.kError).set(!robotContainer.intake.pivotAtTolerance()) }),
                    Commands.runOnce({ Alert("BAD ROLLER", Alert.AlertType.kError).set(!robotContainer.intake.rollerAtTolerance()) }),
                ).withTimeout(2.0),
            robotContainer.intake.stopRollers().withTimeout(0.1),
            robotContainer.intake.stow(),
            Commands.waitUntil { robotContainer.intake.pivotAtTolerance() }.withTimeout(0.5),
            Commands.runOnce({ Alert("BAD PIVOT STOW", Alert.AlertType.kError).set(!robotContainer.intake.pivotAtTolerance()) }),
            // -----------INDEXER-----------
            robotContainer.indexer.index(12.0, 3.0, 12.0).withTimeout(2.0).alongWith(
                Commands.waitUntil { robotContainer.indexer.indexerAtTolerance() }.withTimeout(0.5),
                Commands.runOnce({ Alert("BAD INDEXER", Alert.AlertType.kError).set(!robotContainer.indexer.indexerAtTolerance()) }),
            ),
            robotContainer.indexer.stop().withTimeout(0.1),
            // -----------SHOOTER---------
            robotContainer.shooter.homeHood(),
            Commands.waitUntil { robotContainer.shooter.hoodAngle == 0.0 }.withTimeout(0.5),
            Commands.runOnce(
                { Alert("DID NOT HOME HOOD", Alert.AlertType.kError).set(robotContainer.shooter.hoodAngle >= HOOD_TOLERANCE_RAD) },
            ),
            robotContainer.shooter.setHoodAngle(ShooterConstants.TRENCH_HOOD_ANGLE).withTimeout(1.0),
            robotContainer.shooter
                .setFlywheelVelocity(ShooterConstants.TRENCH_FLYWHEEL_VEL)
                .alongWith(
                    Commands
                        .waitUntil { robotContainer.shooter.isFlywheelAtTolerance() && robotContainer.shooter.isHoodAtTolerance() }
                        .withTimeout(
                            0.75,
                        ),
                    Commands.runOnce(
                        { Alert("BAD TRENCH FLYWHEEL", Alert.AlertType.kError).set(!robotContainer.shooter.isFlywheelAtTolerance()) },
                    ),
                    Commands.runOnce(
                        { Alert("BAD TRENCH HOOD ANGLE", Alert.AlertType.kError).set(!robotContainer.shooter.isHoodAtTolerance()) },
                    ),
                ).withTimeout(2.0),
            robotContainer.shooter.setHoodAngle(ShooterConstants.MAX_HOOD_ANGLE).withTimeout(0.8),
            robotContainer.shooter
                .setFlywheelVelocity(ShooterConstants.HUB_FLYWHEEL_VEL)
                .alongWith(
                    Commands
                        .waitUntil { robotContainer.shooter.isHoodAtTolerance() && robotContainer.shooter.isFlywheelAtTolerance() }
                        .withTimeout(
                            0.75,
                        ),
                    Commands.runOnce(
                        { Alert("BAD MAX HOOD ANGLE", Alert.AlertType.kError).set(!robotContainer.shooter.isHoodAtTolerance()) },
                    ),
                    Commands.runOnce(
                        { Alert("BAD HUB FLYWHEEL", Alert.AlertType.kError).set(!robotContainer.shooter.isFlywheelAtTolerance()) },
                    ),
                ).withTimeout(2.0),
            robotContainer.shooter.stopFlywheel().withTimeout(0.2),
            robotContainer.shooter.homeHood().withTimeout(0.5),
        )
    }
}
