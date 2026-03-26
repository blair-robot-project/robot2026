package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.Constants.ShooterConstants
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
                    driveRequest.withVelocityX(1.0).withVelocityY(0.0)
                )
            },
            WaitCommand(0.75),
            robotContainer.drive.runOnce {
                robotContainer.drive.setControl(
                    driveRequest.withVelocityX(0.0).withVelocityY(1.0)
                )
            },
            WaitCommand(0.75),
            robotContainer.drive.runOnce {
                robotContainer.drive.setControl(
                    driveRequest.withVelocityX(-1.0).withVelocityY(0.0)
                )
            },
            WaitCommand(0.75),
            robotContainer.drive.runOnce {
                robotContainer.drive.setControl(
                    driveRequest.withVelocityX(0.0).withVelocityY(-1.0)
                )
            },
            WaitCommand(0.75),
            robotContainer.drive.runOnce {
                robotContainer.drive.setControl(
                    SwerveRequest.SwerveDriveBrake()
                )
            },

            robotContainer.intake.intake(),
            robotContainer.intake.deploy(),
            WaitCommand(1.5),
            robotContainer.intake.stopRollers(),
            robotContainer.intake.stow(),

            robotContainer.indexer.index(
                12.0,
                1.0,
                12.0
            ),
            WaitCommand(1.0),
            robotContainer.indexer.stop(),

            robotContainer.shooter.setFlywheelVelocity(ShooterConstants.TRENCH_FLYWHEEL_VEL),
            robotContainer.shooter.homeHood(),
            robotContainer.shooter.setHoodAngle(ShooterConstants.TOWER_HOOD_ANGLE),
            WaitCommand(0.75),
            robotContainer.shooter.setHoodAngle(ShooterConstants.TRENCH_HOOD_ANGLE),
            WaitCommand(0.75),
            robotContainer.shooter.setHoodAngle(ShooterConstants.MAX_HOOD_ANGLE),
            WaitCommand(0.75),
            robotContainer.shooter.stopFlywheel(),
            robotContainer.shooter.homeHood()
        )
    }
}
