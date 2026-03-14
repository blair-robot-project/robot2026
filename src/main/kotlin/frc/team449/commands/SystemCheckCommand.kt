package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.Constants
import frc.team449.Constants.IntakeConstants
import frc.team449.Constants.ShooterConstants
import frc.team449.RobotContainer
import kotlin.math.abs

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
            Commands.runOnce({ Alert("BAD PIVOT", Alert.AlertType.kError) }).onlyIf { verifyPivot() },
            Commands.runOnce({ Alert("BAD ROLLER", Alert.AlertType.kError) }).onlyIf { verifyRoller() },
            robotContainer.intake.stopRollers(),
            robotContainer.intake.stow(),
            Commands.runOnce({ Alert("BAD PIVOT", Alert.AlertType.kError) }).onlyIf { verifyPivot() },

            robotContainer.indexer.index(Constants.IndexerConstants.SHOOTING_INDEXER_SPEED),
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

    // Checks Left Pivot Leader to see if it's in a tolerable range
    fun verifyPivot(): Boolean {
        val target: Double =
            if (robotContainer.intake.pivotIsDeployed) {
                IntakeConstants.DEPLOY_POS_RADS
            } else {
                IntakeConstants.STOW_POS_RADS
            }
        return abs(robotContainer.intake.intakeSimAngle - target) <=
            0.175 // Rads ~ 10 degrees
    }

    // Checks the rollerVelocity is in a tolerable range
    fun verifyRoller(): Boolean =
        abs(robotContainer.intake.rollerVelocityRadPerSec - robotContainer.intake.rollerTargetVelocityRadPerSec) <=
            20 // Rad/s

}
