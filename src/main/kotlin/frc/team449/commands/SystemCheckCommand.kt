package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team449.Constants.ShooterConstants
import frc.team449.RobotContainer
import frc.team449.subsystems.drive.DriveIOInputsAutoLogged

class SystemCheckCommand(
    private val robotContainer: RobotContainer
) : SequentialCommandGroup() {
    private val driveRequest = SwerveRequest.RobotCentric()
    private val inputs: DriveIOInputsAutoLogged = DriveIOInputsAutoLogged()
    private var errors = mutableListOf<String>()
    val hoodToleranceCondition: Boolean
        get() = robotContainer.shooter.isHoodAtTolerance()
    val flywheelToleranceCondition: Boolean
        get() = robotContainer.shooter.isFlywheelAtTolerance()
    val swerveToleranceCondition: Boolean
        get() = robotContainer.drive.isWithinTolerance()
    val indexerToleranceCondition: Boolean
        get() = robotContainer.indexer.indexerAtTolerance()
    val pivotToleranceCondition: Boolean
        get() = robotContainer.intake.pivotAtTolerance()
    val rollerToleranceCondition: Boolean
        get() = robotContainer.intake.rollerAtTolerance()

    init {
        addRequirements(
            robotContainer.drive,
            robotContainer.intake,
            robotContainer.indexer,
            robotContainer.shooter,
        )

        addCommands(
            testSubSystem(
                robotContainer.drive.runOnce {
                    robotContainer.drive.setControl(
                        driveRequest.withVelocityX(1.0).withVelocityY(0.0),
                    )
                },
                swerveToleranceCondition,
                0.5,
                "BAD FORWARD DIRECTION",
            ),
            testSubSystem(
                robotContainer.drive.runOnce {
                    robotContainer.drive.setControl(
                        driveRequest.withVelocityX(0.0).withVelocityY(1.0),
                    )
                },
                swerveToleranceCondition,
                0.5,
                "BAD RIGHT DIRECTION",
            ),
            testSubSystem(
                robotContainer.drive.runOnce {
                    robotContainer.drive.setControl(
                        driveRequest.withVelocityX(-1.0).withVelocityY(0.0),
                    )
                },
                swerveToleranceCondition,
                0.5,
                "BAD BACKWARD DIRECTION",
            ),
            testSubSystem(
                robotContainer.drive.runOnce {
                    robotContainer.drive.setControl(
                        driveRequest.withVelocityX(0.0).withVelocityY(-1.0),
                    )
                },
                swerveToleranceCondition,
                0.5,
                "BAD LEFT DIRECTION",
            ),
            robotContainer.drive.runOnce {
                robotContainer.drive.setControl(
                    SwerveRequest.SwerveDriveBrake(),
                )
            },
            // -----------INTAKE-----------
            testSubSystem(robotContainer.intake.intake(), rollerToleranceCondition, 1.0, "BAD ROLLER"),
            testSubSystem(robotContainer.intake.deploy(), pivotToleranceCondition, 1.0, "BAD PIVOT DEPLOY"),
            robotContainer.intake.stopRollers(),
            testSubSystem(robotContainer.intake.stow(), pivotToleranceCondition, 1.0, "BAD PIVOT STOW"),
            // -----------INDEXER-----------
            testSubSystem(robotContainer.indexer.index(12.0, 3.0, 12.0), indexerToleranceCondition, 1.0, "BAD INDEXER"),
            robotContainer.indexer.stop().withTimeout(0.1),
            // -----------SHOOTER---------
            testSubSystem(
                robotContainer.shooter.setFlywheelVelocity(
                    ShooterConstants.TRENCH_FLYWHEEL_VEL,
                ),
                flywheelToleranceCondition,
                0.5,
                "BAD FLYWHEEL VELOCITY",
            ),
            testSubSystem(robotContainer.shooter.homeHood(), hoodToleranceCondition, 0.5, "BAD HOME HOOD ANGLE"),
            testSubSystem(
                robotContainer.shooter.setHoodAngle(
                    ShooterConstants.TOWER_HOOD_ANGLE,
                ),
                hoodToleranceCondition,
                0.75,
                "BAD TOWER HOOD ANGLE",
            ),
            testSubSystem(
                robotContainer.shooter.setHoodAngle(
                    ShooterConstants.TRENCH_HOOD_ANGLE,
                ),
                hoodToleranceCondition,
                0.75,
                "BAD TRENCH HOOD ANGLE",
            ),
            testSubSystem(
                robotContainer.shooter.setHoodAngle(ShooterConstants.MAX_HOOD_ANGLE),
                hoodToleranceCondition,
                0.75,
                "BAD MAX HOOD ANGLE",
            ),
            robotContainer.shooter.stopFlywheel(),
            robotContainer.shooter.homeHood(),
            Commands.runOnce({ Alert(errors.joinToString { ", " }, Alert.AlertType.kError).set(true) }),
        )
    }

    private fun testSubSystem(
        action: Command,
        condition: Boolean,
        timeOut: Double,
        errorMessage: String,
    ): Command =
        Commands.runOnce(
            {
                run { action }
                Commands.waitUntil { condition }.withTimeout(timeOut)
                Commands.runOnce({ errors.add(errorMessage) }).onlyIf { !condition }
            },
        )
}
