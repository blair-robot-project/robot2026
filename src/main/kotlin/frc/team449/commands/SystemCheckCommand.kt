package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
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
                robotContainer.drive.isWithinTolerance(),
                0.5,
                "BAD FORWARD DIRECTION",
            ),
            testSubSystem(
                robotContainer.drive.runOnce {
                    robotContainer.drive.setControl(
                        driveRequest.withVelocityX(0.0).withVelocityY(1.0),
                    )
                },
                robotContainer.drive.isWithinTolerance(),
                0.5,
                "BAD RIGHT DIRECTION",
            ),
            testSubSystem(
                robotContainer.drive.runOnce {
                    robotContainer.drive.setControl(
                        driveRequest.withVelocityX(-1.0).withVelocityY(0.0),
                    )
                },
                robotContainer.drive.isWithinTolerance(),
                0.5,
                "BAD BACKWARD DIRECTION",
            ),
            testSubSystem(
                robotContainer.drive.runOnce {
                    robotContainer.drive.setControl(
                        driveRequest.withVelocityX(0.0).withVelocityY(-1.0),
                    )
                },
                robotContainer.drive.isWithinTolerance(),
                0.5,
                "BAD LEFT DIRECTION",
            ),
            robotContainer.drive.runOnce {
                robotContainer.drive.setControl(
                    SwerveRequest.SwerveDriveBrake(),
                )
            },
            // -----------INTAKE-----------
            testSubSystem(robotContainer.intake.setRollerVoltage(12.0), robotContainer.intake.rollerAtTolerance(), 1.0, "BAD ROLLER"),
            testSubSystem(robotContainer.intake.deploy(), robotContainer.intake.pivotAtTolerance(), 1.0, "BAD PIVOT DEPLOY"),
            robotContainer.intake.stopRollers(),
            testSubSystem(robotContainer.intake.stow(), robotContainer.intake.pivotAtTolerance(), 1.0, "BAD PIVOT STOW"),
            // -----------INDEXER-----------
            testSubSystem(
                robotContainer.indexer.setIndexerVoltage(
                    0.5,
                    0.0,
                ),
                robotContainer.indexer.indexerAtTolerance(),
                1.0,
                "BAD INDEXER",
            ),
            robotContainer.indexer.stop().withTimeout(0.1),
            // -----------SHOOTER---------
            testSubSystem(
                robotContainer.shooter.setFlywheelVelocity(
                    RadiansPerSecond.of(ShooterConstants.FLYWHEEL_VELOCITY_MAP.get(1.3)), // FIX IT
                ),
                robotContainer.shooter.isFlywheelAtTolerance(),
                0.5,
                "BAD FLYWHEEL VELOCITY",
            ),
            testSubSystem(robotContainer.shooter.homeHood(), robotContainer.shooter.isHoodAtTolerance(), 0.5, "BAD HOME HOOD ANGLE"),
            testSubSystem(
                robotContainer.shooter.setHoodAngle(ShooterConstants.MIN_HOOD_ANGLE),
                robotContainer.shooter.isHoodAtTolerance(),
                0.75,
                "BAD MIN HOOD ANGLE",
            ),
            testSubSystem(
                robotContainer.shooter.setHoodAngle(ShooterConstants.MAX_HOOD_ANGLE),
                robotContainer.shooter.isHoodAtTolerance(),
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
        SequentialCommandGroup(
            action,
            Commands.waitUntil { condition }.withTimeout(timeOut),
            Commands.runOnce({ errors.add(errorMessage) }).onlyIf { !condition },
        )
}
