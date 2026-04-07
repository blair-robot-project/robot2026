package frc.team449.subsystems

import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.team449.Constants.ShooterConstants
import frc.team449.RobotContainer
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.indexer.IndexerSubsystem
import frc.team449.subsystems.intake.IntakeSubsystem
import frc.team449.subsystems.shooter.ShooterSubsystem

class RobotActions(
    robotContainer: RobotContainer,
) {
    private val drive: DriveSubsystem = robotContainer.drive
    private val intake: IntakeSubsystem = robotContainer.intake
    private val indexer: IndexerSubsystem = robotContainer.indexer
    private val shooter: ShooterSubsystem = robotContainer.shooter

    fun deployAndIntake(): Command =
        Commands
            .sequence(
                intake.deploy().unless { intake.pivotIsDeployed },
                Commands.parallel(
                    intake.setRollerVoltage(12.0),
                    indexer.setIndexerVoltage(0.5, 0.0),
                ),
            ).withName("DeployAndIntake")

    fun stopIntakeAndPivot(): Command =
        Commands
            .sequence(
                intake.stopRollers(),
                indexer.setIndexerVoltage(0.0, 0.0),
                intake.setPivotVoltage(0.0),
            ).withName("StopIntakeAndPivot")

    fun stopAndStow(): Command =
        Commands
            .sequence(
                intake.stopRollers(),
                indexer.setIndexerVoltage(0.5, 0.0),
                intake.stow(),
                indexer.setIndexerVoltage(0.0, 0.0),
            ).withName("StopAndStow")

    fun shuffleIntakePivot(): Command =
        Commands.sequence(
            intake.setRollerVoltage(3.0),
            Commands.repeatingSequence(
                intake.setPivotAngle(Radians.of(1.6)),
                Commands.waitSeconds(0.15),
                intake.setPivotAngle(Radians.of(0.85)),
                Commands.waitSeconds(0.15),
            ),
        )

    fun prepShotFromAnywhere(distance: Double): Command =
        Commands.sequence(
            shooter.setFlywheelVelocity(RadiansPerSecond.of(ShooterConstants.FLYWHEEL_VELOCITY_MAP.get(distance))),
            shooter.setHoodAngle(Radians.of(ShooterConstants.HOOD_ANGLE_MAP.get(distance))),
        )

    fun checkAndFeed(): Command =
        Commands
            .sequence(
                Commands.waitUntil { shooter.isFlywheelAtTolerance() && shooter.isHoodAtTolerance() },
                indexer.setIndexerVoltage(8.0, 12.0),
            ).withName("CheckAndFeed")

    fun autoUnjam(): Command =
        Commands.defer({
            val originalFlywheelSetpointRadsPerSec = shooter.flywheelTargetVelocityRadsPerSec

            Commands.sequence(
                Commands.print("AUTO-UNJAM!"),
                Commands.parallel(
                    indexer.setIndexerVoltage(0.0, 2.0),
                    shooter.setFlywheelVelocity(ShooterConstants.UNJAM_FLYWHEEL_VEL),
                ),
                Commands.waitSeconds(0.25),
                indexer.stop(),
                Commands.print("POST-UNJAM SETPOINT: $originalFlywheelSetpointRadsPerSec"),
                shooter.setFlywheelVelocity(RadiansPerSecond.of(originalFlywheelSetpointRadsPerSec)),
            )
        }, setOf(indexer, shooter))

    fun reverseAll(): Command =
        Commands.parallel(
            intake.setRollerVoltage(-2.0),
            indexer.setIndexerVoltage(-2.0, -2.0),
            shooter.setFlywheelVelocity(ShooterConstants.UNJAM_FLYWHEEL_VEL),
        )

    fun stopAll(): Command =
        Commands
            .sequence(
                intake.stopRollers(),
                indexer.stop(),
                shooter.stopFlywheel(),
            ).withName("StopAll")

    fun stopAllAndHomeHood(): Command =
        Commands.sequence(
            stopAll(),
            shooter.homeHood(),
        )

    fun autoTrenchShot(): Command =
        Commands.sequence(
            prepShotFromAnywhere(3.43),
            checkAndFeed(),
        )

    fun autoLemonShot(): Command =
        Commands.sequence(
            prepShotFromAnywhere(2.22),
            checkAndFeed(),
        )

    fun autoHubShot(): Command =
        Commands.sequence(
            prepShotFromAnywhere(1.3),
            checkAndFeed(),
        )
}
