package frc.team449.subsystems

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.Constants.ShooterConstants
import frc.team449.RobotContainer
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.indexer.IndexerSubsystem
import frc.team449.subsystems.intake.IntakeSubsystem
import frc.team449.subsystems.shooter.ShooterSubsystem

class RobotActions(
    robotContainer: RobotContainer
) {
    private val drive: DriveSubsystem = robotContainer.drive
    private val intake: IntakeSubsystem = robotContainer.intake
    private val indexer: IndexerSubsystem = robotContainer.indexer
    private val shooter: ShooterSubsystem = robotContainer.shooter

    fun deployAndIntake(): Command =
        Commands.sequence(
            intake.deploy().unless { intake.pivotIsDeployed },
            intake.setPivotVoltage(1.0),
            intake.setRollerVoltage(12.0)
        )
            .withName("DeployIntake")

    fun stopIntakeAndPivot(): Command =
        Commands.sequence(
            intake.setPivotVoltage(0.0),
            intake.stopRollers()
        )
            .withName("StopIntakePivot")

    fun stopIntake(): Command =
        intake.stopRollers()
            .withName("StopIntake")

    fun stopAndStow(): Command =
        Commands.sequence(
            intake.stopRollers(),
            intake.stow(),
        )
            .withName("StopStow")

    fun prepShotFromDistanceMeters(distanceMeters: Double): Command =
        Commands.sequence(
            shooter.setFlywheelVelocity(Units.RadiansPerSecond.of(ShooterConstants.SCORING_FLYWHEEL_VELOCITY_MAP.get(distanceMeters))),
            shooter.setHoodAngle(Units.Radians.of(ShooterConstants.SCORING_HOOD_ANGLE_MAP.get(distanceMeters))),
        )

    fun checkAndFeed(): Command =
        Commands.sequence(
            Commands.waitUntil { shooter.isFlywheelAtTolerance() && shooter.isHoodAtTolerance() }
                .withTimeout(2.0),
            indexer.setIndexerVoltage(12.0, 12.0),
        )
            .withName("CheckFeed")

    fun reverseAll(): Command =
        Commands.parallel(
            intake.setRollerVoltage(-2.0),
            indexer.setIndexerVoltage(-2.0, -2.0),
            shooter.setFlywheelVelocity(ShooterConstants.UNJAM_FLYWHEEL_VEL),
        )
            .withName("ReverseAll")

    fun stopAll(): Command =
        Commands.parallel(
            shooter.stopFlywheel(),
            intake.stopRollers(),
            indexer.stop()
        )
            .withName("StopAll")

    fun stopShooterIndexer(): Command =
        Commands.parallel(
            shooter.stopFlywheel(),
            indexer.stop()
        )

    fun stopAllAndHomeHood(): Command =
        Commands.parallel(
            Commands.sequence(
                shooter.stopFlywheel(),
                shooter.homeHoodNoDeferred()
            ),
            intake.stopRollers(),
            indexer.stop()
        )
            .withName("StopAllHomeHood")

    fun tuckAndClear(): Command =
        Commands.sequence(
            intake.setRollerVoltage(2.0),
            WaitCommand(0.4),
            intake.stowSlow(),
        )
            .withName("TuckClear")

    fun autoShot(distanceMeters: Double): Command =
        Commands.sequence(
            prepShotFromDistanceMeters(distanceMeters),
            checkAndFeed().andThen(tuckAndClear()),
        )
}
