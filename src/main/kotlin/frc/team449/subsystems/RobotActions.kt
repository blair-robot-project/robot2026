package frc.team449.subsystems

import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
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

    fun prepShotFromDistanceMeters(distanceMeters: Double): Command =
        Commands.sequence(
            shooter.setFlywheelVelocity(RadiansPerSecond.of(ShooterConstants.FLYWHEEL_VELOCITY_MAP.get(distanceMeters))),
            shooter.setHoodAngle(Radians.of(ShooterConstants.HOOD_ANGLE_MAP.get(distanceMeters))),
        )

    fun checkAndFeed(): Command =
        Commands
            .sequence(
                Commands.waitUntil { shooter.isFlywheelAtTolerance() && shooter.isHoodAtTolerance() },
                indexer.setIndexerVoltage(12.0, 12.0),
            ).withName("CheckAndFeed")

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

    fun stopFeed(): Command = indexer.stop()

    fun stopAllAndHomeHood(): Command =
        Commands.sequence(
            intake.stopRollers(),
            indexer.stop(),
            shooter.stopFlywheel(),
            shooter.homeHoodNoDeferred(),
        ).withName("StopAllAndHomeHood")

    fun tuckAndClear(): Command =
        Commands.sequence(
            intake.setRollerVoltage(6.0),
            WaitCommand(1.6),
            intake.stopRollers(),
            intake.stowSlow()
        ).withName("TuckAndClear")

    fun autoTrenchShot(): Command =
        Commands.sequence(
            prepShotFromDistanceMeters(3.43),
            checkAndFeed().andThen(tuckAndClear()),
        )

    fun autoBumpShot(): Command =
        Commands.sequence(
            prepShotFromDistanceMeters(2.22),
            checkAndFeed().andThen(tuckAndClear()),
        )

    fun autoHubShot(): Command =
        Commands.sequence(
            prepShotFromDistanceMeters(2.5),
            checkAndFeed().andThen(tuckAndClear()),
        )
}
