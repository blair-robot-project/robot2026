package frc.team449.subsystems
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Constants.IndexerConstants
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

    fun deployAndToggleIntake(): Command =
        SequentialCommandGroup(
            ConditionalCommand(
                intake.stopRollers(),
                intake.intake(),
            ) { intake.rollerTargetVelocityRadPerSec != 0.0 },
            intake.deploy(),
        )

    fun stopAndStow(): Command =
        SequentialCommandGroup(
            intake.stopRollers(),
            intake.stow(),
        )

    fun stopIntake(): Command = intake.stopRollers()

    fun prepTrenchShot(): Command =
        SequentialCommandGroup(
            shooter.setFlywheelVelocity(ShooterConstants.TRENCH_FLYWHEEL_VEL),
            shooter.setHoodAngle(ShooterConstants.TRENCH_HOOD_ANGLE),
        )

    fun prepHubShot(): Command =
        SequentialCommandGroup(
            shooter.setFlywheelVelocity(ShooterConstants.HUB_FLYWHEEL_VEL),
            shooter.setHoodAngle(ShooterConstants.HUB_HOOD_ANGLE),
        )

    fun prepTowerShot(): Command =
        SequentialCommandGroup(
            shooter.setFlywheelVelocity(ShooterConstants.TOWER_FLYWHEEL_VEL),
            shooter.setHoodAngle(ShooterConstants.TOWER_HOOD_ANGLE),
        )

    fun checkAndFeed(): Command =
        SequentialCommandGroup(
            WaitUntilCommand {
                shooter.isFlywheelAtTolerance() && shooter.isHoodAtTolerance()
            },
            indexer.index(IndexerConstants.SHOOTING_INDEXER_SPEED),
        )

    fun stopFeed(): Command = indexer.stop()

    fun stopShooter(): Command =
        ParallelCommandGroup(
            shooter.stopFlywheel(),
            indexer.stop(),
        )

    fun homeHood(): Command = shooter.homeHood()

    fun autoTrenchShot(): Command =
        SequentialCommandGroup(
            prepTrenchShot(),
            checkAndFeed(),
        )
}
