package frc.team449.subsystems
import edu.wpi.first.wpilibj2.command.Command
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
    private val robotContainer: RobotContainer
) {
    val drive: DriveSubsystem = robotContainer.drive
    val intake: IntakeSubsystem = robotContainer.intake
    val indexer: IndexerSubsystem = robotContainer.indexer
    val shooter: ShooterSubsystem = robotContainer.shooter

    fun deployAndIntake(): Command =
        SequentialCommandGroup(
            intake.intake(),
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

    fun feed(): Command =
        SequentialCommandGroup(
            WaitUntilCommand {
                shooter.isFlywheelAtTolerance() && shooter.isHoodAtTolerance()
            },
            shooter.holdHood(),
            indexer.index(IndexerConstants.SHOOTING_INDEXER_SPEED),
        )

    fun stopShooter(): Command =
        ParallelCommandGroup(
            shooter.stopFlywheel(),
            indexer.stop(),
        )

    fun homeHood(): Command = shooter.homeHood()
}
