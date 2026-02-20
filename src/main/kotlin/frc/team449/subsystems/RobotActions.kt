package frc.team449.subsystems
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Constants.IndexerConstants
import frc.team449.Constants.ShooterConstants
import frc.team449.subsystems.indexer.IndexerSubsystem
import frc.team449.subsystems.intake.IntakeSubsystem
import frc.team449.subsystems.shooter.ShooterSubsystem

class RobotActions(
    private val intake: IntakeSubsystem,
    private val indexer: IndexerSubsystem,
    private val shooter: ShooterSubsystem
) {
    fun intake(): Command =
        Commands.sequence(
            ConditionalCommand(
                Commands.none(),
                intake.deploy(),
            ) { intake.isIntakeDeployed() },
            intake.intake(),
// should we run side and floor indexer at low volt here?
        )

    fun stopIntake(): Command = intake.stopRollers()

    fun prepTrenchShooter(): Command =
        Commands.parallel(
            shooter.setHoodAngle(ShooterConstants.TRENCH_HOOD_ANGLE),
            shooter.setFlywheelVelocity(ShooterConstants.TRENCH_FLYWHEEL_VEL),
        )

    fun shoot(): Command =
        Commands.sequence(
            WaitUntilCommand {
                shooter.isFlywheelAtTolerance() && shooter.isHoodAtTolerance()
            },
            shooter.holdHood(),
            indexer.index(IndexerConstants.SHOOTING_INDEXER_SPEED),
            // hopper jank stuff here
        )

    fun stopShooter(): Command =
        Commands.parallel(
            shooter.stopFlywheel(),
            shooter.homeHood(),
            indexer.stop(),
        )

    fun stopAll(): Command =
        Commands.parallel(
            indexer.stop(),
            intake.stopRollers(),
            shooter.stopFlywheel(),
        )
}
