package frc.team449.subsystems

import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Constants.IndexerConstants
import frc.team449.Constants.ShooterConstants
import frc.team449.RobotContainer
import frc.team449.commands.SystemCheckCommand
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.indexer.IndexerSubsystem
import frc.team449.subsystems.intake.IntakeSubsystem
import frc.team449.subsystems.shooter.ShooterSubsystem
import java.util.function.Supplier

class RobotActions(
    private val robotContainer: RobotContainer
) {
    private val drive: DriveSubsystem = robotContainer.drive
    private val intake: IntakeSubsystem = robotContainer.intake
    private val indexer: IndexerSubsystem = robotContainer.indexer
    private val shooter: ShooterSubsystem = robotContainer.shooter

    fun deployAndToggleIntake(): Command =
        SequentialCommandGroup(
            ConditionalCommand(
                ParallelCommandGroup(
                    intake.stopRollers(),
                    indexer.stop(),
                ),
                ParallelCommandGroup(
                    intake.intake(),
                    indexer.index(
                        IndexerConstants.INTAKING_INDEXER_SPEED,
                        IndexerConstants.INTAKING_INDEXER_SPEED,
                        RadiansPerSecond.of(0.0),
                    ),
                ),
            ) { intake.rollerTargetVelocityRadPerSec != 0.0 },
            intake.deploy(),
        )

    fun stopAndStow(): Command =
        SequentialCommandGroup(
            intake.stopRollers(),
            indexer.stop(),
            intake.stow(),
        )

    fun shuffleIntake(): Command =
        SequentialCommandGroup(
            intake.stow(),
            WaitCommand(0.5),
            intake.deploy(),
            WaitCommand(0.5),
        ).repeatedly()

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

    fun prepShotFromAnywhere(distanceSupplier: Supplier<Double>): Command =
        shooter.setFlywheelAndHoodFromSuppliers(
            {
                RadiansPerSecond.of(
                    ShooterConstants.FLYWHEEL_VELOCITY_MAP.get(distanceSupplier.get()),
                )
            },
            { Radians.of(ShooterConstants.HOOD_ANGLE_MAP.get(distanceSupplier.get())) },
        )

    fun checkAndFeed(): Command =
        SequentialCommandGroup(
            WaitUntilCommand {
                shooter.isFlywheelAtTolerance() && shooter.isHoodAtTolerance()
            },
            indexer.index(IndexerConstants.SHOOTING_INDEXER_SPEED),
        )

    fun stopFeed(): Command = indexer.stop()

    fun stopFeedAndShooter(): Command =
        ParallelCommandGroup(
            shooter.stopFlywheel(),
            indexer.stop(),
        )

    fun homeHood(): Command = shooter.homeHood()

    fun autoTrenchShot(): Command =
        SequentialCommandGroup(
            prepTrenchShot(),
            checkAndFeed().alongWith(
                shuffleIntake(),
            ),
        )

    fun autoHubShot(): Command =
        SequentialCommandGroup(
            prepHubShot(),
            checkAndFeed().alongWith(
                shuffleIntake(),
            ),
        )

    fun systemCheckCommand(): Command = SystemCheckCommand(robotContainer)
}
