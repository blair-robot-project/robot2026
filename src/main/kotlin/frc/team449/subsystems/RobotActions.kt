package frc.team449.subsystems

import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.RepeatCommand
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

    fun deployAndRunIntake(): Command =
        SequentialCommandGroup(
            intake.intake(),
            indexer.index(
                IndexerConstants.INTAKING_INDEXER_SPEED,
                IndexerConstants.INTAKING_INDEXER_SPEED,
                RadiansPerSecond.of(0.0),
            ),
            intake.deploy(),
        )

    fun autoDeployAndRunIntake(): Command =
        SequentialCommandGroup(
            intake.intake(),
            indexer.index(
                IndexerConstants.INTAKING_INDEXER_SPEED,
                IndexerConstants.INTAKING_INDEXER_SPEED,
                RadiansPerSecond.of(0.0),
            ),
            intake.repeatedlyDeploy(),
        )

    fun stopAndStow(): Command =
        SequentialCommandGroup(
            intake.stopRollers(),
            indexer.stop(),
            intake.stow(),
        )

    fun shuffleIntakeRoller(): Command =
        SequentialCommandGroup(
            intake.intake(),
            WaitCommand(0.3),
            intake.outtake(),
            WaitCommand(0.1),
        ).repeatedly()

    fun shuffleIntakePivot(): Command =
        SequentialCommandGroup(
            intake.deploy(),
            WaitCommand(0.6),
            intake.stow(),
            WaitCommand(0.3),
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
            indexer.index(IndexerConstants.SHOOTING_INDEXER_SPEED).repeatedly(),
        )

    fun autoUnjam(): Command =
        SequentialCommandGroup(
            PrintCommand("AUTO UNJAM!"),
            RepeatCommand(
                ParallelCommandGroup(
                    robotContainer.indexer.index(IndexerConstants.INTAKING_INDEXER_SPEED),
                    robotContainer.shooter.setFlywheelVelocity(-ShooterConstants.TEST_FLYWHEEL_VEL),
                )
            ).withTimeout(0.25),
            stopFeedAndShooter(),
        )

    fun stopFeed(): Command = indexer.stop()

    fun stopFeedAndShooter(): Command =
        ParallelCommandGroup(
            shooter.stopFlywheel(),
            indexer.stop(),
        )

    fun homeHood(): Command = shooter.homeHood()

    fun outtakeIntakeAndReverseIndex(): Command =
        ParallelCommandGroup(
            intake.outtake(),
            indexer.index(RadiansPerSecond.of(-30.0)),
        )

    fun autoTrenchShot(): Command =
        SequentialCommandGroup(
            prepTrenchShot(),
            checkAndFeed().alongWith(
                shuffleIntakeRoller(), // should add pivot shuffle?
            ),
        )

    fun autoHubShot(): Command =
        SequentialCommandGroup(
            prepHubShot(),
            checkAndFeed(),
        )

    fun systemCheckCommand(): Command = SystemCheckCommand(robotContainer)
}
