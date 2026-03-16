package frc.team449.subsystems

import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.DeferredCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.RepeatCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.team449.Constants.IndexerConstants
import frc.team449.Constants.ShooterConstants
import frc.team449.RobotContainer
import frc.team449.commands.SystemCheckCommand
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.indexer.IndexerSubsystem
import frc.team449.subsystems.intake.IntakeSubsystem
import frc.team449.subsystems.power.PowerProfile
import frc.team449.subsystems.power.PowerSubsystem
import frc.team449.subsystems.shooter.ShooterSubsystem
import org.littletonrobotics.junction.Logger
import kotlin.text.get

class RobotActions(
    private val robotContainer: RobotContainer
) {
    private val drive: DriveSubsystem = robotContainer.drive
    private val intake: IntakeSubsystem = robotContainer.intake
    private val indexer: IndexerSubsystem = robotContainer.indexer
    private val shooter: ShooterSubsystem = robotContainer.shooter
    private val power: PowerSubsystem = robotContainer.power

    fun deployAndRunIntake(): Command =
        SequentialCommandGroup(
            power.requestProfile(PowerProfile.INTAKING),
            intake.deploy(),
            ParallelCommandGroup(
                intake.intake(),
                indexer.index(
                    IndexerConstants.INTAKING_INDEXER_SPEED,
                    IndexerConstants.INTAKING_INDEXER_SPEED,
                    RadiansPerSecond.of(0.0),
                ),
            ),
        ).finallyDo { _ -> power.requestProfile(PowerProfile.DRIVING) }

    fun stopAndStow(): Command =
        SequentialCommandGroup(
            power.requestProfile(PowerProfile.DRIVING),
            intake.stopRollers(),
            ParallelRaceGroup(
                indexer.index(
                    IndexerConstants.INTAKING_INDEXER_SPEED,
                    IndexerConstants.INTAKING_INDEXER_SPEED,
                    RadiansPerSecond.of(0.0),
                ),
                intake.stow(),
            ),
        )

    fun shuffleIntakePivot(): Command =
        RepeatCommand(
//            SequentialCommandGroup(
//                intake.intake().withTimeout(0.580424),
//                intake.outtake().withTimeout(0.1678),
//            ),
            SequentialCommandGroup(
                intake.setPivotAngle(Radians.of(1.74)).withTimeout(.58042024),
                intake.setPivotAngle(Radians.of(1.04)).withTimeout(.58042024)
            )
        )

    fun prepShotFromAnywhere(distance: Double): Command =
        SequentialCommandGroup(
            power.requestProfile(PowerProfile.SHOOTING),
            InstantCommand({
                Logger.recordOutput("Aimbot/FlywheelEstimatedVel", ShooterConstants.FLYWHEEL_VELOCITY_MAP.get(distance))
                Logger.recordOutput("Aimbot/HoodEstimatedAngle", ShooterConstants.HOOD_ANGLE_MAP.get(distance))
            }),
            shooter
                .setAimCommand(
                    Radians.of(ShooterConstants.HOOD_ANGLE_MAP.get(distance)),
                    RadiansPerSecond.of(ShooterConstants.FLYWHEEL_VELOCITY_MAP.get(distance)),
                ).withTimeout(0.2),
        )

    fun checkAndFeed(): Command =
        RepeatCommand(
            ConditionalCommand(
                indexer.index(IndexerConstants.SHOOTING_INDEXER_SPEED).withTimeout(0.25),
                indexer.stop(),
            ) { shooter.isFlywheelAtTolerance() && shooter.isFlywheelAtTolerance() },
        )

    fun autoUnjam(): Command =
        DeferredCommand(
            {
                val currentSetpoint = shooter.flywheelTargetVelocityRadPerSec

                SequentialCommandGroup(
                    PrintCommand("AUTO UNJAM!"),
                    ParallelCommandGroup(
                        indexer.index(IndexerConstants.INTAKING_INDEXER_SPEED),
                        shooter.setFlywheelVelocity(-ShooterConstants.TEST_FLYWHEEL_VEL),
                    ).withTimeout(0.25),
                    indexer.stop(),
                    shooter.setFlywheelVelocity(RadiansPerSecond.of(currentSetpoint)),
                )
            },
            setOf(indexer, shooter),
        )

    fun reverseAll(): Command =
        ParallelCommandGroup(
            intake.outtake(),
            indexer.index(RadiansPerSecond.of(-30.0)),
            shooter.setFlywheelVelocity(-ShooterConstants.TEST_FLYWHEEL_VEL),
        )

    fun stopAllAndHomeHood(): Command =
        SequentialCommandGroup(
            intake.stopRollers().withTimeout(0.1),
            indexer.stop().withTimeout(0.1),
            shooter.stopFlywheel().withTimeout(0.1),
            shooter.homeHood(),
        )

    fun stopAll(): Command =
        SequentialCommandGroup(
            intake.stopRollers(),
            indexer.stop(),
            shooter.stopFlywheel(),
        )

    fun autoTrenchShot(): Command =
        SequentialCommandGroup(
            prepShotFromAnywhere(3.43),
            ParallelCommandGroup(
                checkAndFeed(),
                WaitCommand(0.5).andThen(
                    shuffleIntakePivot().withTimeout(0.5).andThen(
                        intake.stow().withTimeout(0.2).andThen(
                            intake.deploy().withTimeout(0.3),
                        ),
                    ),
                ),
            ),
        ).withTimeout(4.0).andThen(
            intake.stow().withTimeout(0.1),
        )

    fun autoHubShot(): Command =
        SequentialCommandGroup(
            prepShotFromAnywhere(1.294),
            checkAndFeed(),
        )

    fun systemCheckCommand(): Command = SystemCheckCommand(robotContainer)
}
