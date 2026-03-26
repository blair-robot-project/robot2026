package frc.team449

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.commands.AimAtTargetCommand
import frc.team449.commands.SwerveRequestCommand
import frc.team449.subsystems.power.PowerProfile
import frc.team449.subsystems.power.PowerSubsystem
import frc.team449.util.FieldUtil
import kotlin.math.abs

class Bindings(
    val robotContainer: RobotContainer
) {
    val driver = robotContainer.driveController
    val operator = robotContainer.opController
    val actions = robotContainer.actions

    val joysticksMovedPastDeadbandTrigger: Trigger =
        Trigger {
            abs(driver.leftY) > 0.25 || abs(driver.leftX) > 0.25 ||
                abs(driver.rightX) > 0.25
        }

    val shooterJamTrigger: Trigger = robotContainer.shooter.shooterJamTrigger

    fun setDefaultCommands() {
        robotContainer.drive.defaultCommand =
            SwerveRequestCommand(
                robotContainer.drive,
                { -driver.leftY },
                { -driver.leftX },
                { -driver.rightX },
            )

        robotContainer.intake.defaultCommand =
            robotContainer.intake.stopRollers()

        robotContainer.indexer.defaultCommand =
            robotContainer.indexer.stop()

//        robotContainer.shooter.defaultCommand =
//            robotContainer.shooter.stopFlywheel()
    }

    fun bindControls() {
        driver
            .rightTrigger()
            .whileTrue(
                actions.deployAndRunIntake(),
            )

        driver
            .leftTrigger()
            .onTrue(
                actions.stopAndStow(),
            )

        driver
            .rightBumper()
            .whileTrue(
                AimAtTargetCommand(
                    robotContainer.drive,
                    actions,
                    { FieldUtil.HUB_TRANSLATION }
                )
                    .withTimeout(2.0)
                    .andThen(PrintCommand("Align Complete."))
                    .andThen(
                        robotContainer.drive.xLock()
                            .alongWith(actions.checkAndFeed())
                            .alongWith(WaitCommand(1.0).andThen(actions.shuffleIntakePivot()))
                    )
                    .until(joysticksMovedPastDeadbandTrigger)
                    .finallyDo { _ ->
                        CommandScheduler.getInstance().schedule(
                            actions.stopAll(),
                            PowerSubsystem.requestProfile(PowerProfile.DRIVING)
                        )
                    }
            )

        driver
            .leftBumper()
            .whileTrue(
                SwerveRequestCommand(
                    robotContainer.drive,
                    { -driver.leftY },
                    { -driver.leftX },
                    { -driver.rightX },
                    Constants.DriveConstants.SLOW_LINEAR_SPEED_METERS_PER_SECOND,
                    Constants.DriveConstants.SLOW_ANGULAR_SPEED_RADIANS_PER_SECOND,
                ),
            )

        driver
            .a()
            .onTrue(
                robotContainer.drive
                    .xLock()
                    .until(joysticksMovedPastDeadbandTrigger),
            )

        driver
            .x()
            .whileTrue(
                actions.prepShotFromAnywhere(3.43)
                    .andThen(
                        robotContainer.drive.xLock()
                            .alongWith(actions.checkAndFeed())
                            .alongWith(WaitCommand(1.0).andThen(actions.shuffleIntakePivot()))
                    )
                    .finallyDo { _ ->
                        CommandScheduler.getInstance().schedule(
                            actions.stopAll(),
                            PowerSubsystem.requestProfile(PowerProfile.DRIVING)
                        )
                    }
            )

        driver
            .y()
            .whileTrue(
                actions.prepShotFromAnywhere(1.3)
                    .andThen(
                        robotContainer.drive.xLock()
                            .alongWith(actions.checkAndFeed())
                            .alongWith(WaitCommand(1.0).andThen(actions.shuffleIntakePivot()))
                    )
                    .finallyDo { _ ->
                        CommandScheduler.getInstance().schedule(
                            actions.stopAll(),
                            PowerSubsystem.requestProfile(PowerProfile.DRIVING)
                        )
                    }
            )

        driver
            .b()
            .whileTrue(
                actions.prepShotFromAnywhere(2.92)
                    .andThen(
                        robotContainer.drive.xLock()
                            .alongWith(actions.checkAndFeed())
                            .alongWith(WaitCommand(1.0).andThen(actions.shuffleIntakePivot()))
                    )
                    .finallyDo { _ ->
                        CommandScheduler.getInstance().schedule(
                            actions.stopAll(),
                            PowerSubsystem.requestProfile(PowerProfile.DRIVING)
                        )
                    }
            )

        driver
            .povDown()
            .onTrue(
                actions.reverseAll(),
            )

        driver
            .povRight()
            .onTrue(
                robotContainer.shooter.stopFlywheel()
            )

        driver
            .povUp()
            .whileTrue(
                actions.checkAndFeed()
            )

        driver
            .povLeft()
            .onTrue(
                actions.stopAllAndHomeHood()
            )

        driver
            .start()
            .onTrue(
                robotContainer.drive.seedFieldCentric()
            )

        shooterJamTrigger
            .onTrue(
                SequentialCommandGroup(
                    InstantCommand({ driver.setRumble(GenericHID.RumbleType.kBothRumble, 1.0) }),
                    actions.autoUnjam(),
                    InstantCommand({ driver.setRumble(GenericHID.RumbleType.kBothRumble, 0.0) }),
                    PrintCommand("RUMBLE COMPLETE."),
                    actions.checkAndFeed()
                )
            )

        operator
            .a()
            .whileTrue(
                SequentialCommandGroup(
                    actions.prepShotFromAnywhere(1.61),
                    ParallelCommandGroup(
                        actions.checkAndFeed(),
                        actions.shuffleIntakePivot()
                    )
                )
            )
    }
}
