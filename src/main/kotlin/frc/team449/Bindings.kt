package frc.team449

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.DeferredCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.commands.AimAtTargetCommand
import frc.team449.commands.SwerveRequestCommand
import frc.team449.subsystems.power.PowerProfile
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
                robotContainer.power,
                { -driver.leftY },
                { -driver.leftX },
                { -driver.rightX },
            )

        robotContainer.intake.defaultCommand =
            robotContainer.intake.stopRollers()

        robotContainer.indexer.defaultCommand =
            robotContainer.indexer.stop()

        robotContainer.shooter.defaultCommand =
            robotContainer.shooter.stopFlywheel()
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
                DeferredCommand(
                    {
                        SequentialCommandGroup(
                            AimAtTargetCommand(
                                robotContainer.drive,
                                robotContainer.power,
                                actions,
                                FieldUtil.HUB_TRANSLATION
                            ).withTimeout(3.0),
                            PrintCommand("Align Complete!"),
                            ParallelCommandGroup(
                                robotContainer.drive.xLock(),
                                actions.checkAndFeed(),
                                actions.shuffleIntakePivot()
                            )
                        )
                    },
                    setOf<Subsystem>(robotContainer.intake, robotContainer.indexer)
                )
                    .until(joysticksMovedPastDeadbandTrigger)
                    .finallyDo { _ -> CommandScheduler.getInstance().schedule(robotContainer.power.requestProfile(PowerProfile.DRIVING), actions.stopAllAndHomeHood()) }
            )

        driver
            .leftBumper()
            .whileTrue(
                SwerveRequestCommand(
                    robotContainer.drive,
                    robotContainer.power,
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
                    actions.autoUnjam()
                )
                    .finallyDo { _ -> driver.setRumble(GenericHID.RumbleType.kBothRumble, 0.0) }
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
