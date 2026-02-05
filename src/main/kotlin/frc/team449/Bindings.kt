package frc.team449

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team449.RobotContainer.drive
import frc.team449.commands.AimAtTargetCommand
import frc.team449.commands.SwerveRequestCommand
import kotlin.jvm.optionals.getOrDefault

class Bindings(
    val robotContainer: RobotContainer
) {
    val driveController = robotContainer.driveController
    val opController = robotContainer.opController
    val aimbotting = false

    fun setDefaultCommands() {
        // set default commands for systems here
        robotContainer.drive.defaultCommand =
            SwerveRequestCommand(
                robotContainer.drive,
                { -robotContainer.driveController.leftY },
                { -robotContainer.driveController.leftX },
                { robotContainer.driveController.rightX },
            )
        toggleAimbot()
        // controls for simulation

    }

    fun toggleAimbot() {
        driveController.a().onTrue (
            runOnce ({
                if (aimbotting) {
                    CommandScheduler.getInstance().schedule(drive.defaultCommand)
                } else {
                    CommandScheduler.getInstance().schedule(
                        AimAtTargetCommand(
                            robotContainer.drive,
                            { -robotContainer.driveController.leftY },
                            { -robotContainer.driveController.leftX },
                            {if (DriverStation.getAlliance().getOrDefault(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) Constants.RED_GOAL_POSE else Constants.BLUE_GOAL_POSE},
                        )
                    )
                }
            })
        )
    }

    fun bindControls() {
        opController.povUp().whileTrue(drive.sysIDTranslationRoutine.quasistatic(SysIdRoutine.Direction.kForward))
        opController.povUp().whileTrue(drive.sysIDTranslationRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
        opController.povUp().whileTrue(drive.sysIDTranslationRoutine.dynamic(SysIdRoutine.Direction.kForward))
        opController.povUp().whileTrue(drive.sysIDTranslationRoutine.dynamic(SysIdRoutine.Direction.kReverse))
    }
}
