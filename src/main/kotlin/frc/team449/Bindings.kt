package frc.team449

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team449.commands.SwerveRequestCommand
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI

class Bindings(
    val robotContainer: RobotContainer
) {
    val driveController = robotContainer.driveController
    val opController = robotContainer.opController

    fun setDefaultCommands() {
        // set default commands for systems here
        robotContainer.drive.defaultCommand =
            SwerveRequestCommand(
                robotContainer.drive,
                { -robotContainer.driveController.leftY },
                { -robotContainer.driveController.leftX },
                { robotContainer.driveController.rightX },
            )
    }

    fun bindControls() {
        driveController.povUp().onTrue(
            ConditionalCommand(
                InstantCommand({ robotContainer.drive.heading = Rotation2d(PI) }),
                InstantCommand({ robotContainer.drive.heading = Rotation2d() }),
            ) { DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red },
        )

        driveController.a().onTrue(
            robotContainer.intake.deploy()
        )
        driveController.b().onTrue(
            robotContainer.intake.stow()
        )

        opController.povUp().whileTrue(robotContainer.drive.sysIDTranslationRoutine.quasistatic(SysIdRoutine.Direction.kForward))
        opController.povUp().whileTrue(robotContainer.drive.sysIDTranslationRoutine.quasistatic(SysIdRoutine.Direction.kReverse))
        opController.povUp().whileTrue(robotContainer.drive.sysIDTranslationRoutine.dynamic(SysIdRoutine.Direction.kForward))
        opController.povUp().whileTrue(robotContainer.drive.sysIDTranslationRoutine.dynamic(SysIdRoutine.Direction.kReverse))
    }
}
