package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.trajectory.SwerveSample
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.robot.lib.BLine.FollowPath
import frc.robot.lib.BLine.Path
import frc.team449.RobotContainer.drive
import org.littletonrobotics.junction.Logger

class BLineRoutines {
    val autoFactory =
        AutoFactory(
            drive::getPose,
            drive::resetOdometry,
            { sample: SwerveSample -> drive.followTrajectory(sample) },
            true,
            drive,
        )

    var pathBuilder: FollowPath.Builder =
        FollowPath
            .Builder(
                drive,
                drive::getPose,
                drive::getRobotRelativeSpeeds,
                { speeds: ChassisSpeeds ->
                    Logger.recordOutput("Bline speed", speeds)
                    drive.setControl(
                        ApplyRobotSpeeds().withSpeeds(speeds),
                    )
                },
                PIDController(1.6, 0.0, 0.05),
                PIDController(2.7, 0.0, 0.0),
                PIDController(1.0, 0.0, 0.0),
            ).withDefaultShouldFlip()
            .withPoseReset(drive::resetOdometry)

    fun test(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val path = Path("test")

        FollowPath.registerEventTrigger(
            "startIntake",
            PrintCommand("run intake"),
        )
        FollowPath.registerEventTrigger(
            "startShooter",
            Commands.parallel(
                PrintCommand("run shooter"),
                PrintCommand("run indexer"),
                PrintCommand("stop intake"),
            ),
        )
        FollowPath.registerEventTrigger(
            "end",
            PrintCommand("stop all"),
        )

        routine.active().onTrue(
            pathBuilder.build(path),
        )

        return routine
    }

    fun addOptions(autoChooser: AutoChooser) {
        autoChooser.addRoutine("B-Line test path", this::test)
    }
}
