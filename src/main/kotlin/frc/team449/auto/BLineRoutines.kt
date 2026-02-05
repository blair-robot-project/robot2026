package frc.team449.auto
import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.trajectory.SwerveSample
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.robot.lib.BLine.*
import frc.team449.Robot
import frc.team449.RobotContainer.drive
import org.littletonrobotics.junction.Logger

class BLineRoutines(
    robot: Robot
) {
    val autoFactory =
        AutoFactory(
            drive::getPose,
            drive::resetOdometry,
            { sample: SwerveSample -> drive.followTrajectory(robot, sample) },
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
                        SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds),
                    )
                },
                PIDController(1.5, 0.0, 0.05),
                PIDController(3.0, 0.0, 0.0),
                PIDController(1.0, 0.0, 0.0),
            ).withDefaultShouldFlip()
            .withPoseReset(drive::resetOdometry)

    fun bump2cycle(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(Path("zigzag")), // 9.5
                // use the event triggers on the path
            ),
        )
        return routine
    }

    fun trench2cycleRight(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val path = Path("trench_1")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shoot", PrintCommand("Starting shoot"))
        FollowPath.registerEventTrigger("end_shoot", PrintCommand("Ending shoot"))

        routine.active().onTrue(
            pathBuilder.build(path)
        )

        return routine
    }

    fun trench2cycleLeft(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val path = Path("trench_3")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shoot", PrintCommand("Starting shoot"))
        FollowPath.registerEventTrigger("end_shoot", PrintCommand("Ending shoot"))

        routine.active().onTrue(
            pathBuilder.build(path),
        )

        return routine
    }

    fun trenchChuteRight(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val path = Path("trench_2")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("shoot", PrintCommand("Shooting"))
        FollowPath.registerEventTrigger("chute_intake", PrintCommand("Intaking from chute"))

        routine.active().onTrue(
            pathBuilder.build(path),
        )

        return routine
    }

    fun addOptions(autoChooser: AutoChooser) {
        autoChooser.addRoutine("B-Line 2 cycle bump (R)", this::bump2cycle)
        autoChooser.addRoutine("B-Line 2 cycle trench (R)", this::trench2cycleRight)
        autoChooser.addRoutine("B-Line 2 cycle trench (L)", this::trench2cycleLeft)
        autoChooser.addRoutine("B-Line 1 cycle trench + chute cycle (R)", this::trenchChuteRight)
    }
}
