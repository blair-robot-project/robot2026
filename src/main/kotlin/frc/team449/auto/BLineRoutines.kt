package frc.team449.auto
import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.trajectory.SwerveSample
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.BLine.*
import frc.team449.Robot
import frc.team449.RobotContainer.drive
import org.littletonrobotics.junction.Logger

class BLineRoutines(
    robot: Robot,
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
                PIDController(2.65, 0.0, 0.0),
                PIDController(1.0, 0.0, 0.0),
            ).withDefaultShouldFlip()
            .withPoseReset(drive::resetOdometry)

    fun oneCycleR(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("intake")
        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(Path("1")),
                pathBuilder.build(Path("2")),
                pathBuilder.build(Path("3")),
            ),
        )
        return routine
    }

    fun jp(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("intake")
        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(Path("first_half")),
            ),
        )
        return routine
    }

    fun addOptions(autoChooser: AutoChooser) {
        autoChooser.addRoutine("B-Line 1 cycle (R)", this::oneCycleR)
        autoChooser.addRoutine("Jp", this::jp)
    }
}
