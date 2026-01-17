package frc.team449.auto
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.lib.BLine.*
import frc.team449.Robot
import frc.team449.RobotContainer.drive


class bLineRoutines(robot: Robot) {

    // Set global constraints before creating any paths
    init {
        Path.setDefaultGlobalConstraints( Path.DefaultGlobalConstraints(
                4.58,    // maxVelocityMetersPerSec
            6.0,   // maxAccelerationMetersPerSec2
            730.52,    // maxVelocityDegPerSec
            1091.4846,    // maxAccelerationDegPerSec2
            0.001,   // endTranslationToleranceMeters
            2.0,    // endRotationToleranceDeg
            0.02     // intermediateHandoffRadiusMeters
        ))
    }


   var pathBuilder: FollowPath.Builder =  FollowPath.Builder(
    drive,
    drive::getPose,
    drive::getRobotRelativeSpeeds,
    { speeds: ChassisSpeeds ->
        drive.setControl(
            SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds)
        )
    },
    PIDController(5.0, 0.0, 0.0),
     PIDController(2.0, 0.0, 0.0),
     PIDController(3.0, 0.0, 0.0)
    ).withDefaultShouldFlip()
    .withPoseReset(drive::resetOdometry)


    val rangedConstraints = Path.PathConstraints()
        .setMaxVelocityMetersPerSec(
            Path.RangedConstraint(3.0, 1, 2),
            Path.RangedConstraint(2.5, 2, 4)
        )
        .setMaxVelocityDegPerSec(
            Path.RangedConstraint(60.0, 1, 2),
            Path.RangedConstraint(0.057, 2, 3)
        )
    //    .setMaxAccelerationMetersPerSec2(0.2)







    var myPath: Path = Path(
        rangedConstraints,
        Path.Waypoint(Translation2d(4.0, 7.5), Rotation2d(0.0)),
        Path.TranslationTarget(5.5,7.0,0.1),
        Path.Waypoint(Translation2d(7.8, 6.8), Rotation2d.fromDegrees(-90.0)),
        Path.Waypoint(Translation2d(7.8, 4.5), Rotation2d.fromDegrees(-90.0))
    )






    var constraints: Path.PathConstraints = Path.PathConstraints()
        .setMaxVelocityMetersPerSec(2.0)
        .setMaxAccelerationMetersPerSec2(1.5)
        .setMaxVelocityDegPerSec(180.0)
        .setMaxAccelerationDegPerSec2(360.0)
        .setEndTranslationToleranceMeters(0.02)
        .setEndRotationToleranceDeg(1.0)

    var slowPath: Path = Path(
        constraints,
        Path.Waypoint(Translation2d(1.0, 1.0), Rotation2d(0.0)),
        Path.TranslationTarget(Translation2d(2.0, 2.0)),
        Path.Waypoint(Translation2d(3.0, 1.0), Rotation2d(Math.PI))
    )



    val variedPath = Path(
        rangedConstraints,
    )

    //var followCommand: Command = pathBuilder.build(myPath)

    fun runTestBline(): Command{
      return  Commands.sequence(
              pathBuilder.build(myPath),

      )
    }

    fun getAutonomousCommand(): Command {
        return Commands.sequence(
            pathBuilder.build(Path("toFirstScore")),
            pathBuilder.build(Path("toPickup")),
            pathBuilder.build(Path("toSecondScore")),
        )
    }
}