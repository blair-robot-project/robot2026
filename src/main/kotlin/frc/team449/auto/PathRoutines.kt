package frc.team449.auto

import com.ctre.phoenix6.swerve.SwerveRequest
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.GoalEndState
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Constants.AutoConstants.config
import frc.team449.Robot
import frc.team449.RobotContainer.drive

class PathRoutines(
    robot: Robot,
) {
    // pathplanner stuff
    init {
        AutoBuilder.configure(
            drive::getPose,
            drive::resetOdometry,
            drive::getRobotRelativeSpeeds,
            { speeds: ChassisSpeeds ->
                drive.setControl(
                    SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds),
                )
            },
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(5.0, 0.0, 0.0),
            ),
            config,
            {
                val alliance = DriverStation.getAlliance()
                alliance.isPresent && alliance.get() == DriverStation.Alliance.Red
            },
            drive,
        )
    }

    fun testPath(): Command {
        val startPose = Pose2d(4.0, 7.5, Rotation2d.fromDegrees(0.0))

        val waypoints =
            PathPlannerPath.waypointsFromPoses(
                startPose,
                Pose2d(7.8, 6.8, Rotation2d.fromDegrees(0.0)),
                Pose2d(7.8, 4.5, Rotation2d.fromDegrees(270.0)),
            )

        val constraints =
            PathConstraints(
                3.0,
                3.0,
                3.0,
                1.0,
            )
        val path =
            PathPlannerPath(
                waypoints,
                constraints,
                null,
                GoalEndState(
                    0.0,
                    Rotation2d.fromDegrees(270.0),
                ),
            )

// Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = false
        return AutoBuilder.followPath(path)
    }
}
