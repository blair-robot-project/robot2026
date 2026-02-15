package frc.team449

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import frc.team449.Constants.FieldConstants
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI

object Field {
    fun getClosestTrenchPose(robotPose: Pose2d): Pose2d {
        val flipRed = DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red

        val allianceTrenchSpots: List<Pose2d> = if (flipRed) {
            FieldConstants.BLUE_TRENCH_POSES.map { flipPose(it) }
        } else {
            FieldConstants.BLUE_TRENCH_POSES
        }

        return allianceTrenchSpots.minBy {
            it.translation.getDistance(robotPose.translation)
        }
    }

    // flip (wall-blue zero)
    fun flipPose(pose: Pose2d): Pose2d {
        return Pose2d(
            FieldConstants.FIELD_LENGTH_METERS - pose.x,
            pose.y,
            Rotation2d(PI).minus(pose.rotation)
        )
    }
}