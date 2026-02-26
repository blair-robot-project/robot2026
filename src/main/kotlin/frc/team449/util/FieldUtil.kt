package frc.team449.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import frc.team449.Constants
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI

object FieldUtil {

    val BLUE_HUB_TRANSLATION = Translation2d(4.625594, 4.034536)
    var HUB_TRANSLATION = BLUE_HUB_TRANSLATION

    val BLUE_TRENCH_POSES: List<Pose2d> =
        listOf(
            Pose2d(4.35, 0.45, Rotation2d(1.5)),
            Pose2d(4.35, 7.60, Rotation2d(-1.5)),
        )

    val BLUE_TOWER_POSES: List<Pose2d> =
        listOf(
            Pose2d(1.550, 4.022, Rotation2d(0.0)),
        )

    fun getClosestTrenchPose(robotPose: Pose2d): Pose2d {
        val flipRed = DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red

        val allianceTrenchSpots: List<Pose2d> = if (flipRed) {
            BLUE_TRENCH_POSES.map { flipPose(it) }
        } else {
            BLUE_TRENCH_POSES
        }

        return allianceTrenchSpots.minBy {
            it.translation.getDistance(robotPose.translation)
        }
    }

    fun getClosestTowerPose(robotPose : Pose2d) : Pose2d {
        val flipRed = DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red

        val allianceTowerSpots : List<Pose2d> = if (flipRed) {
            BLUE_TOWER_POSES.map { flipPose(it) }
        } else {
            BLUE_TOWER_POSES
        }
        return allianceTowerSpots.minBy {
            it.translation.getDistance(robotPose.translation)
        }
    }

    // flip (wall-blue zero)
    fun flipPose(pose: Pose2d): Pose2d {
        return Pose2d(
            Constants.FieldConstants.FIELD_LENGTH_METERS - pose.x,
            pose.y,
            Rotation2d(PI).minus(pose.rotation)
        )
    }

    fun setHubPosition() {
        val flipRed = DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red

        if (flipRed) {
            HUB_TRANSLATION = flipPose(Pose2d(HUB_TRANSLATION, Rotation2d())).translation
        }
    }
}
