package frc.team449.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.util.Color
import frc.team449.Constants
import org.littletonrobotics.junction.Logger
import kotlin.jvm.optionals.getOrNull
import kotlin.math.PI

object FieldUtil {
    val BLUE_HUB_TRANSLATION = Translation2d(4.625594, 4.034536)
    var HUB_TRANSLATION = BLUE_HUB_TRANSLATION
    var distanceToHub = 0.0

    val BLUE_TOWER_POSE: Pose2d = Pose2d(1.470, 4.034, Rotation2d(0.0))
    var TOWER_POSE = BLUE_TOWER_POSE

    val BLUE_TRENCH_POSES: List<Pose2d> =
        listOf(
            Pose2d(4.35, 0.45, Rotation2d(1.5)),
            Pose2d(4.35, 7.60, Rotation2d(-1.5)),
        )

    var autoWinnerLogged = false

    fun initialize() {
        Logger.recordOutput("Auto Winner", Color.kDimGray.toHexString())
    }

    fun getClosestTrenchPose(robotPose: Pose2d): Pose2d {
        val flipRed = DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red

        val allianceTrenchSpots: List<Pose2d> =
            if (flipRed) {
                BLUE_TRENCH_POSES.map { flipPose(it) }
            } else {
                BLUE_TRENCH_POSES
            }

        return allianceTrenchSpots.minBy {
            it.translation.getDistance(robotPose.translation)
        }
    }

    // flip (wall-blue zero)
    fun flipPose(pose: Pose2d): Pose2d =
        Pose2d(
            Constants.FieldConstants.FIELD_LENGTH_METERS - pose.x,
            pose.y,
            Rotation2d(PI).minus(pose.rotation),
        )

    fun updateKeyPositions() {
        val flipRed = DriverStation.getAlliance().getOrNull() == DriverStation.Alliance.Red
        if (flipRed) {
            HUB_TRANSLATION = flipPose(Pose2d(BLUE_HUB_TRANSLATION, Rotation2d())).translation
            TOWER_POSE = flipPose(BLUE_TOWER_POSE)
        } else {
            HUB_TRANSLATION = BLUE_HUB_TRANSLATION
            TOWER_POSE = BLUE_TOWER_POSE
        }
    }

    fun updateAutoWinner(): Boolean {
        val autoWinner = DriverStation.getGameSpecificMessage()

        if (autoWinner.isBlank()) return false

        if (autoWinner == "R") {
            Logger.recordOutput("Auto Winner", Color.kRed.toHexString())
            return true
        }
        if (autoWinner == "B") {
            Logger.recordOutput("Auto Winner", Color.kBlue.toHexString())
            return true
        }

        return false
    }
}
