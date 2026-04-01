package frc.team449.util

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.util.Color
import frc.team449.Constants.FieldConstants
import org.littletonrobotics.junction.Logger
import kotlin.math.PI

object FieldUtil {
    val isRed: Boolean get() = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red

    private val BLUE_HUB: Translation2d = FieldConstants.BLUE_HUB
    private val RED_HUB: Translation2d = BLUE_HUB.flipped()
    val HUB: Translation2d get() = if (isRed) RED_HUB else BLUE_HUB

    private val BLUE_TRENCHES: Array<Pose2d> = FieldConstants.BLUE_TRENCHES
    private val RED_TRENCHES: Array<Pose2d> = Array(BLUE_TRENCHES.size) { i -> BLUE_TRENCHES[i].flipped() }
    val TRENCHES: Array<Pose2d> get() = if (isRed) RED_TRENCHES else BLUE_TRENCHES

    private val BLUE_PASSES: Array<Translation2d> = FieldConstants.BLUE_PASSES
    private val RED_PASSES: Array<Translation2d> = Array(BLUE_PASSES.size) { i -> BLUE_PASSES[i].flipped() }
    val PASSES: Array<Translation2d> get() = if (isRed) RED_PASSES else BLUE_PASSES

    var autoWinnerLogged = false

    fun getDistanceToFriendlyHub(robotPose: Pose2d): Double = robotPose.translation.getDistance(HUB)
    fun getClosestFriendlyTrench(robotPose: Pose2d): Pose2d = TRENCHES.minBy { it.translation.getDistance(robotPose.translation) }
    fun getClosestFriendlyPass(robotPose: Pose2d): Translation2d = PASSES.minBy { it.getDistance(robotPose.translation) }

    // flip (wall-blue zero)
    fun Pose2d.flipped(): Pose2d = Pose2d(
        FieldConstants.FIELD_LENGTH_METERS - x,
        y,
        Rotation2d(PI).minus(rotation)
    )

    fun Translation2d.flipped(): Translation2d = Translation2d(
        FieldConstants.FIELD_LENGTH_METERS - x,
        y
    )

    fun initializeAutoWinnerField() {
        Logger.recordOutput("Auto Winner", Color.kDimGray.toHexString())
    }

    fun updateAutoWinner(): Boolean {
        val autoWinner = DriverStation.getGameSpecificMessage()
        if (autoWinner.isBlank()) return false

        val statusColor = when (autoWinner.uppercase()) {
            "R" -> Color.kRed
            "B" -> Color.kBlue
            else -> Color.kDimGray
        }

        Logger.recordOutput("Auto Winner", statusColor.toHexString())
        return autoWinner.isNotBlank()
    }
}
