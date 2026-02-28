package frc.team449.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import org.littletonrobotics.junction.AutoLog

interface VisionIO {
    @AutoLog
    open class VisionIOInputs {
        @JvmField var connected: Boolean = false

        @JvmField var latestTargetObservation = TargetObservation(Rotation2d(), Rotation2d())

        @JvmField var poseObservations: Array<PoseObservation> = emptyArray()

        @JvmField var tagIds: IntArray = IntArray(0)

        @JvmField var latestTargetObservationPhoton = TargetObservation(Rotation2d(), Rotation2d())
    }

    data class TargetObservation(
        val tx: Rotation2d,
        val ty: Rotation2d
    )

    data class PoseObservation(
        val timestamp: Double,
        val pose: Pose3d,
        val ambiguity: Double,
        val tagCount: Int,
        val averageTagDistance: Double,
        val type: PoseObservationType
    )

    enum class PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

    fun updateInputs(inputs: VisionIOInputs) {}
}
