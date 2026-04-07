package frc.team449.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import org.littletonrobotics.junction.AutoLog
import java.nio.ByteBuffer

interface VisionIO {
    @AutoLog
    open class VisionIOInputs {
        @JvmField var connected: Boolean = false

        @JvmField var latestTargetObservation: TargetObservation = TargetObservation(Rotation2d.kZero, Rotation2d.kZero)

        @JvmField var poseObservations: Array<PoseObservation> = emptyArray()

        @JvmField var tagIds = IntArray(0)
    }

    fun updateInputs(inputs: VisionIOInputs) {}
}

data class TargetObservation(
    val tx: Rotation2d,
    val ty: Rotation2d
) : StructSerializable {
    companion object {
        @JvmField
        val struct = object : Struct<TargetObservation> {
            override fun getTypeClass(): Class<TargetObservation> = TargetObservation::class.java

            override fun getTypeName(): String = "TargetObservation"

            override fun getSize(): Int = Rotation2d.struct.size * 2

            override fun getSchema(): String = "Rotation2d tx;Rotation2d ty"

            override fun pack(buffer: ByteBuffer, value: TargetObservation) {
                Rotation2d.struct.pack(buffer, value.tx)
                Rotation2d.struct.pack(buffer, value.ty)
            }

            override fun unpack(buffer: ByteBuffer): TargetObservation {
                val newTx = Rotation2d.struct.unpack(buffer)
                val newTy = Rotation2d.struct.unpack(buffer)
                return TargetObservation(newTx, newTy)
            }
        }
    }
}

data class PoseObservation(
    val timestamp: Double,
    val pose: Pose3d,
    val ambiguity: Double,
    val tagCount: Int,
    val averageTagDistance: Double,
    val type: PoseObservationType
) : StructSerializable {
    companion object {
        @JvmField
        val struct: Struct<PoseObservation> = object : Struct<PoseObservation> {
            override fun getTypeClass(): Class<PoseObservation> = PoseObservation::class.java

            override fun getTypeName(): String = "PoseObservation"

            override fun getSize(): Int = 32 + Pose3d.struct.size

            override fun getSchema(): String = "double timestamp;Pose3d pose;double ambiguity;int32 tagCount;double averageTagDistance;int32 type"

            override fun pack(buffer: ByteBuffer, value: PoseObservation) {
                buffer.putDouble(value.timestamp)
                Pose3d.struct.pack(buffer, value.pose)
                buffer.putDouble(value.ambiguity)
                buffer.putInt(value.tagCount)
                buffer.putDouble(value.averageTagDistance)
                buffer.putInt(value.type.ordinal)
            }

            override fun unpack(buffer: ByteBuffer): PoseObservation {
                val newTimestamp = buffer.double
                val newPose = Pose3d.struct.unpack(buffer)
                val newAmbiguity = buffer.double
                val newCount = buffer.int
                val newAvgDist = buffer.double
                val newType: PoseObservationType = PoseObservationType.entries[buffer.int]
                return PoseObservation(newTimestamp, newPose, newAmbiguity, newCount, newAvgDist, newType)
            }
        }
    }
}

enum class PoseObservationType {
    MEGATAG_1,
    MEGATAG_2,
    PHOTONVISION
}
