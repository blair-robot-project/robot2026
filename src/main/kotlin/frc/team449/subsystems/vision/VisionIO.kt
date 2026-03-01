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

        @JvmField var latestTargetObservation = TargetObservation(Rotation2d(), Rotation2d())

        @JvmField var poseObservations: Array<PoseObservation> = emptyArray()

        @JvmField var tagIds: IntArray = IntArray(0)
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

                // two Rotation2d
                override fun getSize(): Int = Rotation2d.struct.size * 2
                override fun getSchema(): String = "Rotation2d tx; Rotation2d ty"
                override fun getNested(): Array<Struct<*>> = arrayOf(Rotation2d.struct)

                override fun pack(bb: ByteBuffer, value: TargetObservation) {
                    Rotation2d.struct.pack(bb, value.tx)
                    Rotation2d.struct.pack(bb, value.ty)
                }

                override fun unpack(bb: ByteBuffer): TargetObservation {
                    return TargetObservation(
                        tx = Rotation2d.struct.unpack(bb),
                        ty = Rotation2d.struct.unpack(bb)
                    )
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
            val struct = object : Struct<PoseObservation> {
                override fun getTypeClass(): Class<PoseObservation> = PoseObservation::class.java
                override fun getTypeName(): String = "PoseObservation"

                // timestamp (8) + pose (Pose3d.size) + ambiguity (8) + tagCount (4) + avgDist (8) + enum ordinal (4)
                override fun getSize(): Int = 8 + Pose3d.struct.size + 8 + 4 + 8 + 4

                // save the Enum as 'int'
                override fun getSchema(): String =
                    "double timestamp; Pose3d pose; double ambiguity; int tagCount; double averageTagDistance; int type"

                override fun getNested(): Array<Struct<*>> = arrayOf(Pose3d.struct)

                override fun pack(bb: ByteBuffer, value: PoseObservation) {
                    bb.putDouble(value.timestamp)
                    Pose3d.struct.pack(bb, value.pose)
                    bb.putDouble(value.ambiguity)
                    bb.putInt(value.tagCount)
                    bb.putDouble(value.averageTagDistance)
                    bb.putInt(value.type.ordinal) // save enum as int
                }

                override fun unpack(bb: ByteBuffer): PoseObservation {
                    return PoseObservation(
                        timestamp = bb.double,
                        pose = Pose3d.struct.unpack(bb),
                        ambiguity = bb.double,
                        tagCount = bb.int,
                        averageTagDistance = bb.double,
                        type = PoseObservationType.entries[bb.int] // restore enum from int
                    )
                }
            }
        }
    }

    enum class PoseObservationType {
        MEGATAG_1,
        MEGATAG_2,
        PHOTONVISION
    }

    fun updateInputs(inputs: VisionIOInputs) {}
}
