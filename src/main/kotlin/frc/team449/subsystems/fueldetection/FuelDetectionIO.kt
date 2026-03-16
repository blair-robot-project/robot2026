package frc.team449.subsystems.fueldetection

import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import org.littletonrobotics.junction.AutoLog
import java.nio.ByteBuffer

interface FuelDetectionIO {
    @AutoLog
    open class FuelDetectionIOInputs {
        @JvmField var connected: Boolean = false

        @JvmField var detections: Array<FuelDetection> = emptyArray()

        @JvmField var pipelineLatencyMs: Double = 0.0

        @JvmField var captureLatencyMs: Double = 0.0
    }

    data class FuelDetection(
        val tx: Double,
        val ty: Double,
        val ta: Double,
        val confidence: Double
    ) : StructSerializable {
        companion object {
            @JvmField
            val struct = object : Struct<FuelDetection> {
                override fun getTypeClass(): Class<FuelDetection> = FuelDetection::class.java
                override fun getTypeName(): String = "FuelDetection"
                override fun getSize(): Int = 8 * 4
                override fun getSchema(): String = "double tx; double ty; double ta; double confidence"
                override fun getNested(): Array<Struct<*>> = emptyArray()

                override fun pack(bb: ByteBuffer, value: FuelDetection) {
                    bb.putDouble(value.tx)
                    bb.putDouble(value.ty)
                    bb.putDouble(value.ta)
                    bb.putDouble(value.confidence)
                }

                override fun unpack(bb: ByteBuffer): FuelDetection {
                    return FuelDetection(
                        tx = bb.double,
                        ty = bb.double,
                        ta = bb.double,
                        confidence = bb.double
                    )
                }
            }
        }
    }

    fun updateInputs(inputs: FuelDetectionIOInputs) {}

    fun setPipeline(index: Int) {}
}
