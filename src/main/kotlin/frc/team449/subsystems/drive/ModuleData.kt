package frc.team449.subsystems.drive

import edu.wpi.first.util.struct.Struct
import edu.wpi.first.util.struct.StructSerializable
import java.nio.ByteBuffer

data class ModuleData(
    val driveSupplyCurrentAmps: Double = 0.0,
    val driveStatorCurrentAmps: Double = 0.0,
    val steerSupplyCurrentAmps: Double = 0.0,
    val steerStatorCurrentAmps: Double = 0.0
) : StructSerializable {
    companion object {
        @JvmField
        val struct = object : Struct<ModuleData> {
            override fun getTypeClass() = ModuleData::class.java

            override fun getTypeName() = "ModuleData"

            override fun getSize() = 32

            override fun getSchema() = "double driveSupplyCurrentAmps;double driveStatorCurrentAmps;double steerSupplyCurrentAmps;double steerStatorCurrentAmps"

            override fun pack(buffer: ByteBuffer, value: ModuleData) {
                buffer.putDouble(value.driveSupplyCurrentAmps)
                buffer.putDouble(value.driveStatorCurrentAmps)
                buffer.putDouble(value.steerSupplyCurrentAmps)
                buffer.putDouble(value.steerStatorCurrentAmps)
            }

            override fun unpack(buffer: ByteBuffer) =
                ModuleData(
                    buffer.double,
                    buffer.double,
                    buffer.double,
                    buffer.double
                )
        }
    }
}
