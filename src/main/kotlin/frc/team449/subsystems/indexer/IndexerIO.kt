package frc.team449.subsystems.indexer
import org.littletonrobotics.junction.AutoLog

interface IndexerIO {
    @AutoLog
    open class IndexerInputs {
        @JvmField var leftVoltage: Double = 0.0

        @JvmField var rightVoltage: Double = 0.0

        // add to indexer sim or hardware later
        @JvmField var leftSupplyCurrent: Double = 0.0

        @JvmField var leftStatorCurrent: Double = 0.0

        @JvmField var rightSupplyCurrent: Double = 0.0

        @JvmField var rightStatorCurrent: Double = 0.0

        @JvmField var leftVelocity: Double = 0.0

        @JvmField var rightVelocity: Double = 0.0
    }

    fun setVoltage(
        leftVoltage: Double,
        rightVoltage: Double
    ) {}

    fun updateInputs(inputs: IndexerInputs) {}
}
