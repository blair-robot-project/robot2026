package frc.team449.subsystems.indexer
import org.littletonrobotics.junction.AutoLog

interface IndexerIO {
    @AutoLog
    open class IndexerInputs {
        @JvmField var floorAppliedVolts: Double = 0.0

        @JvmField var floorVelocityRadsPerSec: Double = 0.0

        @JvmField var floorSupplyCurrentAmps: Double = 0.0

        @JvmField var floorStatorCurrentAmps: Double = 0.0

        @JvmField var floorTempCelsius: Double = 0.0

        @JvmField var topAppliedVolts: Double = 0.0

        @JvmField var topVelocityRadsPerSec: Double = 0.0

        @JvmField var topSupplyCurrentAmps: Double = 0.0

        @JvmField var topStatorCurrentAmps: Double = 0.0

        @JvmField var topTempCelsius: Double = 0.0
    }

    fun updateInputs(inputs: IndexerInputs) {}

    fun setIndexerVoltage(floorVolts: Double, topVolts: Double) {}
}
