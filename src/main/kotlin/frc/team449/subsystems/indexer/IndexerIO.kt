package frc.team449.subsystems.indexer
import edu.wpi.first.units.measure.AngularVelocity
import org.littletonrobotics.junction.AutoLog

interface IndexerIO {
    @AutoLog
    open class IndexerInputs {
        // Top Indexer
        @JvmField var topVoltage: Double = 0.0

        @JvmField var topVelocity: Double = 0.0

        @JvmField var topSupplyCurrent: Double = 0.0

        @JvmField var topStatorCurrent: Double = 0.0

        // Side Indexer
        @JvmField var sideVoltage: Double = 0.0

        @JvmField var sideVelocity: Double = 0.0

        @JvmField var sideSupplyCurrent: Double = 0.0

        @JvmField var sideStatorCurrent: Double = 0.0

        // Bottom Indexer
        @JvmField var bottomVoltage: Double = 0.0

        @JvmField var bottomVelocity: Double = 0.0

        @JvmField var bottomSupplyCurrent: Double = 0.0

        @JvmField var bottomStatorCurrent: Double = 0.0
    }

    fun setVoltage(
        topVoltage: Double,
        sideVoltage: Double,
        bottomVoltage: Double
    ) {}

    fun setIndexerVelocity(
        topVel: AngularVelocity,
        sideVel: AngularVelocity,
        bottomVel: AngularVelocity
    ) {}

    fun simPeriodic() {}

    fun updateInputs(inputs: IndexerInputs) {}
}
