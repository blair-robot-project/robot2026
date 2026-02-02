package frc.team449.subsystems.indexer
import org.littletonrobotics.junction.AutoLog

interface IndexerIO {
    @AutoLog
    open class IndexerInputs {

        @JvmField var leftVoltage: Double = 0.0

        @JvmField var rightVoltage: Double = 0.0

        // add to indexer sim or hardware later
        @JvmField var supplyCurrentLeft: Double = 0.0

        @JvmField var statorCurrentLeft: Double = 0.0

        @JvmField var supplyCurrentRight: Double = 0.0

        @JvmField var statorCurrentRight: Double = 0.0
    }
    fun setVoltage(leftVoltage: Double, rightVoltage: Double) {}
    fun updateInputs(inputs: IndexerInputs) {}
}
