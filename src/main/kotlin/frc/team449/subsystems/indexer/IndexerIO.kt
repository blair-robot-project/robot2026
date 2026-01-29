package frc.team449.subsystems.indexer
import au.grapplerobotics.interfaces.LaserCanInterface
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.AutoLog;

interface IndexerIO {
    @AutoLog
    class IndexerInputs{
        var leftVoltage: Double = 0.0
        var rightVoltage: Double = 0.0
        var current: Double = 0.0
        var supply: Double = 0.0
        var motorIsConnected: Boolean = false


    }
    fun setVoltage(leftVoltage: Double, rightVoltage: Double){}
    fun updateInputs(IndexerInputs: IndexerInputs){}
}