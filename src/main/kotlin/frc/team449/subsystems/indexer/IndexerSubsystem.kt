package frc.team449.subsystems.indexer
import com.sun.org.apache.xalan.internal.lib.ExsltMath.abs
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class IndexerSubsystem(
    private val io: IndexerIO
) : SubsystemBase() {
    private val inputs: IndexerInputsAutoLogged = IndexerInputsAutoLogged()

    var floorTargetVolts: Double = 0.0
        private set
    var topTargetVolts: Double = 0.0
        private set

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)

        Logger.recordOutput("Indexer/FloorTargetVolts", floorTargetVolts)
        Logger.recordOutput("Indexer/TopTargetVolts", topTargetVolts)
        Logger.recordOutput("Indexer/ActiveCommand", currentCommand?.name ?: "None")
    }

    fun setIndexerVoltage(
        floorVolts: Double,
        topVolts: Double
    ): Command =
        runOnce {
            floorTargetVolts = floorVolts
            topTargetVolts = topVolts

            io.setIndexerVoltage(floorTargetVolts, topTargetVolts)
        }
            .withName("VOLTAGE")

    fun stop(): Command =
        runOnce {
            floorTargetVolts = 0.0
            topTargetVolts = 0.0
            io.setIndexerVoltage(0.0, 0.0)
        }
            .withName("STOP")
    fun indexerAtTolerance(): Boolean {
        val floor = abs(floorTargetVolts - floorTargetVolts)
        val top = abs(topTargetVolts - topTargetVolts)

        val isAtTarget = floor < 1.0 && top < 1.0

        return isAtTarget
    }
}
