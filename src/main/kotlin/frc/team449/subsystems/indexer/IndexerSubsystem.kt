package frc.team449.subsystems.indexer
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants.IndexerConstants
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class IndexerSubsystem(
    private val io: IndexerIO
) : SubsystemBase() {
    private val inputs: IndexerInputsAutoLogged = IndexerInputsAutoLogged()

    @AutoLogOutput(key = "Indexer/FloorTargetVolts")
    var floorTargetVolts: Double = 0.0
        private set

    @AutoLogOutput(key = "Indexer/TopTargetVolts")
    var topTargetVolts: Double = 0.0
        private set

    private val floorDisconnectedAlert =
        Alert("Floor Indexer Disconnected (ID ${IndexerConstants.FLOOR_ID}).", Alert.AlertType.kError)
    private val topDisconnectedAlert =
        Alert("Top Indexer Disconnected (ID ${IndexerConstants.TOP_ID}).", Alert.AlertType.kError)

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)

        floorDisconnectedAlert.set(!inputs.floorConnected)
        topDisconnectedAlert.set(!inputs.topConnected)

        Logger.recordOutput("Indexer/ActiveCommand", currentCommand?.name ?: "None")
    }

    fun setIndexerVoltageInternal(floorVolts: Double, topVolts: Double) {
        floorTargetVolts = floorVolts
        topTargetVolts = topVolts

        io.setIndexerVoltage(floorTargetVolts, topTargetVolts)
    }

    fun setIndexerVoltage(floorVolts: Double, topVolts: Double): Command =
        runOnce {
            setIndexerVoltageInternal(floorVolts, topVolts)
        }

    fun stop(): Command =
        runOnce {
            floorTargetVolts = 0.0
            topTargetVolts = 0.0
            io.setIndexerVoltage(0.0, 0.0)
        }
}
