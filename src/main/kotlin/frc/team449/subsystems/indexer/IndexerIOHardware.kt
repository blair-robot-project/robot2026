package frc.team449.subsystems.indexer

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants

class IndexerIOHardware : IndexerIO {
    private val leftIndexerVoltageRequest = VoltageOut(0.0).withUpdateFreqHz(0.0)
    private val rightIndexerVoltageRequest = VoltageOut(0.0).withUpdateFreqHz(0.0)
    val leftIndexer: TalonFX = TalonFX(INDEXER_ID) // kraken x60
    val rightIndexer: TalonFX = TalonFX(Constants.IndexerConstants.INDEXER_ID_2) // kraken x60
    private var config = TalonFXConfiguration()

    // private val leftVoltage: StatusSignal<Voltage> = leftIndexer.motorVoltage
    // private val rightVoltage: StatusSignal<Voltage> = rightIndexer.motorVoltage
    private val leftIndexerDisconnectedAlert =
        Alert("Left Indexer motor disconnected (ID ${Constants.IndexerConstants.INDEXER_ID})", Alert.AlertType.kError)

    private val rightIndexerDisconnectedAlert =
        Alert("Right Indexer motor disconnected (ID ${Constants.IndexerConstants.INDEXER_ID_2})", Alert.AlertType.kError)

    private val leftIndexerConnected: Boolean
        get() = leftIndexer.isAlive

    private val rightIndexerConnected: Boolean
        get() = rightIndexer.isAlive
    init {
        val indexerMotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive)

        val indexerConfig = TalonFXConfiguration()
            .withMotorOutput(indexerMotorOutput)

        leftIndexer.configurator.apply(indexerConfig)
        rightIndexer.configurator.apply(indexerConfig)
    }

    // 99% percent sure update inputs will need some changes
    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        leftIndexerDisconnectedAlert.set(!leftIndexerConnected)
        rightIndexerDisconnectedAlert.set(!rightIndexerConnected)
    }

    override fun setVoltage(
        leftVoltage: Double,
        rightVoltage: Double
    ) {
        leftIndexer.setControl(leftIndexerVoltageRequest.withOutput(leftVoltage))
        rightIndexer.setControl(rightIndexerVoltageRequest.withOutput(rightVoltage))
    }

    override fun resetPosition() {
        leftIndexer.setPosition(0.0)
        rightIndexer.setPosition(0.0)
    }
}
