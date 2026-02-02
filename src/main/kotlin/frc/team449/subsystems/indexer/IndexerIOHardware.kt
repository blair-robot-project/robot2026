package frc.team449.subsystems.indexer

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants
import frc.team449.Constants.IndexerConstants.INDEXER_STATOR_LIMIT
import frc.team449.Constants.IndexerConstants.INDEXER_SUPPLY_LIMIT
import frc.team449.Constants.IndexerConstants.LEFT_INDEXER_ID
import frc.team449.Constants.IndexerConstants.RIGHT_INDEXER_ID
import frc.team449.util.PhoenixUtil.tryUntilOk


class IndexerIOHardware : IndexerIO {
    private val leftIndexerVoltageRequest = VoltageOut(0.0).withUpdateFreqHz(0.0)
    private val rightIndexerVoltageRequest = VoltageOut(0.0).withUpdateFreqHz(0.0)
    val leftIndexer: TalonFX = TalonFX(LEFT_INDEXER_ID) // kraken x60
    val rightIndexer: TalonFX = TalonFX(Constants.IndexerConstants.RIGHT_INDEXER_ID) // kraken x44

    private val leftSupplyCurrent: StatusSignal<Current> = leftIndexer.supplyCurrent
    private val rightSupplyCurrent: StatusSignal<Current> = rightIndexer.supplyCurrent

    private val leftStatorCurrent: StatusSignal<Current> = leftIndexer.statorCurrent
    private val rightStatorCurrent: StatusSignal<Current> = rightIndexer.statorCurrent

    private val leftVoltageSignal: StatusSignal<Voltage> = leftIndexer.motorVoltage
    private val rightVoltageSignal: StatusSignal<Voltage> = rightIndexer.motorVoltage
    private val leftIndexerDisconnectedAlert =
        Alert("Left Indexer motor disconnected (ID ${LEFT_INDEXER_ID})", Alert.AlertType.kError)

    private val rightIndexerDisconnectedAlert =
        Alert("Right Indexer motor disconnected (ID ${RIGHT_INDEXER_ID})", Alert.AlertType.kError)

    private val leftIndexerConnected: Boolean
        get() = leftIndexer.isAlive

    private val rightIndexerConnected: Boolean
        get() = rightIndexer.isAlive
    init {
        val indexerCurrentLimitConfigs = CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(INDEXER_SUPPLY_LIMIT)//change this in constants cus idk what it's suppsoed to be
            .withStatorCurrentLimitEnable(true)
            .withStatorCurrentLimit(INDEXER_STATOR_LIMIT)


        val indexerMotorOutput = MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive)

        val indexerConfig = TalonFXConfiguration()
            .withCurrentLimits(indexerCurrentLimitConfigs)
            .withMotorOutput(indexerMotorOutput)

        tryUntilOk(5) { leftIndexer.configurator.apply(indexerConfig, 0.25) }
        tryUntilOk(5) { rightIndexer.configurator.apply(indexerConfig, 0.25) }

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            leftVoltageSignal,
            rightVoltageSignal,
            leftSupplyCurrent,
            rightSupplyCurrent,
            leftStatorCurrent,
            rightStatorCurrent
        )
    }

    // 99% percent sure update inputs will need some changes
    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        //error
        inputs.leftVoltage = leftIndexer.motorVoltage.value.`in`(Units.Volts)
        inputs.rightVoltage = rightIndexer.motorVoltage.value.`in`(Units.Volts)

        inputs.statorCurrentLeft = leftIndexer.statorCurrent.value.`in`(Units.Amps)
        inputs.supplyCurrentLeft = leftIndexer.supplyCurrent.value.`in`(Units.Amps)

        inputs.statorCurrentRight = rightIndexer.statorCurrent.value.`in`(Units.Amps)
        inputs.supplyCurrentRight = rightIndexer.supplyCurrent.value.`in`(Units.Amps)

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
}
