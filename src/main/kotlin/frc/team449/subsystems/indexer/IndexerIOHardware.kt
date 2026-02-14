package frc.team449.subsystems.indexer

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants.IndexerConstants.BOTTOM_INDEXER_ID
import frc.team449.Constants.IndexerConstants.BOTTOM_INDEXER_STATOR_LIMIT
import frc.team449.Constants.IndexerConstants.BOTTOM_INDEXER_SUPPLY_LIMIT
import frc.team449.Constants.IndexerConstants.SIDE_INDEXER_ID
import frc.team449.Constants.IndexerConstants.SIDE_INDEXER_STATOR_LIMIT
import frc.team449.Constants.IndexerConstants.SIDE_INDEXER_SUPPLY_LIMIT
import frc.team449.Constants.IndexerConstants.TOP_INDEXER_ID
import frc.team449.Constants.IndexerConstants.TOP_INDEXER_STATOR_LIMIT
import frc.team449.Constants.IndexerConstants.TOP_INDEXER_SUPPLY_LIMIT
import frc.team449.util.PhoenixUtil.tryUntilOk

class IndexerIOHardware : IndexerIO {
    private val topIndexerVoltageRequest = VoltageOut(0.0).withUpdateFreqHz(0.0)
    private val sideIndexerVoltageRequest = VoltageOut(0.0).withUpdateFreqHz(0.0)
    private val bottomIndexerVoltageRequest = VoltageOut(0.0).withUpdateFreqHz(0.0)

    val topIndexer: TalonFX = TalonFX(TOP_INDEXER_ID) // kraken x44
    val bottomIndexer: TalonFX = TalonFX(BOTTOM_INDEXER_ID) // kraken x44
    val sideIndexer: TalonFX = TalonFX(SIDE_INDEXER_ID) // kraken x60

    private val topSupplyCurrent: StatusSignal<Current> = topIndexer.supplyCurrent
    private val sideSupplyCurrent: StatusSignal<Current> = sideIndexer.supplyCurrent
    private val bottomSupplyCurrent: StatusSignal<Current> = bottomIndexer.supplyCurrent

    private val topStatorCurrent: StatusSignal<Current> = topIndexer.statorCurrent
    private val sideStatorCurrent: StatusSignal<Current> = sideIndexer.statorCurrent
    private val bottomStatorCurrent: StatusSignal<Current> = bottomIndexer.statorCurrent

    private val topVoltageSignal: StatusSignal<Voltage> = topIndexer.motorVoltage
    private val sideVoltageSignal: StatusSignal<Voltage> = sideIndexer.motorVoltage
    private val bottomVoltageSignal: StatusSignal<Voltage> = bottomIndexer.motorVoltage

    private val topIndexerDisconnectedAlert =
        Alert("Top Indexer motor disconnected (ID $TOP_INDEXER_ID)", Alert.AlertType.kError)

    private val sideIndexerDisconnectedAlert =
        Alert("Side Indexer motor disconnected (ID $SIDE_INDEXER_ID)", Alert.AlertType.kError)

    private val bottomIndexerDisconnectedAlert =
        Alert("Bottom Indexer motor disconnected (ID $BOTTOM_INDEXER_ID)", Alert.AlertType.kError)

    private val topIndexerConnected: Boolean
        get() = topIndexer.isAlive

    private val sideIndexerConnected: Boolean
        get() = sideIndexer.isAlive

    private val bottomIndexerConnected: Boolean
        get() = bottomIndexer.isAlive

    init {
        // make indexer current limit configs for all 3
        val topCurrentLimitConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(TOP_INDEXER_SUPPLY_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(TOP_INDEXER_STATOR_LIMIT)

        val sideCurrentLimitConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(SIDE_INDEXER_SUPPLY_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(SIDE_INDEXER_STATOR_LIMIT)

        val bottomCurrentLimitConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(BOTTOM_INDEXER_SUPPLY_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(BOTTOM_INDEXER_STATOR_LIMIT)

        val topIndexerMotorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        val sideIndexerMotorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        val bottomIndexerMotorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        val topIndexerConfig =
            TalonFXConfiguration()
                .withCurrentLimits(topCurrentLimitConfigs)
                .withMotorOutput(topIndexerMotorOutput)

        val sideIndexerConfig =
            TalonFXConfiguration()
                .withCurrentLimits(sideCurrentLimitConfigs)
                .withMotorOutput(sideIndexerMotorOutput)

        val bottomIndexerConfig =
            TalonFXConfiguration()
                .withCurrentLimits(bottomCurrentLimitConfigs)
                .withMotorOutput(bottomIndexerMotorOutput)

        tryUntilOk(5) { topIndexer.configurator.apply(topIndexerConfig, 0.25) }
        tryUntilOk(5) { sideIndexer.configurator.apply(sideIndexerConfig, 0.25) }
        tryUntilOk(5) { bottomIndexer.configurator.apply(bottomIndexerConfig, 0.25) }

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            topVoltageSignal,
            sideVoltageSignal,
            bottomVoltageSignal,
            topSupplyCurrent,
            sideSupplyCurrent,
            bottomSupplyCurrent,
            topStatorCurrent,
            sideStatorCurrent,
            bottomStatorCurrent,
        )
    }

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        // Refresh all signals first to get synchronized data
        BaseStatusSignal.refreshAll(
            topVoltageSignal,
            sideVoltageSignal,
            bottomVoltageSignal,
            topStatorCurrent,
            sideStatorCurrent,
            bottomStatorCurrent,
            topSupplyCurrent,
            sideSupplyCurrent,
            bottomSupplyCurrent,
        )

        inputs.topVoltage = topVoltageSignal.value.`in`(Volts)
        inputs.topVelocity = topIndexer.velocity.value.`in`(RadiansPerSecond)
        inputs.topStatorCurrent = topStatorCurrent.value.`in`(Amps)
        inputs.topSupplyCurrent = topSupplyCurrent.value.`in`(Amps)

        inputs.sideVoltage = sideVoltageSignal.value.`in`(Volts)
        inputs.sideVelocity = sideIndexer.velocity.value.`in`(RadiansPerSecond)
        inputs.sideStatorCurrent = sideStatorCurrent.value.`in`(Amps)
        inputs.sideSupplyCurrent = sideSupplyCurrent.value.`in`(Amps)

        inputs.bottomVoltage = bottomVoltageSignal.value.`in`(Volts)
        inputs.bottomVelocity = bottomIndexer.velocity.value.`in`(RadiansPerSecond)
        inputs.bottomStatorCurrent = bottomStatorCurrent.value.`in`(Amps)
        inputs.bottomSupplyCurrent = bottomSupplyCurrent.value.`in`(Amps)

        topIndexerDisconnectedAlert.set(!topIndexerConnected)
        sideIndexerDisconnectedAlert.set(!sideIndexerConnected)
        bottomIndexerDisconnectedAlert.set(!bottomIndexerConnected)
    }

    override fun setVoltage(
        topVoltage: Double,
        sideVoltage: Double,
        bottomVoltage: Double
    ) {
        topIndexer.setControl(topIndexerVoltageRequest.withOutput(topVoltage))
        sideIndexer.setControl(sideIndexerVoltageRequest.withOutput(sideVoltage))
        bottomIndexer.setControl(bottomIndexerVoltageRequest.withOutput(bottomVoltage))
    }

    override fun setIndexerVelocity(
        topVel: AngularVelocity,
        sideVel: AngularVelocity,
        bottomVel: AngularVelocity
    ) {
        topIndexer.setControl(VelocityVoltage(topVel))
        sideIndexer.setControl(VelocityVoltage(sideVel))
        bottomIndexer.setControl(VelocityVoltage(bottomVel))
    }
}
