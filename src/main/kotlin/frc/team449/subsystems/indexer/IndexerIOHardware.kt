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
import frc.team449.Constants
import frc.team449.Constants.IndexerConstants
import frc.team449.util.PhoenixUtil.tryUntilOk
import jdk.jshell.Snippet

open class IndexerIOHardware : IndexerIO {
    private val voltageRequest = VoltageOut(0.0)
        .withUpdateFreqHz(IndexerConstants.REQUEST_UPDATE_FREQ_HZ)
        .withEnableFOC(false)


    private val velocityRequest = VelocityVoltage(0.0)
        .withUpdateFreqHz(IndexerConstants.REQUEST_UPDATE_FREQ_HZ)
        .withSlot(0)
        .withEnableFOC(false)

    val topIndexerMotor: TalonFX = TalonFX(IndexerConstants.TOP_INDEXER_ID) // kraken x44
    val bottomIndexerMotor: TalonFX = TalonFX(IndexerConstants.BOTTOM_INDEXER_ID) // kraken x44
    val sideIndexerMotor: TalonFX = TalonFX(IndexerConstants.SIDE_INDEXER_ID) // kraken x60

    private val topSupplyCurrent: StatusSignal<Current> = topIndexerMotor.supplyCurrent
    private val sideSupplyCurrent: StatusSignal<Current> = sideIndexerMotor.supplyCurrent
    private val bottomSupplyCurrent: StatusSignal<Current> = bottomIndexerMotor.supplyCurrent

    private val topStatorCurrent: StatusSignal<Current> = topIndexerMotor.statorCurrent
    private val sideStatorCurrent: StatusSignal<Current> = sideIndexerMotor.statorCurrent
    private val bottomStatorCurrent: StatusSignal<Current> = bottomIndexerMotor.statorCurrent

    private val topVelocity: StatusSignal<AngularVelocity> = topIndexerMotor.velocity
    private val sideVelocity: StatusSignal<AngularVelocity> = sideIndexerMotor.velocity
    private val bottomVelocity: StatusSignal<AngularVelocity> = bottomIndexerMotor.velocity

    private val topVelocitySetpoint: StatusSignal<Double> = topIndexerMotor.closedLoopReference
    private val sideVelocitySetpoint: StatusSignal<Double> = sideIndexerMotor.closedLoopReference
    private val bottomVelocitySetpoint: StatusSignal<Double> = bottomIndexerMotor.closedLoopReference

    private val topVoltageSignal: StatusSignal<Voltage> = topIndexerMotor.motorVoltage
    private val sideVoltageSignal: StatusSignal<Voltage> = sideIndexerMotor.motorVoltage
    private val bottomVoltageSignal: StatusSignal<Voltage> = bottomIndexerMotor.motorVoltage

    private val topIndexerDisconnectedAlert =
        Alert("Top Indexer motor disconnected (ID ${IndexerConstants.TOP_INDEXER_ID})", Alert.AlertType.kError)

    private val sideIndexerDisconnectedAlert =
        Alert("Side Indexer motor disconnected (ID ${IndexerConstants.SIDE_INDEXER_ID})", Alert.AlertType.kError)

    private val bottomIndexerDisconnectedAlert =
        Alert("Bottom Indexer motor disconnected (ID ${IndexerConstants.BOTTOM_INDEXER_ID})", Alert.AlertType.kError)

    private val topIndexerConnected: Boolean
        get() = topIndexerMotor.isAlive

    private val sideIndexerConnected: Boolean
        get() = sideIndexerMotor.isAlive

    private val bottomIndexerConnected: Boolean
        get() = bottomIndexerMotor.isAlive

    init {
        // make indexer current limit configs for all 3
        val topCurrentLimitConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IndexerConstants.TOP_INDEXER_SUPPLY_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(IndexerConstants.TOP_INDEXER_STATOR_LIMIT)

        val sideCurrentLimitConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IndexerConstants.SIDE_INDEXER_SUPPLY_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(IndexerConstants.SIDE_INDEXER_STATOR_LIMIT)

        val bottomCurrentLimitConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(IndexerConstants.BOTTOM_INDEXER_SUPPLY_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(IndexerConstants.BOTTOM_INDEXER_STATOR_LIMIT)

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

        tryUntilOk(5) { topIndexerMotor.configurator.apply(topIndexerConfig, 0.25) }
        tryUntilOk(5) { sideIndexerMotor.configurator.apply(sideIndexerConfig, 0.25) }
        tryUntilOk(5) { bottomIndexerMotor.configurator.apply(bottomIndexerConfig, 0.25) }

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
        inputs.topVelocity = topVelocity.value.`in`(RadiansPerSecond)
        inputs.topSetpoint = topVelocitySetpoint.value
        inputs.topStatorCurrent = topStatorCurrent.value.`in`(Amps)
        inputs.topSupplyCurrent = topSupplyCurrent.value.`in`(Amps)

        inputs.sideVoltage = sideVoltageSignal.value.`in`(Volts)
        inputs.sideVelocity = sideVelocity.value.`in`(RadiansPerSecond)
        inputs.sideSetpoint = sideVelocitySetpoint.value
        inputs.sideStatorCurrent = sideStatorCurrent.value.`in`(Amps)
        inputs.sideSupplyCurrent = sideSupplyCurrent.value.`in`(Amps)

        inputs.bottomVoltage = bottomVoltageSignal.value.`in`(Volts)
        inputs.bottomVelocity = bottomVelocity.value.`in`(RadiansPerSecond)
        inputs.sideSetpoint = sideVelocitySetpoint.value
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
        topIndexerMotor.setControl(voltageRequest.withOutput(topVoltage))
        sideIndexerMotor.setControl(voltageRequest.withOutput(sideVoltage))
        bottomIndexerMotor.setControl(voltageRequest.withOutput(bottomVoltage))
    }

    override fun setIndexerVelocity(
        topVel: AngularVelocity,
        sideVel: AngularVelocity,
        bottomVel: AngularVelocity
    ) {
        topIndexerMotor.setControl(velocityRequest.withVelocity(topVel))
        sideIndexerMotor.setControl(velocityRequest.withVelocity(sideVel))
        bottomIndexerMotor.setControl(velocityRequest.withVelocity(bottomVel))
    }
}
