package frc.team449.subsystems.indexer

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants.IndexerConstants
import frc.team449.util.PhoenixUtil
import frc.team449.util.PhoenixUtil.tryUntilOk

open class IndexerIOHardware : IndexerIO {
    val floor: TalonFX = TalonFX(IndexerConstants.FLOOR_ID) // x60
    val top: TalonFX = TalonFX(IndexerConstants.TOP_ID) // x44

    val floorVoltageRequest: VoltageOut = VoltageOut(0.0)
    val topVoltageRequest: VoltageOut = VoltageOut(0.0)

    private val floorVoltage = floor.motorVoltage
    private val floorVelocity = floor.velocity
    private val floorSupplyCurrent = floor.supplyCurrent
    private val floorStatorCurrent = floor.statorCurrent
    private val floorTemp = floor.deviceTemp

    private val topVoltage = top.motorVoltage
    private val topVelocity = top.velocity
    private val topSupplyCurrent = top.supplyCurrent
    private val topStatorCurrent = top.statorCurrent
    private val topTemp = top.deviceTemp

    private val lowPrioSignals =
        arrayOf(
            floorTemp,
            topTemp,
        )

    private val highPrioSignals =
        arrayOf(
            floorVoltage,
            floorVelocity,
            floorSupplyCurrent,
            floorStatorCurrent,
            topVoltage,
            topVelocity,
            topSupplyCurrent,
            topStatorCurrent,
        )

    private val floorConnected: Boolean
        get() =
            BaseStatusSignal.isAllGood(
                floorVoltage,
                floorVelocity,
                floorStatorCurrent,
                floorSupplyCurrent,
            )

    private val topConnected: Boolean
        get() =
            BaseStatusSignal.isAllGood(
                topVoltage,
                topVelocity,
                topStatorCurrent,
                topSupplyCurrent,
            )

    private val floorDisconnectedAlert =
        Alert("Floor Indexer Disconnected (ID ${IndexerConstants.FLOOR_ID}).", Alert.AlertType.kError)
    private val topDisconnectedAlert =
        Alert("Top Indexer Disconnected (ID ${IndexerConstants.TOP_ID}).", Alert.AlertType.kError)

    init {
        ParentDevice.optimizeBusUtilizationForAll(floor, top)

        tryUntilOk(5) { floor.configurator.apply(floorConfig) }
        tryUntilOk(5) { top.configurator.apply(topConfig) }

        BaseStatusSignal.setUpdateFrequencyForAll(4.0, *lowPrioSignals)
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, *highPrioSignals)

        PhoenixUtil.registerSignals(*lowPrioSignals, *highPrioSignals)
    }

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        BaseStatusSignal.refreshAll(*lowPrioSignals, *highPrioSignals)

        inputs.floorAppliedVolts = floorVoltage.valueAsDouble
        inputs.floorVelocityRadsPerSec = floorVelocity.value.`in`(Units.RadiansPerSecond)
        inputs.floorSupplyCurrentAmps = floorSupplyCurrent.value.`in`(Units.Amps)
        inputs.floorStatorCurrentAmps = floorStatorCurrent.value.`in`(Units.Amps)
        inputs.floorTempCelsius = floorTemp.value.`in`(Units.Celsius)

        inputs.topAppliedVolts = topVoltage.valueAsDouble
        inputs.topVelocityRadsPerSec = topVelocity.value.`in`(Units.RadiansPerSecond)
        inputs.topSupplyCurrentAmps = topSupplyCurrent.value.`in`(Units.Amps)
        inputs.topStatorCurrentAmps = topStatorCurrent.value.`in`(Units.Amps)
        inputs.topTempCelsius = topTemp.value.`in`(Units.Celsius)

        floorDisconnectedAlert.set(!floorConnected)
        topDisconnectedAlert.set(!topConnected)
    }

    override fun setIndexerVoltage(
        floorVolts: Double,
        topVolts: Double,
    ) {
        floor.setControl(floorVoltageRequest.withOutput(floorVolts))
        top.setControl(topVoltageRequest.withOutput(topVolts))
    }

    companion object {
        val floorConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimit = IndexerConstants.FLOOR_SUPPLY_LIMIT
                    StatorCurrentLimit = IndexerConstants.FLOOR_STATOR_LIMIT
                }

                MotorOutput.apply {
                    NeutralMode = IndexerConstants.FLOOR_NEUTRAL_MODE
                    Inverted = IndexerConstants.FLOOR_INVERSION
                }

                Feedback.SensorToMechanismRatio = IndexerConstants.FLOOR_GEARING
            }

        val topConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimit = IndexerConstants.TOP_SUPPLY_LIMIT
                    StatorCurrentLimit = IndexerConstants.TOP_STATOR_LIMIT
                }

                MotorOutput.apply {
                    NeutralMode = IndexerConstants.TOP_NEUTRAL_MODE
                    Inverted = IndexerConstants.TOP_INVERSION
                }

                Feedback.SensorToMechanismRatio = IndexerConstants.TOP_GEARING
            }
    }
}
