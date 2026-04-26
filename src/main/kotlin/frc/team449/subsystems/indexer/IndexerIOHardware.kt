package frc.team449.subsystems.indexer

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units
import frc.team449.Constants.IndexerConstants
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

    private val indexerSignals =
        arrayOf(
            floorVoltage,
            floorVelocity,
            floorSupplyCurrent,
            floorStatorCurrent,
            floorTemp,
            topVoltage,
            topVelocity,
            topSupplyCurrent,
            topStatorCurrent,
            topTemp,
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

    init {
        tryUntilOk(5) { floor.configurator.apply(floorConfig) }
        tryUntilOk(5) { top.configurator.apply(topConfig) }

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, *indexerSignals)

        ParentDevice.optimizeBusUtilizationForAll(floor, top)
    }

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        BaseStatusSignal.refreshAll(*indexerSignals)

        inputs.floorConnected = floorConnected
        inputs.floorAppliedVolts = floorVoltage.valueAsDouble
        inputs.floorVelocityRadsPerSec = floorVelocity.value.`in`(Units.RadiansPerSecond)
        inputs.floorSupplyCurrentAmps = floorSupplyCurrent.value.`in`(Units.Amps)
        inputs.floorStatorCurrentAmps = floorStatorCurrent.value.`in`(Units.Amps)
        inputs.floorTempCelsius = floorTemp.value.`in`(Units.Celsius)

        inputs.topConnected = topConnected
        inputs.topAppliedVolts = topVoltage.valueAsDouble
        inputs.topVelocityRadsPerSec = topVelocity.value.`in`(Units.RadiansPerSecond)
        inputs.topSupplyCurrentAmps = topSupplyCurrent.value.`in`(Units.Amps)
        inputs.topStatorCurrentAmps = topStatorCurrent.value.`in`(Units.Amps)
        inputs.topTempCelsius = topTemp.value.`in`(Units.Celsius)
    }

    override fun setIndexerVoltage(floorVolts: Double, topVolts: Double) {
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
