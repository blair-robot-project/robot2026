package frc.team449.subsystems.indexer

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants.IndexerConstants
import frc.team449.util.PhoenixUtil
import frc.team449.util.PhoenixUtil.tryUntilOk

open class IndexerIOHardware : IndexerIO {
    val wedgeIndexer: TalonFX = TalonFX(IndexerConstants.WEDGE_INDEXER_ID) // kraken x44
    val wedgeVelocityRequest: VelocityVoltage = VelocityVoltage(0.0)
    val wedgeVoltageRequest: VoltageOut = VoltageOut(0.0)

    val floorIndexer: TalonFX = TalonFX(IndexerConstants.FLOOR_INDEXER_ID) // kraken x60
    val floorVelocityRequest: VelocityVoltage = VelocityVoltage(0.0)
    val floorVoltageRequest: VoltageOut = VoltageOut(0.0)

    val topIndexer: TalonFX = TalonFX(IndexerConstants.TOP_INDEXER_ID) // kraken x44
    val topVelocityRequest: VelocityVoltage = VelocityVoltage(0.0)
    val topVoltageRequest: VoltageOut = VoltageOut(0.0)

    private val wedgeVelocity = wedgeIndexer.velocity
    private val wedgeVoltage = wedgeIndexer.motorVoltage
    private val wedgeSupplyCurrent = wedgeIndexer.supplyCurrent
    private val wedgeStatorCurrent = wedgeIndexer.statorCurrent
    private val wedgeTemperature = wedgeIndexer.deviceTemp

    private val floorVelocity = floorIndexer.velocity
    private val floorVoltage = floorIndexer.motorVoltage
    private val floorSupplyCurrent = floorIndexer.supplyCurrent
    private val floorStatorCurrent = floorIndexer.statorCurrent
    private val floorTemperature = floorIndexer.deviceTemp

    private val topVelocity = topIndexer.velocity
    private val topVoltage = topIndexer.motorVoltage
    private val topSupplyCurrent = topIndexer.supplyCurrent
    private val topStatorCurrent = topIndexer.statorCurrent
    private val topTemperature = topIndexer.deviceTemp

    private val lowPrioritySignals =
        arrayOf(
            wedgeSupplyCurrent,
            wedgeTemperature,
            floorSupplyCurrent,
            floorTemperature,
            topTemperature,
            topSupplyCurrent,
        )
    private val highPrioritySignals =
        arrayOf(
            wedgeVelocity,
            wedgeVoltage,
            wedgeStatorCurrent,
            floorVelocity,
            floorVoltage,
            floorStatorCurrent,
            topVelocity,
            topVoltage,
            topStatorCurrent,
        )

    private val wedgeMotorConnected: Boolean =
        BaseStatusSignal.isAllGood(
            wedgeVelocity,
            wedgeVoltage,
            wedgeStatorCurrent,
            wedgeSupplyCurrent,
        )

    private val topMotorConnected: Boolean =
        BaseStatusSignal.isAllGood(
            topVelocity,
            topVoltage,
            topStatorCurrent,
            topSupplyCurrent,
        )

    private val floorMotorConnected: Boolean =
        BaseStatusSignal.isAllGood(
            floorVelocity,
            floorVoltage,
            floorStatorCurrent,
            floorSupplyCurrent,
        )

    private val wedgeIndexerDisconnectedAlert =
        Alert("Wedge Indexing Motor Disconnected (ID ${IndexerConstants.WEDGE_INDEXER_ID}).", Alert.AlertType.kError)
    private val floorIndexerDisconnectedAlert =
        Alert("Floor Indexing Motor Disconnected (ID ${IndexerConstants.FLOOR_INDEXER_ID}).", Alert.AlertType.kError)
    private val topIndexerDisconnectedAlert =
        Alert("Top Indexing Motor Disconnected (ID ${IndexerConstants.TOP_INDEXER_ID}).", Alert.AlertType.kError)

    init {
        ParentDevice.optimizeBusUtilizationForAll(wedgeIndexer, floorIndexer, topIndexer)

        tryUntilOk(5) { wedgeIndexer.configurator.apply(leftWedgeConfig) }
        tryUntilOk(5) { floorIndexer.configurator.apply(rightFloorConfig) }
        tryUntilOk(5) { topIndexer.configurator.apply(topFloorConfig) }

        BaseStatusSignal.setUpdateFrequencyForAll(4.0, *lowPrioritySignals)
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, *highPrioritySignals)

        PhoenixUtil.registerSignals(*lowPrioritySignals, *highPrioritySignals)
    }

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        inputs.wedgeVelocityRadPerSec = wedgeVelocity.value.`in`(Units.RadiansPerSecond)
        inputs.wedgeAppliedVolts = wedgeVoltage.valueAsDouble
        inputs.wedgeStatorCurrentAmps = wedgeStatorCurrent.value.`in`(Units.Amps)
        inputs.wedgeSupplyCurrentAmps = wedgeSupplyCurrent.value.`in`(Units.Amps)
        inputs.wedgeTempCelsius = wedgeTemperature.value.`in`(Units.Celsius)

        inputs.floorVelocityRadPerSec = floorVelocity.value.`in`(Units.RadiansPerSecond)
        inputs.floorAppliedVolts = floorVoltage.valueAsDouble
        inputs.floorStatorCurrentAmps = floorStatorCurrent.value.`in`(Units.Amps)
        inputs.floorSupplyCurrentAmps = floorSupplyCurrent.value.`in`(Units.Amps)
        inputs.floorTempCelsius = floorTemperature.value.`in`(Units.Celsius)

        inputs.topVelocityRadPerSec = topVelocity.value.`in`(Units.RadiansPerSecond)
        inputs.topAppliedVolts = topVoltage.valueAsDouble
        inputs.topStatorCurrentAmps = topStatorCurrent.value.`in`(Units.Amps)
        inputs.topSupplyCurrentAmps = topSupplyCurrent.value.`in`(Units.Amps)
        inputs.topTempCelsius = topTemperature.value.`in`(Units.Celsius)

        wedgeIndexerDisconnectedAlert.set(!wedgeMotorConnected)
        floorIndexerDisconnectedAlert.set(!floorMotorConnected)
        topIndexerDisconnectedAlert.set(!topMotorConnected)
    }

    override fun setFloorSpeed(floorSurfaceSpeed: AngularVelocity) {
        floorIndexer.setControl(floorVelocityRequest.withVelocity(floorSurfaceSpeed))
    }

    override fun setWedgeSpeed(wedgeSurfaceSpeed: AngularVelocity) {
        wedgeIndexer.setControl(wedgeVelocityRequest.withVelocity(wedgeSurfaceSpeed))
    }

    override fun setTopSpeed(topSurfaceSpeed: AngularVelocity) {
        topIndexer.setControl(topVelocityRequest.withVelocity(topSurfaceSpeed))
    }

    override fun setIndexerVoltage(
        floorVolts: Double,
        wedgeVolts: Double,
        topVolts: Double
    ) {
        floorIndexer.setControl(floorVoltageRequest.withOutput(floorVolts))
        wedgeIndexer.setControl(wedgeVoltageRequest.withOutput(wedgeVolts))
        topIndexer.setControl(topVoltageRequest.withOutput(topVolts))
    }

    override fun setSupplyLimits(
        floorSupplyLimitAmps: Double,
        wedgeSupplyLimitAmps: Double,
        topSupplyLimitAmps: Double
    ) {
        val floorCurrentConfig = CurrentLimitsConfigs()
            .withSupplyCurrentLimit(floorSupplyLimitAmps)
        val wedgeCurrentConfig = CurrentLimitsConfigs()
            .withSupplyCurrentLimit(wedgeSupplyLimitAmps)
        val topCurrentConfig = CurrentLimitsConfigs()
            .withSupplyCurrentLimit(topSupplyLimitAmps)

        floorIndexer.configurator.apply(floorCurrentConfig, 0.0)
        wedgeIndexer.configurator.apply(wedgeCurrentConfig, 0.0)
        topIndexer.configurator.apply(topCurrentConfig, 0.0)
    }

    companion object {
        val leftWedgeConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimit = IndexerConstants.WEDGE_SUPPLY_LIMIT
                    StatorCurrentLimit = IndexerConstants.WEDGE_STATOR_LIMIT
                }

                MotorOutput.apply {
                    NeutralMode = IndexerConstants.WEDGE_NEUTRAL_MODE
                    Inverted = IndexerConstants.WEDGE_INVERSION
                }

                Feedback.SensorToMechanismRatio = IndexerConstants.WEDGE_GEARING

                Slot0.apply {
                    kP = IndexerConstants.WEDGE_KP
                    kI = IndexerConstants.WEDGE_KI
                    kD = IndexerConstants.WEDGE_KD
                    kS = IndexerConstants.WEDGE_KS
                    kV = IndexerConstants.WEDGE_KV
                }
            }

        val rightFloorConfig =
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

                Slot0.apply {
                    kP = IndexerConstants.FLOOR_KP
                    kI = IndexerConstants.FLOOR_KI
                    kD = IndexerConstants.FLOOR_KD
                    kS = IndexerConstants.FLOOR_KS
                    kV = IndexerConstants.FLOOR_KV
                }
            }

        val topFloorConfig =
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

                Slot0.apply {
                    kP = IndexerConstants.TOP_KP
                    kI = IndexerConstants.TOP_KI
                    kD = IndexerConstants.TOP_KD
                    kS = IndexerConstants.TOP_KS
                    kV = IndexerConstants.TOP_KV
                }
            }
    }
}
