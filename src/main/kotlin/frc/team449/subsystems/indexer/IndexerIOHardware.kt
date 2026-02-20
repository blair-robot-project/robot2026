package frc.team449.subsystems.indexer

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants.IndexerConstants
import frc.team449.Constants.IndexerConstants.FLOOR_INDEXER_ID
import frc.team449.Constants.IndexerConstants.TOP_INDEXER_ID
import frc.team449.Constants.IndexerConstants.WEDGE_INDEXER_ID
import frc.team449.util.PhoenixUtil
import frc.team449.util.PhoenixUtil.tryUntilOk

open class IndexerIOHardware : IndexerIO {
    val wedgeIndexer: TalonFX = TalonFX(WEDGE_INDEXER_ID) // kraken x44
    val wedgeControlRequest: VelocityVoltage = VelocityVoltage(0.0).withUpdateFreqHz(0.0)

    val floorIndexer: TalonFX = TalonFX(FLOOR_INDEXER_ID) // kraken x60
    val floorControlRequest: VelocityVoltage = VelocityVoltage(0.0).withUpdateFreqHz(0.0)

    val topIndexer: TalonFX = TalonFX(TOP_INDEXER_ID) // kraken x44
    val topControlRequest: VelocityVoltage = VelocityVoltage(0.0).withUpdateFreqHz(0.0)

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
            wedgeStatorCurrent,
            wedgeTemperature,
            floorSupplyCurrent,
            floorStatorCurrent,
            floorTemperature,
            topStatorCurrent,
            topTemperature,
            topSupplyCurrent,
        )
    private val highPrioritySignals =
        arrayOf(
            wedgeVelocity,
            wedgeVoltage,
            floorVelocity,
            floorVoltage,
            topVelocity,
            topVoltage,
        )

    private val wedgeIndexerDisconnectedAlert =
        Alert("Wedge indexing motor disconnected (ID $WEDGE_INDEXER_ID)", Alert.AlertType.kError)
    private val floorIndexerDisconnectedAlert =
        Alert("Floor indexing motor disconnected (ID $FLOOR_INDEXER_ID)", Alert.AlertType.kError)

    private val topIndexerDisconnectedAlert =
        Alert("Top indexing motor disconnected (ID $TOP_INDEXER_ID)", Alert.AlertType.kError)

    init {
        ParentDevice.optimizeBusUtilizationForAll(wedgeIndexer, floorIndexer)

        tryUntilOk(5) {
            wedgeIndexer.configurator.apply(leftWedgeConfig)
        }

        tryUntilOk(5) {
            floorIndexer.configurator.apply(rightFloorConfig)
        }

        tryUntilOk(5) {
            topIndexer.configurator.apply(topFloorConfig)
        }
        BaseStatusSignal.setUpdateFrequencyForAll(4.0, *lowPrioritySignals)
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, *highPrioritySignals)

        PhoenixUtil.registerSignals(*lowPrioritySignals, *highPrioritySignals)
    }

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        inputs.wedgeVelocityRadPerSec = wedgeIndexer.velocity.value.`in`(Units.RadiansPerSecond)
        inputs.wedgeAppliedVolts = wedgeIndexer.motorVoltage.valueAsDouble
        inputs.wedgeStatorCurrentAmps = wedgeIndexer.statorCurrent.value.`in`(Units.Amps)
        inputs.wedgeSupplyCurrentAmps = wedgeIndexer.supplyCurrent.value.`in`(Units.Amps)
        inputs.wedgeTempCelsius = wedgeIndexer.deviceTemp.value.`in`(Units.Celsius)
        wedgeIndexerDisconnectedAlert.set(!wedgeIndexer.isAlive)

        inputs.floorVelocityRadPerSec = floorIndexer.velocity.value.`in`(Units.RadiansPerSecond)
        inputs.floorAppliedVolts = floorIndexer.motorVoltage.valueAsDouble
        inputs.floorStatorCurrentAmps = floorIndexer.statorCurrent.value.`in`(Units.Amps)
        inputs.floorSupplyCurrentAmps = floorIndexer.supplyCurrent.value.`in`(Units.Amps)
        inputs.floorTempCelsius = floorIndexer.deviceTemp.value.`in`(Units.Celsius)
        floorIndexerDisconnectedAlert.set(!floorIndexer.isAlive)

        inputs.topVelocityRadPerSec = topIndexer.velocity.value.`in`(Units.RadiansPerSecond)
        inputs.topAppliedVolts = topIndexer.motorVoltage.valueAsDouble
        inputs.topStatorCurrentAmps = topIndexer.statorCurrent.value.`in`(Units.Amps)
        inputs.topSupplyCurrentAmps = topIndexer.supplyCurrent.value.`in`(Units.Amps)
        inputs.topTempCelsius = topIndexer.deviceTemp.value.`in`(Units.Celsius)
        topIndexerDisconnectedAlert.set(!topIndexer.isAlive)
    }

    override fun setFloorSpeed(floorSurfaceSpeed: AngularVelocity) {
        floorIndexer.setControl(
            floorControlRequest.withVelocity(
                (floorSurfaceSpeed.`in`(RotationsPerSecond)),
            ),
        )
    }

    override fun setWedgeSpeed(wedgeSurfaceSpeed: AngularVelocity) {
        wedgeIndexer.setControl(
            wedgeControlRequest.withVelocity(wedgeSurfaceSpeed.`in`(RotationsPerSecond)),
        )
    }

    override fun setTopSpeed(topSurfaceSpeed: AngularVelocity) {
        topIndexer.setControl(
            topControlRequest.withVelocity(topSurfaceSpeed.`in`(RotationsPerSecond)),
        )
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
