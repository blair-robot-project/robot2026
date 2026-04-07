package frc.team449.subsystems.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants.IntakeConstants
import frc.team449.util.PhoenixUtil
import frc.team449.util.PhoenixUtil.tryUntilOk

open class IntakeIOHardware : IntakeIO {
    val leftPivot = TalonFX(IntakeConstants.LEFT_PIVOT_ID)
    val rightPivot = TalonFX(IntakeConstants.RIGHT_PIVOT_ID)
    val leftRollerLeader = TalonFX(IntakeConstants.LEFT_ROLLER_LEADER_ID)
    val rightRollerFollower = TalonFX(IntakeConstants.RIGHT_ROLLER_FOLLOWER_ID)

    private val pivotVoltageRequest = VoltageOut(0.0)
    private val pivotPositionRequest = PositionVoltage(0.0)
    private val rollerVoltageRequest = VoltageOut(0.0)

    private val leftPivotVoltage = leftPivot.motorVoltage
    private val leftPivotPosition = leftPivot.position
    private val leftPivotVelocity = leftPivot.velocity
    private val leftPivotSupplyCurrent = leftPivot.supplyCurrent
    private val leftPivotStatorCurrent = leftPivot.statorCurrent
    private val leftPivotTemp = leftPivot.deviceTemp

    private val rightPivotVoltage = rightPivot.motorVoltage
    private val rightPivotPosition = rightPivot.position
    private val rightPivotVelocity = rightPivot.velocity
    private val rightPivotSupplyCurrent = rightPivot.supplyCurrent
    private val rightPivotStatorCurrent = rightPivot.statorCurrent
    private val rightPivotTemp = rightPivot.deviceTemp

    private val leftRollerLeaderVoltage = leftRollerLeader.motorVoltage
    private val leftRollerLeaderVelocity = leftRollerLeader.velocity
    private val leftRollerLeaderSupplyCurrent = leftRollerLeader.supplyCurrent
    private val leftRollerLeaderStatorCurrent = leftRollerLeader.statorCurrent
    private val leftRollerLeaderTemp = leftRollerLeader.deviceTemp

    private val rightRollerFollowerVoltage = rightRollerFollower.motorVoltage
    private val rightRollerFollowerVelocity = rightRollerFollower.velocity
    private val rightRollerFollowerSupplyCurrent = rightRollerFollower.supplyCurrent
    private val rightRollerFollowerStatorCurrent = rightRollerFollower.statorCurrent
    private val rightRollerFollowerTemp = rightRollerFollower.deviceTemp

    private val lowPrioSignals =
        arrayOf(
            leftPivotTemp,
            rightPivotTemp,
            leftRollerLeaderTemp,
            rightRollerFollowerVoltage,
            rightRollerFollowerVelocity,
            rightRollerFollowerSupplyCurrent,
            rightRollerFollowerStatorCurrent,
            rightRollerFollowerTemp,
        )

    private val highPrioSignals =
        arrayOf(
            leftPivotVoltage,
            leftPivotPosition,
            leftPivotVelocity,
            leftPivotSupplyCurrent,
            leftPivotStatorCurrent,
            rightPivotVoltage,
            rightPivotPosition,
            rightPivotVelocity,
            rightPivotSupplyCurrent,
            rightPivotStatorCurrent,
            leftRollerLeaderVoltage,
            leftRollerLeaderVelocity,
            leftRollerLeaderSupplyCurrent,
            leftRollerLeaderStatorCurrent,
        )

    private val leftPivotConnected: Boolean
        get() = BaseStatusSignal.isAllGood(
            leftPivotVoltage,
            leftPivotPosition,
            leftPivotVelocity,
            leftPivotSupplyCurrent,
            leftPivotStatorCurrent,
        )

    private val rightPivotConnected: Boolean
        get() = BaseStatusSignal.isAllGood(
            rightPivotVoltage,
            rightPivotPosition,
            rightPivotVelocity,
            rightPivotSupplyCurrent,
            rightPivotStatorCurrent,
        )

    private val leftRollerConnected: Boolean
        get() = BaseStatusSignal.isAllGood(
            leftRollerLeaderVoltage,
            leftRollerLeaderSupplyCurrent,
            leftRollerLeaderStatorCurrent,
        )

    private val rightRollerConnected: Boolean
        get() = BaseStatusSignal.isAllGood(
            rightRollerFollowerVoltage,
            rightRollerFollowerVelocity,
            rightRollerFollowerSupplyCurrent,
            rightRollerFollowerStatorCurrent,
        )

    val leftPivotLeaderDisconnectedAlert =
        Alert("Left Pivot Disconnected (ID ${IntakeConstants.LEFT_PIVOT_ID}).", Alert.AlertType.kError)
    val rightPivotFollowerDisconnectedAlert =
        Alert("Right Pivot Disconnected (ID ${IntakeConstants.RIGHT_PIVOT_ID}).", Alert.AlertType.kError)
    val leftRollerLeaderDisconnectedAlert =
        Alert("Left Roller Disconnected (ID ${IntakeConstants.LEFT_ROLLER_LEADER_ID}).", Alert.AlertType.kError)
    val rightRollerFollowerDisconnectedAlert =
        Alert("Right Roller Disconnected (ID ${IntakeConstants.RIGHT_ROLLER_FOLLOWER_ID}).", Alert.AlertType.kError)

    init {
        ParentDevice.optimizeBusUtilizationForAll(leftPivot, rightPivot, leftRollerLeader, rightRollerFollower)

        tryUntilOk(5) { leftPivot.configurator.apply(leftPivotConfig, 0.25) }
        tryUntilOk(5) { rightPivot.configurator.apply(rightPivotConfig, 0.25) }
        tryUntilOk(5) { leftRollerLeader.configurator.apply(rollerConfig, 0.25) }
        tryUntilOk(5) { rightRollerFollower.configurator.apply(rollerConfig, 0.25) }

        rightRollerFollower.setControl(Follower(leftRollerLeader.deviceID, IntakeConstants.RIGHT_ROLLER_FOLLOWER_ALIGNMENT))

        BaseStatusSignal.setUpdateFrequencyForAll(4.0, *lowPrioSignals)
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, *highPrioSignals)

        PhoenixUtil.registerSignals(*lowPrioSignals, *highPrioSignals)
    }

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        BaseStatusSignal.refreshAll(*lowPrioSignals, *highPrioSignals)

        inputs.leftPivotAppliedVolts = leftPivotVoltage.value.`in`(Units.Volts)
        inputs.leftPivotPositionRads = leftPivotPosition.value.`in`(Units.Radians)
        inputs.leftPivotVelocityRadsPerSec = leftPivotVelocity.value.`in`(Units.RadiansPerSecond)
        inputs.leftPivotSupplyCurrentAmps = leftPivotSupplyCurrent.value.`in`(Units.Amps)
        inputs.leftPivotStatorCurrentAmps = leftPivotStatorCurrent.value.`in`(Units.Amps)
        inputs.leftPivotTempCelsius = leftPivotTemp.value.`in`(Units.Celsius)

        inputs.rightPivotAppliedVolts = rightPivotVoltage.value.`in`(Units.Volts)
        inputs.rightPivotPositionRads = rightPivotPosition.value.`in`(Units.Radians)
        inputs.rightPivotVelocityRadsPerSec = rightPivotVelocity.value.`in`(Units.RadiansPerSecond)
        inputs.rightPivotSupplyCurrentAmps = rightPivotSupplyCurrent.value.`in`(Units.Amps)
        inputs.rightPivotStatorCurrentAmps = rightPivotStatorCurrent.value.`in`(Units.Amps)
        inputs.rightPivotTempCelsius = rightPivotTemp.value.`in`(Units.Celsius)

        inputs.leftRollerLeaderAppliedVolts = leftRollerLeaderVoltage.value.`in`(Units.Volts)
        inputs.leftRollerLeaderVelocityRadsPerSec = leftRollerLeaderVelocity.value.`in`(Units.RadiansPerSecond)
        inputs.leftRollerLeaderSupplyCurrentAmps = leftRollerLeaderSupplyCurrent.value.`in`(Units.Amps)
        inputs.leftRollerLeaderStatorCurrentAmps = leftRollerLeaderStatorCurrent.value.`in`(Units.Amps)
        inputs.leftRollerLeaderTempCelsius = leftRollerLeaderTemp.value.`in`(Units.Celsius)

        inputs.rightRollerFollowerAppliedVolts = rightRollerFollowerVoltage.value.`in`(Units.Volts)
        inputs.rightRollerFollowerVelocityRadsPerSec = rightRollerFollowerVelocity.value.`in`(Units.RadiansPerSecond)
        inputs.rightRollerFollowerSupplyCurrentAmps = rightRollerFollowerSupplyCurrent.value.`in`(Units.Amps)
        inputs.rightRollerFollowerStatorCurrentAmps = rightRollerFollowerStatorCurrent.value.`in`(Units.Amps)
        inputs.rightRollerFollowerTempCelsius = rightRollerFollowerTemp.value.`in`(Units.Celsius)

        leftPivotLeaderDisconnectedAlert.set(!leftPivotConnected)
        rightPivotFollowerDisconnectedAlert.set(!rightPivotConnected)
        leftRollerLeaderDisconnectedAlert.set(!leftRollerConnected)
        rightRollerFollowerDisconnectedAlert.set(!rightRollerConnected)
    }

    override fun setPivotVoltage(pivotVolts: Double) {
        leftPivot.setControl(pivotVoltageRequest.withOutput(pivotVolts))
        rightPivot.setControl(pivotVoltageRequest.withOutput(pivotVolts))
    }

    override fun setPivotAngle(pivotAngle: Angle) {
        leftPivot.setControl(pivotPositionRequest.withPosition(pivotAngle))
        rightPivot.setControl(pivotPositionRequest.withPosition(pivotAngle))
    }

    override fun resetPivotAngle(pivotAngle: Angle) {
        leftPivot.setPosition(pivotAngle)
        rightPivot.setPosition(pivotAngle)
    }

    override fun setRollerVoltage(rollerVolts: Double) {
        leftRollerLeader.setControl(rollerVoltageRequest.withOutput(rollerVolts))
    }

    companion object {
        val pivotSlot0Config = Slot0Configs().apply {
            kP = 100.0
            kI = 0.0
            kD = 0.1
            kS = 0.2
        }

        val leftPivotConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimit = IntakeConstants.PIVOT_SUPPLY_LIMIT
                    StatorCurrentLimit = IntakeConstants.PIVOT_STATOR_LIMIT
                }

                MotorOutput.apply {
                    NeutralMode = IntakeConstants.LEFT_PIVOT_NEUTRAL_MODE
                    Inverted = IntakeConstants.LEFT_PIVOT_INVERSION
                }

                Feedback.SensorToMechanismRatio = IntakeConstants.PIVOT_GEARING_SENSOR_TO_MECH

                Slot0 = pivotSlot0Config
            }

        val rightPivotConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimit = IntakeConstants.PIVOT_SUPPLY_LIMIT
                    StatorCurrentLimit = IntakeConstants.PIVOT_STATOR_LIMIT
                }

                MotorOutput.apply {
                    NeutralMode = IntakeConstants.RIGHT_PIVOT_NEUTRAL_MODE
                    Inverted = IntakeConstants.RIGHT_PIVOT_INVERSION
                }

                Feedback.SensorToMechanismRatio = IntakeConstants.PIVOT_GEARING_SENSOR_TO_MECH

                Slot0 = pivotSlot0Config
            }

        val rollerConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimit = IntakeConstants.ROLLER_SUPPLY_LIMIT
                    StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_LIMIT
                }

                MotorOutput.apply {
                    NeutralMode = IntakeConstants.LEFT_ROLLER_NEUTRAL_MODE
                    Inverted = IntakeConstants.LEFT_ROLLER_INVERSION
                }

                Feedback.SensorToMechanismRatio = IntakeConstants.ROLLER_GEARING
            }
    }
}
