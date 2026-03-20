package frc.team449.subsystems.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants.IntakeConstants
import frc.team449.util.PhoenixUtil
import frc.team449.util.PhoenixUtil.tryUntilOk

open class IntakeIOHardware : IntakeIO {
    val leftPivotLeader = TalonFX(IntakeConstants.LEFT_PIVOT_MOTOR_ID)
    val rightPivotFollower = TalonFX(IntakeConstants.RIGHT_PIVOT_FOLLOWER_ID)
    val leftRollerLeader = TalonFX(IntakeConstants.LEFT_ROLLER_MOTOR_ID)
    val rightRollerFollower = TalonFX(IntakeConstants.RIGHT_ROLLER_FOLLOWER_ID)

    private val leftPivotLeaderVoltage = leftPivotLeader.motorVoltage
    private val leftPivotLeaderSupplyCurrent = leftPivotLeader.supplyCurrent
    private val leftPivotLeaderStatorCurrent = leftPivotLeader.statorCurrent
    private val leftPivotLeaderPosition = leftPivotLeader.position
    private val leftPivotLeaderVelocity = leftPivotLeader.velocity
    private val leftPivotLeaderTemp = leftPivotLeader.deviceTemp

    private val rightPivotFollowerVoltage = rightPivotFollower.motorVoltage
    private val rightPivotFollowerSupplyCurrent = rightPivotFollower.supplyCurrent
    private val rightPivotFollowerStatorCurrent = rightPivotFollower.statorCurrent
    private val rightPivotFollowerTemp = rightPivotFollower.deviceTemp

    private val leftRollerLeaderVoltage = leftRollerLeader.motorVoltage
    private val leftRollerLeaderSupplyCurrent = leftRollerLeader.supplyCurrent
    private val leftRollerLeaderStatorCurrent = leftRollerLeader.statorCurrent
    private val leftRollerLeaderVelocity = leftRollerLeader.velocity
    private val leftRollerLeaderTemp = leftRollerLeader.deviceTemp

    private val rightRollerFollowerVoltage = rightRollerFollower.motorVoltage
    private val rightRollerFollowerSupplyCurrent = rightRollerFollower.supplyCurrent
    private val rightRollerFollowerStatorCurrent = rightRollerFollower.statorCurrent
    private val rightRollerFollowerTemp = rightRollerFollower.deviceTemp

    private val leftRollerMotorConnected: Boolean =
        BaseStatusSignal.isAllGood(
            leftRollerLeaderVelocity,
            leftRollerLeaderVoltage,
            leftRollerLeaderSupplyCurrent,
            leftRollerLeaderStatorCurrent,
        )

    private val rightRollerMotorConnected: Boolean =
        BaseStatusSignal.isAllGood(
            rightRollerFollowerVoltage,
            rightRollerFollowerSupplyCurrent,
            rightRollerFollowerStatorCurrent,
        )

    private val leftPivotMotorConnected: Boolean =
        BaseStatusSignal.isAllGood(
            leftPivotLeaderPosition,
            leftPivotLeaderVelocity,
            leftPivotLeaderVoltage,
            leftPivotLeaderSupplyCurrent,
            leftPivotLeaderStatorCurrent,
        )

    private val rightPivotMotorConnected: Boolean =
        BaseStatusSignal.isAllGood(
            rightPivotFollowerVoltage,
            rightPivotFollowerSupplyCurrent,
            rightPivotFollowerStatorCurrent,
            rightPivotFollowerTemp,
        )

    val leftPivotLeaderDisconnectedAlert =
        Alert("left pivot leader disconnected (ID ${IntakeConstants.LEFT_PIVOT_MOTOR_ID})", Alert.AlertType.kError)
    val rightPivotFollowerDisconnectedAlert =
        Alert("right pivot follower disconnected (ID ${IntakeConstants.RIGHT_PIVOT_FOLLOWER_ID})", Alert.AlertType.kError)
    val leftRollerLeaderDisconnectedAlert =
        Alert("left roller leader motor disconnected (ID ${IntakeConstants.LEFT_ROLLER_MOTOR_ID})", Alert.AlertType.kError)
    val rightRollerFollowerDisconnectedAlert =
        Alert("right roller follower motor disconnected (ID ${IntakeConstants.RIGHT_ROLLER_FOLLOWER_ID})", Alert.AlertType.kError)

    private val allSignals =
        arrayOf(
            leftPivotLeaderVoltage,
            leftPivotLeaderSupplyCurrent,
            leftPivotLeaderStatorCurrent,
            leftPivotLeaderPosition,
            leftPivotLeaderVelocity,
            leftPivotLeaderTemp,
            rightPivotFollowerVoltage,
            rightPivotFollowerSupplyCurrent,
            rightPivotFollowerStatorCurrent,
            rightPivotFollowerTemp,
            leftRollerLeaderVoltage,
            leftRollerLeaderSupplyCurrent,
            leftRollerLeaderStatorCurrent,
            leftRollerLeaderVelocity,
            leftRollerLeaderTemp,
            rightRollerFollowerVoltage,
            rightRollerFollowerSupplyCurrent,
            rightRollerFollowerStatorCurrent,
            rightRollerFollowerTemp,
        )

    private val pivotVoltageRequest = VoltageOut(0.0)
    private val pivotPositionRequest = PositionVoltage(0.0)
    private val rollerVelocityRequest = VelocityVoltage(0.0)
    private val rollerVoltageRequest = VoltageOut(0.0)

    init {
        ParentDevice.optimizeBusUtilizationForAll(leftPivotLeader, rightPivotFollower, leftRollerLeader, rightRollerFollower)

        tryUntilOk(5) { leftPivotLeader.configurator.apply(leftPivotConfig, 0.25) }
        tryUntilOk(5) { rightPivotFollower.configurator.apply(rightPivotConfig, 0.25) }
        tryUntilOk(5) { leftRollerLeader.configurator.apply(rollerConfig, 0.25) }
        tryUntilOk(5) { rightRollerFollower.configurator.apply(rollerConfig, 0.25) }

        rightRollerFollower.setControl(Follower(leftRollerLeader.deviceID, IntakeConstants.RIGHT_ROLLER_FOLLOWER_ALIGNMENT))

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, *allSignals)

        PhoenixUtil.registerSignals(*allSignals)
    }

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        BaseStatusSignal.refreshAll(*allSignals)

        inputs.leftPivotLeaderAppliedVolts = leftPivotLeaderVoltage.value.`in`(Units.Volts)
        inputs.leftPivotLeaderPositionRad = leftPivotLeaderPosition.value.`in`(Units.Radians)
        inputs.leftPivotLeaderVelocityRadPerSec = leftPivotLeaderVelocity.value.`in`(Units.RadiansPerSecond)
        inputs.leftPivotLeaderSupplyCurrentAmps = leftPivotLeaderSupplyCurrent.value.`in`(Units.Amps)
        inputs.leftPivotLeaderStatorCurrentAmps = leftPivotLeaderStatorCurrent.value.`in`(Units.Amps)
        inputs.leftPivotLeaderTempCelsius = leftPivotLeaderTemp.value.`in`(Units.Celsius)

        inputs.rightPivotFollowerAppliedVolts = rightPivotFollowerVoltage.value.`in`(Units.Volts)
        inputs.rightPivotFollowerSupplyCurrentAmps = rightPivotFollowerSupplyCurrent.value.`in`(Units.Amps)
        inputs.rightPivotFollowerStatorCurrentAmps = rightPivotFollowerStatorCurrent.value.`in`(Units.Amps)
        inputs.rightPivotFollowerTempCelsius = rightPivotFollowerTemp.value.`in`(Units.Celsius)

        inputs.leftRollerLeaderAppliedVolts = leftRollerLeaderVoltage.value.`in`(Units.Volts)
        inputs.leftRollerLeaderVelocityRadPerSec = leftRollerLeaderVelocity.value.`in`(Units.RadiansPerSecond)
        inputs.leftRollerLeaderSupplyCurrentAmps = leftRollerLeaderSupplyCurrent.value.`in`(Units.Amps)
        inputs.leftRollerLeaderStatorCurrentAmps = leftRollerLeaderStatorCurrent.value.`in`(Units.Amps)
        inputs.leftRollerLeaderTempCelsius = leftRollerLeaderTemp.value.`in`(Units.Celsius)

        inputs.rightRollerFollowerAppliedVolts = rightRollerFollowerVoltage.value.`in`(Units.Volts)
        inputs.rightRollerFollowerSupplyCurrentAmps = rightRollerFollowerSupplyCurrent.value.`in`(Units.Amps)
        inputs.rightRollerFollowerStatorCurrentAmps = rightRollerFollowerStatorCurrent.value.`in`(Units.Amps)
        inputs.rightRollerFollowerTempCelsius = rightRollerFollowerTemp.value.`in`(Units.Celsius)

        leftPivotLeaderDisconnectedAlert.set(!leftPivotMotorConnected)
        rightPivotFollowerDisconnectedAlert.set(!rightPivotMotorConnected)
        leftRollerLeaderDisconnectedAlert.set(!leftRollerMotorConnected)
        rightRollerFollowerDisconnectedAlert.set(!rightRollerMotorConnected)
    }

    override fun setPivotVoltage(volts: Double) {
        leftPivotLeader.setControl(pivotVoltageRequest.withOutput(volts))
        rightPivotFollower.setControl(pivotVoltageRequest.withOutput(volts))
    }

    override fun setPivotAngle(angle: Angle) {
        leftPivotLeader.setControl(pivotPositionRequest.withPosition(angle))
        rightPivotFollower.setControl(pivotPositionRequest.withPosition(angle))
    }

    override fun resetPivotAngle(angle: Angle) {
        leftPivotLeader.setPosition(angle)
        rightPivotFollower.setPosition(angle)
    }

    override fun setRollerVelocity(velocity: AngularVelocity) {
        leftRollerLeader.setControl(rollerVelocityRequest.withVelocity(velocity))
    }

    override fun setRollerVoltage(volts: Double) {
        leftRollerLeader.setControl(rollerVoltageRequest.withOutput(volts))
    }

    val pivotCurrentConfig = CurrentLimitsConfigs()
    val rollerCurrentConfig = CurrentLimitsConfigs()

    override fun setSupplyLimits(
        pivotSupplyLimitAmps: Double,
        rollerSupplyLimitAmps: Double,
    ) {
        pivotCurrentConfig.SupplyCurrentLimit = pivotSupplyLimitAmps
        pivotCurrentConfig.StatorCurrentLimit = IntakeConstants.PIVOT_STATOR_LIMIT
        rollerCurrentConfig.SupplyCurrentLimit = rollerSupplyLimitAmps
        rollerCurrentConfig.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_LIMIT

        leftPivotLeader.configurator.apply(pivotCurrentConfig, 0.0)
        rightPivotFollower.configurator.apply(pivotCurrentConfig, 0.0)

        leftRollerLeader.configurator.apply(rollerCurrentConfig, 0.0)
        rightRollerFollower.configurator.apply(rollerCurrentConfig, 0.0)
    }

    companion object {
        val pivotSlot0Config =
            Slot0Configs().apply {
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

                Slot0.apply {
                    kP = 0.0
                    kV = 0.09
                }
            }
    }
}
