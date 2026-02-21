package frc.team449.subsystems.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units.*
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

    val leftPivotLeaderDisconnectedAlert = Alert("left pivot leader disconnected (ID ${IntakeConstants.LEFT_PIVOT_MOTOR_ID})", Alert.AlertType.kError)
    val rightPivotFollowerDisconnectedAlert = Alert("right pivot follower disconnected (ID ${IntakeConstants.RIGHT_PIVOT_FOLLOWER_ID})", Alert.AlertType.kError)
    val leftRollerLeaderDisconnectedAlert = Alert("left roller leader motor disconnected (ID ${IntakeConstants.LEFT_ROLLER_MOTOR_ID})", Alert.AlertType.kError)
    val rightRollerFollowerDisconnectedAlert = Alert("right roller follower motor disconnected (ID ${IntakeConstants.RIGHT_ROLLER_FOLLOWER_ID})", Alert.AlertType.kError)

    private val allSignals = arrayOf(
        leftPivotLeaderVoltage, leftPivotLeaderSupplyCurrent, leftPivotLeaderStatorCurrent, leftPivotLeaderPosition, leftPivotLeaderVelocity, leftPivotLeaderTemp,
        rightPivotFollowerVoltage, rightPivotFollowerSupplyCurrent, rightPivotFollowerStatorCurrent, rightPivotFollowerTemp,
        leftRollerLeaderVoltage, leftRollerLeaderSupplyCurrent, leftRollerLeaderStatorCurrent, leftRollerLeaderVelocity, leftRollerLeaderTemp,
        rightRollerFollowerVoltage, rightRollerFollowerSupplyCurrent, rightRollerFollowerStatorCurrent, rightRollerFollowerTemp
    )

    private val pivotVoltageRequest = VoltageOut(0.0)
    private val rollerVelocityRequest = VelocityVoltage(0.0)

    init {
        ParentDevice.optimizeBusUtilizationForAll(leftPivotLeader, rightPivotFollower, leftRollerLeader, rightRollerFollower)

        tryUntilOk(5) { leftPivotLeader.configurator.apply(pivotConfig, 0.25) }
        tryUntilOk(5) { rightPivotFollower.configurator.apply(pivotConfig, 0.25) }
        tryUntilOk(5) { leftRollerLeader.configurator.apply(rollerConfig, 0.25) }
        tryUntilOk(5) { rightRollerFollower.configurator.apply(rollerConfig, 0.25) }

        rightRollerFollower.setControl(Follower(leftRollerLeader.deviceID, IntakeConstants.RIGHT_ROLLER_FOLLOWER_ALIGNMENT))
        rightPivotFollower.setControl(Follower(leftPivotLeader.deviceID, IntakeConstants.RIGHT_PIVOT_FOLLOWER_ALIGNMENT))

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, *allSignals)

        PhoenixUtil.registerSignals(*allSignals)
    }

    private var isAliveCounter = 0

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        BaseStatusSignal.refreshAll(*allSignals)

        inputs.leftPivotLeaderAppliedVolts = leftPivotLeaderVoltage.value.`in`(Volts)
        inputs.leftPivotLeaderPositionRad = leftPivotLeaderPosition.value.`in`(Radians)
        inputs.leftPivotLeaderVelocityRadPerSec = leftPivotLeaderVelocity.value.`in`(RadiansPerSecond)
        inputs.leftPivotLeaderSupplyCurrentAmps = leftPivotLeaderSupplyCurrent.value.`in`(Amps)
        inputs.leftPivotLeaderStatorCurrentAmps = leftPivotLeaderStatorCurrent.value.`in`(Amps)
        inputs.leftPivotLeaderTempCelsius = leftPivotLeaderTemp.value.`in`(Celsius)

        inputs.rightPivotFollowerAppliedVolts = rightPivotFollowerVoltage.value.`in`(Volts)
        inputs.rightPivotFollowerSupplyCurrentAmps = rightPivotFollowerSupplyCurrent.value.`in`(Amps)
        inputs.rightPivotFollowerStatorCurrentAmps = rightPivotFollowerStatorCurrent.value.`in`(Amps)
        inputs.rightPivotFollowerTempCelsius = rightPivotFollowerTemp.value.`in`(Celsius)

        inputs.leftRollerLeaderAppliedVolts = leftRollerLeaderVoltage.value.`in`(Volts)
        inputs.leftRollerLeaderVelocityRadPerSec = leftRollerLeaderVelocity.value.`in`(RadiansPerSecond)
        inputs.leftRollerLeaderSupplyCurrentAmps = leftRollerLeaderSupplyCurrent.value.`in`(Amps)
        inputs.leftRollerLeaderStatorCurrentAmps = leftRollerLeaderStatorCurrent.value.`in`(Amps)
        inputs.leftRollerLeaderTempCelsius = leftRollerLeaderTemp.value.`in`(Celsius)

        inputs.rightRollerFollowerAppliedVolts = rightRollerFollowerVoltage.value.`in`(Volts)
        inputs.rightRollerFollowerSupplyCurrentAmps = rightRollerFollowerSupplyCurrent.value.`in`(Amps)
        inputs.rightRollerFollowerStatorCurrentAmps = rightRollerFollowerStatorCurrent.value.`in`(Amps)
        inputs.rightRollerFollowerTempCelsius = rightRollerFollowerTemp.value.`in`(Celsius)

        if (isAliveCounter++ >= 50) {
            isAliveCounter = 0
            leftPivotLeaderDisconnectedAlert.set(!leftPivotLeader.isAlive)
            rightPivotFollowerDisconnectedAlert.set(!rightPivotFollower.isAlive)
            leftRollerLeaderDisconnectedAlert.set(!leftRollerLeader.isAlive)
            rightRollerFollowerDisconnectedAlert.set(!rightRollerFollower.isAlive)
        }
    }

    override fun setPivotVoltage(volts: Double) {
        leftPivotLeader.setControl(pivotVoltageRequest.withOutput(volts))
    }

    override fun setRollerVelocity(velocity: AngularVelocity) {
        leftRollerLeader.setControl(rollerVelocityRequest.withVelocity(velocity))
    }

    companion object {
        val pivotConfig = TalonFXConfiguration().apply {
            CurrentLimits.apply {
                SupplyCurrentLimit = IntakeConstants.PIVOT_SUPPLY_LIMIT
                StatorCurrentLimit = IntakeConstants.PIVOT_STATOR_LIMIT
            }

            MotorOutput.apply {
                NeutralMode = IntakeConstants.LEFT_PIVOT_NEUTRAL_MODE
                Inverted = IntakeConstants.LEFT_PIVOT_INVERSION
            }

            Feedback.SensorToMechanismRatio = IntakeConstants.PIVOT_GEARING_SENSOR_TO_MECH

            Slot0.apply {
                kP = 5.0
                kG = 0.1
            }
        }

        val rollerConfig = TalonFXConfiguration().apply {
            CurrentLimits.apply {
                SupplyCurrentLimit = IntakeConstants.ROLLER_SUPPLY_LIMIT
                StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_LIMIT
            }

            MotorOutput.apply {
                NeutralMode = IntakeConstants.LEFT_ROLLER_NEUTRAL_MODE
                Inverted = IntakeConstants.LEFT_ROLLER_INVERSION
            }

            Slot0.apply {
                kP = 6.0
                kV = 0.12
            }
        }
    }
}
