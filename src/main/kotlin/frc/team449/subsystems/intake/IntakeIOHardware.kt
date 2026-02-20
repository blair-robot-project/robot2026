package frc.team449.subsystems.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants.IntakeConstants
import frc.team449.util.PhoenixUtil
import frc.team449.util.PhoenixUtil.tryUntilOk

open class IntakeIOHardware : IntakeIO {
    val pivotLeader = TalonFX(IntakeConstants.PIVOT_MOTOR_ID)
    val pivotFollower = TalonFX(IntakeConstants.PIVOT_FOLLOWER_ID)
    val rollerLeader = TalonFX(IntakeConstants.ROLLER_MOTOR_ID)
    val rollerFollower = TalonFX(IntakeConstants.ROLLER_FOLLOWER_ID)

    private val pivotVoltage = pivotLeader.motorVoltage
    private val pivotSupplyCurrent = pivotLeader.supplyCurrent
    private val pivotStatorCurrent = pivotLeader.statorCurrent
    private val pivotPosition = pivotLeader.position
    private val pivotVelocity = pivotLeader.velocity
    private val pivotTemp = pivotLeader.deviceTemp

    private val pivotFollowerVoltage = pivotFollower.motorVoltage
    private val pivotFollowerSupplyCurrent = pivotFollower.supplyCurrent
    private val pivotFollowerStatorCurrent = pivotFollower.statorCurrent
    private val pivotFollowerTemp = pivotFollower.deviceTemp

    private val rollerVoltage = rollerLeader.motorVoltage
    private val rollerSupplyCurrent = rollerLeader.supplyCurrent
    private val rollerStatorCurrent = rollerLeader.statorCurrent
    private val rollerVelocity = rollerLeader.velocity
    private val rollerTemp = rollerLeader.deviceTemp

    private val rollerFollowerVoltage = rollerFollower.motorVoltage
    private val rollerFollowerSupplyCurrent = rollerFollower.supplyCurrent
    private val rollerFollowerStatorCurrent = rollerFollower.statorCurrent
    private val rollerFollowerTemp = rollerFollower.deviceTemp

    val pivotLeaderDisconnectedAlert = Alert("Pivot Leader Disconnected (ID ${IntakeConstants.PIVOT_MOTOR_ID}).", Alert.AlertType.kError)
    val pivotFollowerDisconnectedAlert = Alert("Pivot Follower Disconnected (ID ${IntakeConstants.PIVOT_FOLLOWER_ID}).", Alert.AlertType.kError)
    val rollerLeaderDisconnectedAlert = Alert("Right Roller Motor disconnected (ID ${IntakeConstants.ROLLER_MOTOR_ID}).", Alert.AlertType.kError)
    val rollerFollowerDisconnectedAlert = Alert("Left Roller Motor disconnected (ID ${IntakeConstants.ROLLER_FOLLOWER_ID}).", Alert.AlertType.kError)

    private val allSignals = arrayOf(
        pivotVoltage, pivotSupplyCurrent, pivotStatorCurrent, pivotPosition, pivotVelocity, pivotTemp,
        pivotFollowerVoltage, pivotFollowerSupplyCurrent, pivotFollowerStatorCurrent, pivotFollowerTemp,
        rollerVoltage, rollerSupplyCurrent, rollerStatorCurrent, rollerVelocity, rollerTemp,
        rollerFollowerVoltage, rollerFollowerSupplyCurrent, rollerFollowerStatorCurrent, rollerFollowerTemp
    )

    private val pivotVoltageRequest = VoltageOut(0.0)
    private val rollerVelocityRequest = VelocityVoltage(0.0)

    init {
        ParentDevice.optimizeBusUtilizationForAll(pivotLeader, pivotFollower, rollerLeader, rollerFollower)

        tryUntilOk(5) { pivotLeader.configurator.apply(pivotConfig, 0.25) }
        tryUntilOk(5) { pivotFollower.configurator.apply(pivotConfig, 0.25) }
        tryUntilOk(5) { rollerLeader.configurator.apply(rollerConfig, 0.25) }
        tryUntilOk(5) { rollerFollower.configurator.apply(rollerConfig, 0.25) }

        rollerFollower.setControl(Follower(rollerLeader.deviceID, IntakeConstants.ROLLER_FOLLOWER_ALIGNMENT))
        pivotFollower.setControl(Follower(pivotLeader.deviceID, IntakeConstants.PIVOT_FOLLOWER_ALIGNMENT))

        BaseStatusSignal.setUpdateFrequencyForAll(50.0, *allSignals)

        PhoenixUtil.registerSignals(*allSignals)
    }

    private var isAliveCounter = 0

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        BaseStatusSignal.refreshAll(*allSignals)

        inputs.pivotAppliedVolts = pivotVoltage.value.`in`(Volts)
        inputs.pivotPositionRad = pivotPosition.value.`in`(Radians)
        inputs.pivotVelocityRadPerSec = pivotVelocity.value.`in`(RadiansPerSecond)
        inputs.pivotSupplyCurrentAmps = pivotSupplyCurrent.value.`in`(Amps)
        inputs.pivotStatorCurrentAmps = pivotStatorCurrent.value.`in`(Amps)
        inputs.pivotTempCelsius = pivotTemp.value.`in`(Celsius)

        inputs.pivotFollowerAppliedVolts = pivotFollowerVoltage.value.`in`(Volts)
        inputs.pivotFollowerSupplyCurrentAmps = pivotFollowerSupplyCurrent.value.`in`(Amps)
        inputs.pivotFollowerStatorCurrentAmps = pivotFollowerStatorCurrent.value.`in`(Amps)
        inputs.pivotFollowerTempCelsius = pivotFollowerTemp.value.`in`(Celsius)

        inputs.rollerAppliedVolts = rollerVoltage.value.`in`(Volts)
        inputs.rollerVelocityRadPerSec = rollerVelocity.value.`in`(RadiansPerSecond)
        inputs.rollerSupplyCurrentAmps = rollerSupplyCurrent.value.`in`(Amps)
        inputs.rollerStatorCurrentAmps = rollerStatorCurrent.value.`in`(Amps)
        inputs.rollerTempCelsius = rollerTemp.value.`in`(Celsius)

        inputs.rollerFollowerAppliedVolts = rollerFollowerVoltage.value.`in`(Volts)
        inputs.rollerFollowerSupplyCurrentAmps = rollerFollowerSupplyCurrent.value.`in`(Amps)
        inputs.rollerFollowerStatorCurrentAmps = rollerFollowerStatorCurrent.value.`in`(Amps)
        inputs.rollerFollowerTempCelsius = rollerFollowerTemp.value.`in`(Celsius)

        if (isAliveCounter++ >= 50) {
            isAliveCounter = 0
            pivotLeaderDisconnectedAlert.set(!pivotLeader.isAlive)
            pivotFollowerDisconnectedAlert.set(!pivotFollower.isAlive)
            rollerLeaderDisconnectedAlert.set(!rollerLeader.isAlive)
            rollerFollowerDisconnectedAlert.set(!rollerFollower.isAlive)
        }
    }

    override fun setPivotVoltage(volts: Double) {
        pivotLeader.setControl(pivotVoltageRequest.withOutput(volts))
    }

    override fun setRollerVelocity(velocity: AngularVelocity) {
        rollerLeader.setControl(rollerVelocityRequest.withVelocity(velocity))
    }

    companion object {
        val pivotConfig = TalonFXConfiguration().apply {
            CurrentLimits.apply {
                SupplyCurrentLimit = IntakeConstants.PIVOT_SUPPLY_LIMIT
                StatorCurrentLimit = IntakeConstants.PIVOT_STATOR_LIMIT
            }

            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.CounterClockwise_Positive // TODO: verify direction
            }

            Feedback.SensorToMechanismRatio = IntakeConstants.PIVOT_SENSOR_TO_MECH

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
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.CounterClockwise_Positive // TODO: verify direction
            }

            Slot0.apply {
                kP = 6.0
                kV = 0.12
            }
        }
    }
}
