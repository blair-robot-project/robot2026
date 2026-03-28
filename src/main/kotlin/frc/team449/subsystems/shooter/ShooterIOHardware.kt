package frc.team449.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.GravityTypeValue
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants.ShooterConstants
import frc.team449.util.PhoenixUtil
import frc.team449.util.PhoenixUtil.tryUntilOk

open class ShooterIOHardware : ShooterIO {
    val leftTopLeader = TalonFX(ShooterConstants.LEFT_TOP_LEADER_ID)
    val leftBottomFollower = TalonFX(ShooterConstants.LEFT_BOTTOM_FOLLOWER_ID)
    val rightTopFollower = TalonFX(ShooterConstants.RIGHT_TOP_FOLLOWER_ID)
    val rightBottomFollower = TalonFX(ShooterConstants.RIGHT_BOTTOM_FOLLOWER_ID)
    val hood = TalonFX(ShooterConstants.HOOD_MOTOR_ID)

    private val flywheelVelocityRequest = VelocityVoltage(0.0)
    private val flywheelVoltageRequest = VoltageOut(0.0)
    private val hoodAngleRequest = PositionVoltage(0.0)
    private val hoodVoltageRequest = VoltageOut(0.0)

    private val leftTopLeaderVoltage = leftTopLeader.motorVoltage
    private val leftTopLeaderVelocity = leftTopLeader.velocity
    private val leftTopLeaderSupplyCurrent = leftTopLeader.supplyCurrent
    private val leftTopLeaderStatorCurrent = leftTopLeader.statorCurrent
    private val leftTopLeaderTemp = leftTopLeader.deviceTemp

    private val leftBottomFollowerVoltage = leftBottomFollower.motorVoltage
    private val leftBottomFollowerVelocity = leftBottomFollower.velocity
    private val leftBottomFollowerSupplyCurrent = leftBottomFollower.supplyCurrent
    private val leftBottomFollowerStatorCurrent = leftBottomFollower.statorCurrent
    private val leftBottomFollowerTemp = leftBottomFollower.deviceTemp

    private val rightTopFollowerVoltage = rightTopFollower.motorVoltage
    private val rightTopFollowerVelocity = rightTopFollower.velocity
    private val rightTopFollowerSupplyCurrent = rightTopFollower.supplyCurrent
    private val rightTopFollowerStatorCurrent = rightTopFollower.statorCurrent
    private val rightTopFollowerTemp = rightTopFollower.deviceTemp

    private val rightBottomFollowerVoltage = rightBottomFollower.motorVoltage
    private val rightBottomFollowerVelocity = rightBottomFollower.velocity
    private val rightBottomFollowerSupplyCurrent = rightBottomFollower.supplyCurrent
    private val rightBottomFollowerStatorCurrent = rightBottomFollower.statorCurrent
    private val rightBottomFollowerTemp = rightBottomFollower.deviceTemp

    private val hoodVoltage = hood.motorVoltage
    private val hoodAngle = hood.position
    private val hoodVelocity = hood.velocity
    private val hoodSupplyCurrent = hood.supplyCurrent
    private val hoodStatorCurrent = hood.statorCurrent
    private val hoodTemp = hood.deviceTemp

    private val lowPrioSignals =
        arrayOf(
            leftTopLeaderTemp,
            hoodTemp,
            leftBottomFollowerVoltage,
            leftBottomFollowerVelocity,
            leftBottomFollowerSupplyCurrent,
            leftBottomFollowerStatorCurrent,
            leftBottomFollowerTemp,
            rightTopFollowerVoltage,
            rightTopFollowerVelocity,
            rightTopFollowerSupplyCurrent,
            rightTopFollowerStatorCurrent,
            rightTopFollowerTemp,
            rightBottomFollowerVoltage,
            rightBottomFollowerVelocity,
            rightBottomFollowerSupplyCurrent,
            rightBottomFollowerStatorCurrent,
            rightBottomFollowerTemp,
        )

    private val highPrioSignals =
        arrayOf(
            leftTopLeaderVoltage,
            leftTopLeaderVelocity,
            leftTopLeaderSupplyCurrent,
            leftTopLeaderStatorCurrent,
            hoodVoltage,
            hoodAngle,
            hoodVelocity,
            hoodSupplyCurrent,
            hoodStatorCurrent,
        )

    private val hoodConnected: Boolean
        get() = BaseStatusSignal.isAllGood(
            hoodVoltage,
            hoodAngle,
            hoodVelocity,
            hoodSupplyCurrent,
            hoodStatorCurrent,
        )

    private val leftTopLeaderConnected: Boolean
        get() = BaseStatusSignal.isAllGood(
            leftTopLeaderVoltage,
            leftTopLeaderVelocity,
            leftTopLeaderStatorCurrent,
        )

    private val leftBottomFollowerConnected: Boolean
        get() = BaseStatusSignal.isAllGood(
            leftBottomFollowerVoltage,
            leftBottomFollowerSupplyCurrent,
            leftBottomFollowerStatorCurrent,
        )

    private val rightTopFollowerConnected: Boolean
        get() = BaseStatusSignal.isAllGood(
            rightTopFollowerVoltage,
            rightTopFollowerSupplyCurrent,
            rightTopFollowerStatorCurrent,
        )

    private val rightBottomFollowerConnected: Boolean
        get() = BaseStatusSignal.isAllGood(
            rightBottomFollowerVoltage,
            rightBottomFollowerSupplyCurrent,
            rightBottomFollowerStatorCurrent,
        )

    private val leftTopLeaderDisconnectedAlert =
        Alert("Left Top Flywheel Disconnected (ID ${ShooterConstants.LEFT_TOP_LEADER_ID}).", Alert.AlertType.kError)
    private val leftBottomFollowerDisconnectedAlert =
        Alert("Left Bottom Flywheel Disconnected (ID ${ShooterConstants.LEFT_BOTTOM_FOLLOWER_ID}).", Alert.AlertType.kError)
    private val rightTopFollowerDisconnectedAlert =
        Alert("Right Top Flywheel Disconnected (ID ${ShooterConstants.RIGHT_TOP_FOLLOWER_ID}).", Alert.AlertType.kError)
    private val rightBottomFollowerDisconnectedAlert =
        Alert("Right Bottom Flywheel Disconnected (ID ${ShooterConstants.RIGHT_BOTTOM_FOLLOWER_ID}).", Alert.AlertType.kError)
    private val hoodDisconnectedAlert =
        Alert("Hood Disconnected (ID ${ShooterConstants.HOOD_MOTOR_ID}).", Alert.AlertType.kError)

    init {
        tryUntilOk(5) { leftTopLeader.configurator.apply(flywheelConfig) }
        tryUntilOk(5) { leftBottomFollower.configurator.apply(flywheelConfig) }
        tryUntilOk(5) { rightTopFollower.configurator.apply(flywheelConfig) }
        tryUntilOk(5) { rightBottomFollower.configurator.apply(flywheelConfig) }
        tryUntilOk(5) { hood.configurator.apply(hoodConfig) }

        leftBottomFollower.setControl(Follower(leftTopLeader.deviceID, ShooterConstants.LEFT_FOLLOWER_ALIGNMENT))
        rightTopFollower.setControl(Follower(leftTopLeader.deviceID, ShooterConstants.RIGHT_FOLLOWER_ALIGNMENT))
        rightBottomFollower.setControl(Follower(leftTopLeader.deviceID, ShooterConstants.RIGHT_FOLLOWER_ALIGNMENT))

        ParentDevice.optimizeBusUtilizationForAll(leftTopLeader, leftBottomFollower, rightTopFollower, rightBottomFollower, hood)

        BaseStatusSignal.setUpdateFrequencyForAll(4.0, *lowPrioSignals)
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, *highPrioSignals)

        PhoenixUtil.registerSignals(*lowPrioSignals, *highPrioSignals)

        resetHoodAngle(ShooterConstants.MIN_HOOD_ANGLE)
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        BaseStatusSignal.refreshAll(*lowPrioSignals, *highPrioSignals)

        inputs.leftTopLeaderAppliedVolts = leftTopLeaderVoltage.value.`in`(Volts)
        inputs.leftTopLeaderVelocityRadsPerSec = leftTopLeaderVelocity.value.`in`(RadiansPerSecond)
        inputs.leftTopLeaderSupplyCurrentAmps = leftTopLeaderSupplyCurrent.value.`in`(Amps)
        inputs.leftTopLeaderStatorCurrentAmps = leftTopLeaderStatorCurrent.value.`in`(Amps)
        inputs.leftTopLeaderTempCelsius = leftTopLeaderTemp.value.`in`(Celsius)

        inputs.leftBottomFollowerAppliedVolts = leftBottomFollowerVoltage.value.`in`(Volts)
        inputs.leftBottomFollowerVelocityRadsPerSec = leftBottomFollowerVelocity.value.`in`(RadiansPerSecond)
        inputs.leftBottomFollowerSupplyCurrentAmps = leftBottomFollowerSupplyCurrent.value.`in`(Amps)
        inputs.leftBottomFollowerStatorCurrentAmps = leftBottomFollowerStatorCurrent.value.`in`(Amps)
        inputs.leftBottomFollowerTempCelsius = leftBottomFollowerTemp.value.`in`(Celsius)

        inputs.rightTopFollowerAppliedVolts = rightTopFollowerVoltage.value.`in`(Volts)
        inputs.rightTopFollowerVelocityRadsPerSec = rightTopFollowerVelocity.value.`in`(RadiansPerSecond)
        inputs.rightTopFollowerSupplyCurrentAmps = rightTopFollowerSupplyCurrent.value.`in`(Amps)
        inputs.rightTopFollowerStatorCurrentAmps = rightTopFollowerStatorCurrent.value.`in`(Amps)
        inputs.rightTopFollowerTempCelsius = rightTopFollowerTemp.value.`in`(Celsius)

        inputs.rightBottomFollowerAppliedVolts = rightBottomFollowerVoltage.value.`in`(Volts)
        inputs.rightBottomFollowerVelocityRadsPerSec = rightBottomFollowerVelocity.value.`in`(RadiansPerSecond)
        inputs.rightBottomFollowerSupplyCurrentAmps = rightBottomFollowerSupplyCurrent.value.`in`(Amps)
        inputs.rightBottomFollowerStatorCurrentAmps = rightBottomFollowerStatorCurrent.value.`in`(Amps)
        inputs.rightBottomFollowerTempCelsius = rightBottomFollowerTemp.value.`in`(Celsius)

        inputs.hoodAppliedVolts = hoodVoltage.value.`in`(Volts)
        inputs.hoodAngleRad = hoodAngle.value.`in`(Radians)
        inputs.hoodVelocityRadPerSec = hoodVelocity.value.`in`(RadiansPerSecond)
        inputs.hoodSupplyCurrentAmps = hoodSupplyCurrent.value.`in`(Amps)
        inputs.hoodStatorCurrentAmps = hoodStatorCurrent.value.`in`(Amps)
        inputs.hoodTempCelsius = hoodTemp.value.`in`(Celsius)

        leftTopLeaderDisconnectedAlert.set(!leftTopLeaderConnected)
        leftBottomFollowerDisconnectedAlert.set(!leftBottomFollowerConnected)
        rightTopFollowerDisconnectedAlert.set(!rightTopFollowerConnected)
        rightBottomFollowerDisconnectedAlert.set(!rightBottomFollowerConnected)
        hoodDisconnectedAlert.set(!hoodConnected)
    }

    override fun setFlywheelVoltage(flywheelVolts: Double) {
        leftTopLeader.setControl(flywheelVoltageRequest.withOutput(flywheelVolts))
    }

    override fun setFlywheelVelocity(flywheelVelocity: AngularVelocity) {
        leftTopLeader.setControl(flywheelVelocityRequest.withVelocity(flywheelVelocity))
    }

    override fun setHoodVoltage(hoodVolts: Double) {
        hood.setControl(hoodVoltageRequest.withOutput(hoodVolts))
    }

    override fun setHoodAngle(hoodAngle: Angle) {
        hood.setControl(hoodAngleRequest.withPosition(hoodAngle))
    }

    override fun resetHoodAngle(hoodAngle: Angle) {
        hood.setPosition(hoodAngle)
    }

    companion object {
        val flywheelConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimit = ShooterConstants.FLYWHEEL_SUPPLY_LIM
                    StatorCurrentLimit = ShooterConstants.FLYWHEEL_STATOR_LIM
                }

                MotorOutput.apply {
                    NeutralMode = ShooterConstants.FLYWHEEL_NEUTRAL_MODE
                    Inverted = ShooterConstants.LEFT_LEADER_INVERSION
                }

                Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_GEARING

                Slot0.apply {
                    kP = ShooterConstants.FLYWHEEL_KP
                    kI = ShooterConstants.FLYWHEEL_KI
                    kD = ShooterConstants.FLYWHEEL_KD
                    kS = ShooterConstants.FLYWHEEL_KS
                    kV = ShooterConstants.FLYWHEEL_KV
                }
            }

        val hoodConfig =
            TalonFXConfiguration().apply {
                CurrentLimits.apply {
                    SupplyCurrentLimit = ShooterConstants.HOOD_SUPPLY_LIM
                    StatorCurrentLimit = ShooterConstants.HOOD_STATOR_LIM
                }

                MotorOutput.apply {
                    NeutralMode = ShooterConstants.HOOD_NEUTRAL_MODE
                    Inverted = ShooterConstants.HOOD_INVERSION
                }

                Feedback.SensorToMechanismRatio = ShooterConstants.HOOD_GEARING

                Slot0.apply {
                    kP = ShooterConstants.HOOD_KP
                    kI = ShooterConstants.HOOD_KI
                    kD = ShooterConstants.HOOD_KD
                    kS = ShooterConstants.HOOD_KS
                    kV = ShooterConstants.HOOD_KV
                    kG = ShooterConstants.HOOD_KG
                    GravityType = GravityTypeValue.Arm_Cosine
                }
            }
    }
}
