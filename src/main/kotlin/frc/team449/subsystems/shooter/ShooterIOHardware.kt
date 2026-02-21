package frc.team449.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants.ShooterConstants.FLYWHEEL_GEARING
import frc.team449.Constants.ShooterConstants.FLYWHEEL_KD
import frc.team449.Constants.ShooterConstants.FLYWHEEL_KI
import frc.team449.Constants.ShooterConstants.FLYWHEEL_KP
import frc.team449.Constants.ShooterConstants.FLYWHEEL_KS
import frc.team449.Constants.ShooterConstants.FLYWHEEL_KV
import frc.team449.Constants.ShooterConstants.FLYWHEEL_STATOR_LIM
import frc.team449.Constants.ShooterConstants.FLYWHEEL_SUPPLY_LIM
import frc.team449.Constants.ShooterConstants.HOOD_GEARING
import frc.team449.Constants.ShooterConstants.HOOD_KD
import frc.team449.Constants.ShooterConstants.HOOD_KG
import frc.team449.Constants.ShooterConstants.HOOD_KI
import frc.team449.Constants.ShooterConstants.HOOD_KP
import frc.team449.Constants.ShooterConstants.HOOD_KS
import frc.team449.Constants.ShooterConstants.HOOD_KV
import frc.team449.Constants.ShooterConstants.HOOD_MOTOR_ID
import frc.team449.Constants.ShooterConstants.HOOD_STATOR_LIM
import frc.team449.Constants.ShooterConstants.HOOD_SUPPLY_LIM
import frc.team449.Constants.ShooterConstants.LEFT_FLYWHEEL_FOLLOWER_ID
import frc.team449.Constants.ShooterConstants.LEFT_FLYWHEEL_LEADER_ID
import frc.team449.Constants.ShooterConstants.RIGHT_FLYWHEEL_FOLLOWER_ID
import frc.team449.Constants.ShooterConstants.RIGHT_FLYWHEEL_LEADER_ID
import frc.team449.util.PhoenixUtil

open class ShooterIOHardware : ShooterIO {
    // TODO: a little constants
    // TODO: interpolating double map for flywheel velocity + hood angle

    val leftLeaderMotor = TalonFX(LEFT_FLYWHEEL_LEADER_ID)
    val leftFollowerMotor = TalonFX(LEFT_FLYWHEEL_FOLLOWER_ID)
    val rightLeaderMotor = TalonFX(RIGHT_FLYWHEEL_LEADER_ID)
    val rightFollowerMotor = TalonFX(RIGHT_FLYWHEEL_FOLLOWER_ID)
    val hoodMotor = TalonFX(HOOD_MOTOR_ID)

    private val flywheelVelocityRequest = VelocityVoltage(0.0)
        .withEnableFOC(false)
        .withSlot(0)
    private val flywheelVoltageRequest = VoltageOut(0.0)
    private val hoodPositionRequest = PositionVoltage(0.0)
    private val hoodVoltageRequest = VoltageOut(0.0)

    private val leftLeaderVelocity = leftLeaderMotor.velocity
    private val leftLeaderMotorVoltage = leftLeaderMotor.motorVoltage
    private val leftLeaderSupplyCurrent = leftLeaderMotor.supplyCurrent
    private val leftLeaderStatorCurrent = leftLeaderMotor.statorCurrent
    private val leftLeaderTemperature = leftLeaderMotor.deviceTemp

    private val leftFollowerMotorVoltage = leftFollowerMotor.motorVoltage
    private val leftFollowerSupplyCurrent = leftFollowerMotor.supplyCurrent
    private val leftFollowerStatorCurrent = leftFollowerMotor.statorCurrent
    private val leftFollowerTemperature = leftFollowerMotor.deviceTemp

    private val rightLeaderVelocity = rightLeaderMotor.velocity
    private val rightLeaderMotorVoltage = rightLeaderMotor.motorVoltage
    private val rightLeaderSupplyCurrent = rightLeaderMotor.supplyCurrent
    private val rightLeaderStatorCurrent = rightLeaderMotor.statorCurrent
    private val rightLeaderTemperature = rightLeaderMotor.deviceTemp

    private val rightFollowerMotorVoltage = rightFollowerMotor.motorVoltage
    private val rightFollowerSupplyCurrent = rightFollowerMotor.supplyCurrent
    private val rightFollowerStatorCurrent = rightFollowerMotor.statorCurrent
    private val rightFollowerTemperature = rightFollowerMotor.deviceTemp

    private val hoodPosition = hoodMotor.position
    private val hoodVelocity = hoodMotor.velocity
    private val hoodTargetPosition = hoodMotor.closedLoopReference
    private val hoodMotorVoltage = hoodMotor.motorVoltage
    private val hoodSupplyCurrent = hoodMotor.supplyCurrent
    private val hoodStatorCurrent = hoodMotor.statorCurrent
    private val hoodTemperature = hoodMotor.deviceTemp

    private val lowPrioSignals = arrayOf(
        leftLeaderSupplyCurrent,
        leftLeaderTemperature,
        leftFollowerMotorVoltage,
        leftFollowerSupplyCurrent,
        leftFollowerStatorCurrent,
        leftFollowerTemperature,

        rightLeaderSupplyCurrent,
        rightLeaderTemperature,
        rightFollowerMotorVoltage,
        rightFollowerSupplyCurrent,
        rightFollowerStatorCurrent,
        rightFollowerTemperature,

        hoodVelocity,
        hoodTargetPosition,
        hoodMotorVoltage,
        hoodSupplyCurrent,
        hoodTemperature
    )

    private val highPrioSignals = arrayOf(
        leftLeaderVelocity,
        leftLeaderMotorVoltage,
        leftLeaderStatorCurrent,

        rightLeaderVelocity,
        rightLeaderMotorVoltage,
        rightLeaderStatorCurrent,

        hoodPosition,
        hoodStatorCurrent,
    )

    private val leftLeaderDisconnectedAlert = Alert("left leader shooter flywheel motor disconnected (ID $LEFT_FLYWHEEL_LEADER_ID)", Alert.AlertType.kError)
    private val rightLeaderDisconnectedAlert = Alert("right leader shooter flywheel motor disconnected (ID $RIGHT_FLYWHEEL_LEADER_ID)", Alert.AlertType.kError)
    private val leftFollowerDisconnectedAlert = Alert("left leader shooter flywheel motor disconnected (ID $LEFT_FLYWHEEL_LEADER_ID)", Alert.AlertType.kError)
    private val rightFollowerDisconnectedAlert = Alert("right leader shooter flywheel motor disconnected (ID $RIGHT_FLYWHEEL_LEADER_ID)", Alert.AlertType.kError)
    private val hoodDisconnectedAlert = Alert("hood motor disconnected (ID $HOOD_MOTOR_ID)", Alert.AlertType.kError)

    init {
        leftLeaderMotor.configurator.apply(leftFlywheelConfig)
        leftFollowerMotor.configurator.apply(leftFlywheelConfig)
        rightLeaderMotor.configurator.apply(rightFlywheelConfig)
        rightFollowerMotor.configurator.apply(rightFlywheelConfig)
        hoodMotor.configurator.apply(hoodConfig)

        leftFollowerMotor.setControl(Follower(leftLeaderMotor.deviceID, MotorAlignmentValue.Aligned))
        rightFollowerMotor.setControl(Follower(rightLeaderMotor.deviceID, MotorAlignmentValue.Aligned))

        ParentDevice.optimizeBusUtilizationForAll(leftLeaderMotor, leftFollowerMotor, rightLeaderMotor, rightFollowerMotor, hoodMotor)

        BaseStatusSignal.setUpdateFrequencyForAll(4.0, *lowPrioSignals)
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, *highPrioSignals)

        PhoenixUtil.registerSignals(*lowPrioSignals, *highPrioSignals)
    }

    private var isAliveCounter = 0

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        BaseStatusSignal.refreshAll(*lowPrioSignals, *highPrioSignals)

        inputs.leftLeaderVelocityRadPerSec = leftLeaderVelocity.value.`in`(RadiansPerSecond)
        inputs.leftLeaderAppliedVolts = leftLeaderMotorVoltage.value.`in`(Volts)
        inputs.leftLeaderSupplyCurrentAmps = leftLeaderSupplyCurrent.value.`in`(Amps)
        inputs.leftLeaderStatorCurrentAmps = leftLeaderStatorCurrent.value.`in`(Amps)
        inputs.leftLeaderTempCelsius = leftLeaderTemperature.value.`in`(Celsius)

        inputs.leftFollowerAppliedVolts = leftFollowerMotorVoltage.value.`in`(Volts)
        inputs.leftFollowerSupplyCurrentAmps = leftFollowerSupplyCurrent.value.`in`(Amps)
        inputs.leftFollowerStatorCurrentAmps = leftFollowerStatorCurrent.value.`in`(Amps)
        inputs.leftFollowerTempCelsius = leftFollowerTemperature.value.`in`(Celsius)

        inputs.rightLeaderVelocityRadPerSec = rightLeaderVelocity.value.`in`(RadiansPerSecond)
        inputs.rightLeaderAppliedVolts = rightLeaderMotorVoltage.value.`in`(Volts)
        inputs.rightLeaderSupplyCurrentAmps = rightLeaderSupplyCurrent.value.`in`(Amps)
        inputs.rightLeaderStatorCurrentAmps = rightLeaderStatorCurrent.value.`in`(Amps)
        inputs.rightLeaderTempCelsius = rightLeaderTemperature.value.`in`(Celsius)

        inputs.rightFollowerAppliedVolts = rightFollowerMotorVoltage.value.`in`(Volts)
        inputs.rightFollowerSupplyCurrentAmps = rightFollowerSupplyCurrent.value.`in`(Amps)
        inputs.rightFollowerStatorCurrentAmps = rightFollowerStatorCurrent.value.`in`(Amps)
        inputs.rightFollowerTempCelsius = rightFollowerTemperature.value.`in`(Celsius)

        inputs.hoodPositionRad = hoodPosition.value.`in`(Radians)
        inputs.hoodVelocityRadPerSec = hoodVelocity.value.`in`(RadiansPerSecond)
        inputs.hoodTargetPositionRad = Units.rotationsToRadians(hoodTargetPosition.value)
        inputs.hoodAppliedVolts = hoodMotorVoltage.value.`in`(Volts)
        inputs.hoodSupplyCurrentAmps = hoodSupplyCurrent.value.`in`(Amps)
        inputs.hoodStatorCurrentAmps = hoodStatorCurrent.value.`in`(Amps)
        inputs.hoodTempCelsius = hoodTemperature.value.`in`(Celsius)

        if (isAliveCounter++ >= 50) {
            isAliveCounter = 0
            hoodDisconnectedAlert.set(!hoodMotor.isAlive)
            leftLeaderDisconnectedAlert.set(!leftLeaderMotor.isAlive)
            rightLeaderDisconnectedAlert.set(!rightLeaderMotor.isAlive)
            leftFollowerDisconnectedAlert.set(!leftFollowerMotor.isAlive)
            rightFollowerDisconnectedAlert.set(!rightFollowerMotor.isAlive)
        }
    }

    override fun setFlywheelVelocity(velocity: AngularVelocity) {
        leftLeaderMotor.setControl(flywheelVelocityRequest.withVelocity(velocity))
        rightLeaderMotor.setControl(flywheelVelocityRequest.withVelocity(velocity))
    }

    // probably used for characterization
    override fun setFlywheelVoltage(volts: Double) {
        leftLeaderMotor.setVoltage(volts)
        rightLeaderMotor.setVoltage(volts)
    }

    override fun setHoodAngle(angle: Angle) {
        hoodMotor.setControl(PositionVoltage(angle))
    }

    override fun setHoodVoltage(voltage: Double) {
        hoodMotor.setVoltage(voltage)
    }

    override fun resetHoodPosition(angle: Angle) {
        hoodMotor.setPosition(angle)
    }

    companion object {
        val leftFlywheelConfig = TalonFXConfiguration().apply {
            CurrentLimits.apply {
                SupplyCurrentLimit = FLYWHEEL_SUPPLY_LIM
                StatorCurrentLimit = FLYWHEEL_STATOR_LIM
            }

            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.CounterClockwise_Positive
            }

            Feedback.SensorToMechanismRatio = FLYWHEEL_GEARING

            Slot0.apply {
                kP = FLYWHEEL_KP
                kI = FLYWHEEL_KI
                kD = FLYWHEEL_KD
                kS = FLYWHEEL_KS
                kV = FLYWHEEL_KV
            }
        }
        val rightFlywheelConfig: TalonFXConfiguration? = leftFlywheelConfig.withMotorOutput(
            MotorOutputConfigs().withNeutralMode(
                NeutralModeValue.Coast
            ).withInverted(InvertedValue.Clockwise_Positive)
        )

        val hoodConfig = TalonFXConfiguration().apply {
            CurrentLimits.apply {
                SupplyCurrentLimit = HOOD_SUPPLY_LIM
                StatorCurrentLimit = HOOD_STATOR_LIM
            }

            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }

            Feedback.SensorToMechanismRatio = HOOD_GEARING

            Slot0.apply {
                kP = HOOD_KP
                kI = HOOD_KI
                kD = HOOD_KD
                kS = HOOD_KS
                kG = HOOD_KG
                kV = HOOD_KV
            }
        }
    }
}
