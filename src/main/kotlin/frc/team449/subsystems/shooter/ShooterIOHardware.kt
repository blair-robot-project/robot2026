package frc.team449.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import frc.team449.Constants.ShooterConstants.FLYWHEEL_GEARING
import frc.team449.Constants.ShooterConstants.FLYWHEEL_STATOR_LIM
import frc.team449.Constants.ShooterConstants.FLYWHEEL_SUPPLY_LIM
import frc.team449.Constants.ShooterConstants.HOOD_ACCELERATION
import frc.team449.Constants.ShooterConstants.HOOD_CRUISE_VELOCITY
import frc.team449.Constants.ShooterConstants.HOOD_GEARING
import frc.team449.Constants.ShooterConstants.HOOD_MIN_ANGLE
import frc.team449.Constants.ShooterConstants.HOOD_MOTOR_ID
import frc.team449.Constants.ShooterConstants.HOOD_STATOR_LIM
import frc.team449.Constants.ShooterConstants.HOOD_SUPPLY_LIM
import frc.team449.Constants.ShooterConstants.HOOD_TOLERANCE
import frc.team449.Constants.ShooterConstants.HOOD_VOLTAGE_CONTROL
import frc.team449.Constants.ShooterConstants.LEFT_FLYWHEEL_FOLLOWER_ID
import frc.team449.Constants.ShooterConstants.LEFT_FLYWHEEL_LEADER_ID
import frc.team449.Constants.ShooterConstants.RIGHT_FLYWHEEL_FOLLOWER_ID
import frc.team449.Constants.ShooterConstants.RIGHT_FLYWHEEL_LEADER_ID
import frc.team449.Constants.ShooterConstants.SHOOTER_VOLTAGE
import kotlin.math.abs

class ShooterIOHardware : ShooterIO {
    // TODO: p much all the constants
    // TODO: current homing
    // TODO: interpolating double map for voltage + hood angle
    // TODO: velocity control for flywheels

    private val leftLeaderMotor = TalonFX(LEFT_FLYWHEEL_LEADER_ID)
    private val leftFollowerMotor = TalonFX(LEFT_FLYWHEEL_FOLLOWER_ID)

    private val rightLeaderMotor = TalonFX(RIGHT_FLYWHEEL_LEADER_ID)
    private val rightFollowerMotor = TalonFX(RIGHT_FLYWHEEL_FOLLOWER_ID)

    private val hoodMotor = TalonFX(HOOD_MOTOR_ID)

    private val flywheelConfig: TalonFXConfiguration
    private val hoodConfig: TalonFXConfiguration

    private var leftLeaderMotorConnected: Boolean = true
    private var rightLeaderMotorConnected: Boolean = true
    private var leftFollowerMotorConnected: Boolean = true
    private var rightFollowerMotorConnected: Boolean = true
    private var hoodMotorConnected: Boolean = true

    private val leftLeaderMotorVoltage: StatusSignal<Voltage>
    private val leftLeaderSupplyCurrent: StatusSignal<Current>
    private val leftLeaderStatorCurrent: StatusSignal<Current>
    private val leftLeaderTemperature: StatusSignal<Temperature>

    private val rightLeaderMotorVoltage: StatusSignal<Voltage>
    private val rightLeaderSupplyCurrent: StatusSignal<Current>
    private val rightLeaderStatorCurrent: StatusSignal<Current>
    private val rightLeaderTemperature: StatusSignal<Temperature>

    private val leftFollowerMotorVoltage: StatusSignal<Voltage>
    private val leftFollowerSupplyCurrent: StatusSignal<Current>
    private val leftFollowerStatorCurrent: StatusSignal<Current>
    private val leftFollowerTemperature: StatusSignal<Temperature>

    private val rightFollowerMotorVoltage: StatusSignal<Voltage>
    private val rightFollowerSupplyCurrent: StatusSignal<Current>
    private val rightFollowerStatorCurrent: StatusSignal<Current>
    private val rightFollowerTemperature: StatusSignal<Temperature>

    private val hoodMotorVoltage: StatusSignal<Voltage>
    private val hoodSupplyCurrent: StatusSignal<Current>
    private val hoodStatorCurrent: StatusSignal<Current>
    private val hoodTemperature: StatusSignal<Temperature>
    private val hoodCurrentPos: StatusSignal<Angle>
    private val hoodTargetPos: StatusSignal<Double>

    private val leftLeaderMotorDisconnectedAlert =
        Alert(
            "left leader shooter flywheel motor disconnected (ID $LEFT_FLYWHEEL_LEADER_ID)",
            Alert.AlertType.kError,
        )
    private val rightLeaderMotorDisconnectedAlert =
        Alert(
            "right leader shooter flywheel motor disconnected (ID $RIGHT_FLYWHEEL_LEADER_ID)",
            Alert.AlertType.kError,
        )
    private val leftFollowerMotorDisconnectedAlert =
        Alert(
            "left leader shooter flywheel motor disconnected (ID $LEFT_FLYWHEEL_LEADER_ID)",
            Alert.AlertType.kError,
        )
    private val rightFollowerMotorDisconnectedAlert =
        Alert(
            "right leader shooter flywheel motor disconnected (ID $RIGHT_FLYWHEEL_LEADER_ID)",
            Alert.AlertType.kError,
        )
    private val hoodMotorDisconnectedAlert =
        Alert(
            "hood motor disconnected (ID $HOOD_MOTOR_ID",
            Alert.AlertType.kError
        )

    init {
        val flywheelCurrentLimitConfigs: CurrentLimitsConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(FLYWHEEL_SUPPLY_LIM)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(FLYWHEEL_STATOR_LIM)

        val hoodCurrentLimitConfigs: CurrentLimitsConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(HOOD_SUPPLY_LIM)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(HOOD_STATOR_LIM)

        val flywheelMotorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        val hoodMotorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        val flywheelFeedback =
            FeedbackConfigs()
                .withSensorToMechanismRatio(FLYWHEEL_GEARING)

        val hoodFeedback =
            FeedbackConfigs()
                .withSensorToMechanismRatio(HOOD_GEARING)

        val hoodMotionMagicConfigs =
            MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(HOOD_CRUISE_VELOCITY)
                .withMotionMagicAcceleration(HOOD_ACCELERATION)

        flywheelConfig =
            TalonFXConfiguration()
                .withCurrentLimits(flywheelCurrentLimitConfigs)
                .withMotorOutput(flywheelMotorOutput)
                .withFeedback(flywheelFeedback)

        leftLeaderMotor.configurator.apply(flywheelConfig)
        leftFollowerMotor.configurator.apply(flywheelConfig)
        rightLeaderMotor.configurator.apply(flywheelConfig)
        rightFollowerMotor.configurator.apply(flywheelConfig)

        leftFollowerMotor.setControl(Follower(leftLeaderMotor.deviceID, MotorAlignmentValue.Aligned))
        rightFollowerMotor.setControl(Follower(rightLeaderMotor.deviceID, MotorAlignmentValue.Aligned))

        hoodConfig =
            TalonFXConfiguration()
                .withCurrentLimits(hoodCurrentLimitConfigs)
                .withMotorOutput(hoodMotorOutput)
                .withFeedback(hoodFeedback)
                .withMotionMagic(hoodMotionMagicConfigs)

        hoodMotor.configurator.apply(hoodConfig)

        leftLeaderTemperature = leftLeaderMotor.deviceTemp
        leftLeaderMotorVoltage = leftLeaderMotor.motorVoltage
        leftLeaderSupplyCurrent = leftLeaderMotor.supplyCurrent
        leftLeaderStatorCurrent = leftLeaderMotor.statorCurrent

        rightLeaderTemperature = rightLeaderMotor.deviceTemp
        rightLeaderMotorVoltage = rightLeaderMotor.motorVoltage
        rightLeaderSupplyCurrent = rightLeaderMotor.supplyCurrent
        rightLeaderStatorCurrent = rightLeaderMotor.statorCurrent

        leftFollowerTemperature = leftFollowerMotor.deviceTemp
        leftFollowerMotorVoltage = leftFollowerMotor.motorVoltage
        leftFollowerSupplyCurrent = leftFollowerMotor.supplyCurrent
        leftFollowerStatorCurrent = leftFollowerMotor.statorCurrent

        rightFollowerTemperature = rightFollowerMotor.deviceTemp
        rightFollowerMotorVoltage = rightFollowerMotor.motorVoltage
        rightFollowerSupplyCurrent = rightFollowerMotor.supplyCurrent
        rightFollowerStatorCurrent = rightFollowerMotor.statorCurrent

        hoodTemperature = hoodMotor.deviceTemp
        hoodMotorVoltage = hoodMotor.motorVoltage
        hoodSupplyCurrent = hoodMotor.supplyCurrent
        hoodStatorCurrent = hoodMotor.statorCurrent
        hoodCurrentPos = hoodMotor.position
        hoodTargetPos = hoodMotor.closedLoopReference

        BaseStatusSignal.setUpdateFrequencyForAll(
            150.0, //  doesn't need to update that often
            leftLeaderTemperature,
            leftLeaderMotorVoltage,
            leftLeaderSupplyCurrent,
            leftLeaderStatorCurrent,
            rightLeaderTemperature,
            rightLeaderMotorVoltage,
            rightLeaderSupplyCurrent,
            rightLeaderStatorCurrent,
            leftFollowerTemperature,
            leftFollowerMotorVoltage,
            leftFollowerSupplyCurrent,
            leftFollowerStatorCurrent,
            rightFollowerTemperature,
            rightFollowerMotorVoltage,
            rightFollowerSupplyCurrent,
            rightFollowerStatorCurrent,
            hoodTemperature,
            hoodMotorVoltage,
            hoodSupplyCurrent,
            hoodStatorCurrent,
            hoodCurrentPos
        )

        ParentDevice.optimizeBusUtilizationForAll(leftLeaderMotor, leftFollowerMotor, rightLeaderMotor, rightFollowerMotor, hoodMotor)
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        BaseStatusSignal.refreshAll(
            leftLeaderTemperature,
            leftLeaderMotorVoltage,
            leftLeaderSupplyCurrent,
            leftLeaderStatorCurrent,
            rightLeaderTemperature,
            rightLeaderMotorVoltage,
            rightLeaderSupplyCurrent,
            rightLeaderStatorCurrent,
            leftFollowerTemperature,
            leftFollowerMotorVoltage,
            leftFollowerSupplyCurrent,
            leftFollowerStatorCurrent,
            rightFollowerTemperature,
            rightFollowerMotorVoltage,
            rightFollowerSupplyCurrent,
            rightFollowerStatorCurrent,
            hoodTemperature,
            hoodMotorVoltage,
            hoodSupplyCurrent,
            hoodStatorCurrent,
            hoodCurrentPos
        )

        leftLeaderMotorConnected = leftLeaderMotor.isAlive

        rightLeaderMotorConnected = rightLeaderMotor.isAlive

        leftFollowerMotorConnected = leftFollowerMotor.isAlive

        rightFollowerMotorConnected = rightFollowerMotor.isAlive

        leftLeaderMotorDisconnectedAlert.set(!leftLeaderMotorConnected)
        rightLeaderMotorDisconnectedAlert.set(!rightLeaderMotorConnected)
        leftFollowerMotorDisconnectedAlert.set(!leftFollowerMotorConnected)
        rightFollowerMotorDisconnectedAlert.set(!rightFollowerMotorConnected)
        hoodMotorDisconnectedAlert.set(!hoodMotorConnected)

        inputs.leftVoltage = leftLeaderMotorVoltage.getValue().`in`(Units.Volts)
        inputs.leftSupplyCurrent = leftLeaderSupplyCurrent.getValue().`in`(Units.Amps)
        inputs.leftStatorCurrent = leftLeaderStatorCurrent.getValue().`in`(Units.Amps)
        inputs.leftTemperature = leftLeaderTemperature.getValue().`in`(Units.Celsius)
        inputs.leftMotorIsConnected = leftLeaderMotorConnected

        inputs.rightVoltage = rightLeaderMotorVoltage.getValue().`in`(Units.Volts)
        inputs.rightSupplyCurrent = rightLeaderSupplyCurrent.getValue().`in`(Units.Amps)
        inputs.rightStatorCurrent = rightLeaderStatorCurrent.getValue().`in`(Units.Amps)
        inputs.rightTemperature = rightLeaderTemperature.getValue().`in`(Units.Celsius)
        inputs.rightMotorIsConnected = rightLeaderMotorConnected

        inputs.hoodVoltage = hoodMotorVoltage.getValue().`in`(Units.Volts)
        inputs.hoodSupplyCurrent = hoodSupplyCurrent.getValue().`in`(Units.Amps)
        inputs.hoodStatorCurrent = hoodStatorCurrent.getValue().`in`(Units.Amps)
        inputs.hoodTemperature = hoodTemperature.getValue().`in`(Units.Celsius)
        inputs.hoodMotorIsConnected = hoodMotorConnected
        inputs.hoodCurrentPos = hoodCurrentPos.getValue().`in`(Units.Radians)
        inputs.hoodTargetPos = hoodTargetPos.value
    }

    private var request = MotionMagicVoltage(HOOD_MIN_ANGLE)

    override fun runFlywheel(): Command {
        return runOnce({
            leftLeaderMotor.setVoltage(SHOOTER_VOLTAGE)
            rightLeaderMotor.setVoltage(SHOOTER_VOLTAGE)
        })
    }

    override fun stopFlywheel(): Command {
        return runOnce({
            leftLeaderMotor.stopMotor()
            rightLeaderMotor.stopMotor()
        })
    }

    override fun setHood(angle: Angle): Command {
        return runOnce({
            hoodMotor.setControl(request.withPosition(angle))
        })
    }

    override fun holdHood(): Command {
        return runOnce({
            hoodMotor.setControl(request.withPosition(hoodMotor.position.value.`in`(Radians)))
        })
    }

    override fun hoodUp(): Command {
        return runOnce({
            // hoodMotor.setControl(request.withPosition(hoodMotor.position.value.`in`(Radians) + Degrees.of(7.5).`in`(Radians)))
            hoodMotor.setVoltage(HOOD_VOLTAGE_CONTROL)
        })
    }

    override fun hoodDown(): Command {
        return runOnce({
            // hoodMotor.setControl(request.withPosition(hoodMotor.position.value.`in`(Radians) - Degrees.of(7.5).`in`(Radians)))
            hoodMotor.setVoltage(-HOOD_VOLTAGE_CONTROL)
        })
    }

    override fun stopHood(): Command {
        return runOnce({
            hoodMotor.setVoltage(0.0)
        })
    }

    override fun atTolerance(): Boolean {
        return abs(hoodMotor.position.valueAsDouble - hoodMotor.closedLoopReference.value) < HOOD_TOLERANCE.`in`(Radians)
    }
}
