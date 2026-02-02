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
import frc.team449.Constants
import frc.team449.Constants.ShooterConstants.HOOD_MOTOR_ID
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
    // TODO: voltage control for hood

    private val leftLeaderMotor = TalonFX(LEFT_FLYWHEEL_LEADER_ID)
    private val leftFollowerMotor = TalonFX(LEFT_FLYWHEEL_FOLLOWER_ID)

    private val rightLeaderMotor = TalonFX(RIGHT_FLYWHEEL_LEADER_ID)
    private val rightFollowerMotor = TalonFX(RIGHT_FLYWHEEL_FOLLOWER_ID)

    private val hoodMotor = TalonFX(HOOD_MOTOR_ID)

    private val flywheelConfig: TalonFXConfiguration
    private val hoodConfig: TalonFXConfiguration

    private var leftLeaderMotorConnected: Boolean = true
    private var rightLeaderMotorConnected: Boolean = true
    private var hoodMotorConnected: Boolean = true

    private val leftMotorVoltage: StatusSignal<Voltage>
    private val leftSupplyCurrent: StatusSignal<Current>
    private val leftStatorCurrent: StatusSignal<Current>
    private val leftTemperature: StatusSignal<Temperature>

    private val rightMotorVoltage: StatusSignal<Voltage>
    private val rightSupplyCurrent: StatusSignal<Current>
    private val rightStatorCurrent: StatusSignal<Current>
    private val rightTemperature: StatusSignal<Temperature>

    private val hoodMotorVoltage: StatusSignal<Voltage>
    private val hoodSupplyCurrent: StatusSignal<Current>
    private val hoodStatorCurrent: StatusSignal<Current>
    private val hoodTemperature: StatusSignal<Temperature>
    private val hoodCurrentPos: StatusSignal<Angle>
    private val hoodTargetPos: StatusSignal<Double>

    private val leftMotorDisconnectedAlert =
        Alert(
            "left leader shooter flywheel motor disconnected (ID $LEFT_FLYWHEEL_LEADER_ID)",
            Alert.AlertType.kError,
        )
    private val rightMotorDisconnectedAlert =
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
                .withSupplyCurrentLimit(Constants.ShooterConstants.FLYWHEEL_SUPPLY_LIM)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Constants.ShooterConstants.FLYWHEEL_STATOR_LIM)

        val hoodCurrentLimitConfigs: CurrentLimitsConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.ShooterConstants.HOOD_SUPPLY_LIM)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Constants.ShooterConstants.HOOD_STATOR_LIM)

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
                .withSensorToMechanismRatio(Constants.ShooterConstants.FLYWHEEL_GEARING)

        val hoodFeedback =
            FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.ShooterConstants.HOOD_GEARING)

        val hoodMotionMagicConfigs =
            MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.ShooterConstants.HOOD_CRUISE_VELOCITY)
                .withMotionMagicAcceleration(Constants.ShooterConstants.HOOD_ACCELERATION)

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

        leftTemperature = leftLeaderMotor.deviceTemp
        leftMotorVoltage = leftLeaderMotor.motorVoltage
        leftSupplyCurrent = leftLeaderMotor.supplyCurrent
        leftStatorCurrent = leftLeaderMotor.statorCurrent

        rightTemperature = rightLeaderMotor.deviceTemp
        rightMotorVoltage = rightLeaderMotor.motorVoltage
        rightSupplyCurrent = rightLeaderMotor.supplyCurrent
        rightStatorCurrent = rightLeaderMotor.statorCurrent

        hoodTemperature = hoodMotor.deviceTemp
        hoodMotorVoltage = hoodMotor.motorVoltage
        hoodSupplyCurrent = hoodMotor.supplyCurrent
        hoodStatorCurrent = hoodMotor.statorCurrent
        hoodCurrentPos = hoodMotor.position
        hoodTargetPos = hoodMotor.closedLoopReference

        BaseStatusSignal.setUpdateFrequencyForAll(
            150.0, //  doesn't need to update that often
            leftTemperature,
            leftMotorVoltage,
            leftSupplyCurrent,
            leftStatorCurrent,
            rightTemperature,
            rightMotorVoltage,
            rightSupplyCurrent,
            rightStatorCurrent,
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
            leftMotorVoltage,
            leftSupplyCurrent,
            leftStatorCurrent,
            leftTemperature,
            rightMotorVoltage,
            rightSupplyCurrent,
            rightStatorCurrent,
            rightTemperature,
        )

        leftLeaderMotorConnected =
            BaseStatusSignal.isAllGood(
                leftMotorVoltage,
                leftSupplyCurrent,
                leftStatorCurrent,
                leftTemperature,
            )

        rightLeaderMotorConnected =
            BaseStatusSignal.isAllGood(
                rightMotorVoltage,
                rightSupplyCurrent,
                rightStatorCurrent,
                rightTemperature,
            )

        leftMotorDisconnectedAlert.set(!leftLeaderMotorConnected)
        rightMotorDisconnectedAlert.set(!rightLeaderMotorConnected)
        hoodMotorDisconnectedAlert.set(!hoodMotorConnected)

        inputs.leftVoltage = leftMotorVoltage.getValue().`in`(Units.Volts)
        inputs.leftSupplyCurrent = leftSupplyCurrent.getValue().`in`(Units.Amps)
        inputs.leftStatorCurrent = leftStatorCurrent.getValue().`in`(Units.Amps)
        inputs.leftTemperature = leftTemperature.getValue().`in`(Units.Celsius)
        inputs.leftMotorIsConnected = leftLeaderMotorConnected

        inputs.rightVoltage = rightMotorVoltage.getValue().`in`(Units.Volts)
        inputs.rightSupplyCurrent = rightSupplyCurrent.getValue().`in`(Units.Amps)
        inputs.rightStatorCurrent = rightStatorCurrent.getValue().`in`(Units.Amps)
        inputs.rightTemperature = rightTemperature.getValue().`in`(Units.Celsius)
        inputs.rightMotorIsConnected = rightLeaderMotorConnected

        inputs.hoodVoltage = hoodMotorVoltage.getValue().`in`(Units.Volts)
        inputs.hoodSupplyCurrent = hoodSupplyCurrent.getValue().`in`(Units.Amps)
        inputs.hoodStatorCurrent = hoodStatorCurrent.getValue().`in`(Units.Amps)
        inputs.hoodTemperature = hoodTemperature.getValue().`in`(Units.Celsius)
        inputs.hoodMotorIsConnected = hoodMotorConnected
        inputs.hoodCurrentPos = hoodCurrentPos.getValue().`in`(Units.Radians)
        inputs.hoodTargetPos = hoodTargetPos.value
    }

    private var request = MotionMagicVoltage(Constants.ShooterConstants.HOOD_MIN_ANGLE)

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
        return abs(hoodMotor.position.valueAsDouble - hoodMotor.closedLoopReference.value) < Constants.ShooterConstants.HOOD_TOLERANCE.`in`(Radians)
    }
}
