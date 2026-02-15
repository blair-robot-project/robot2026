package frc.team449.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
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
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
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
import frc.team449.Constants.ShooterConstants.HOOD_TOLERANCE
import frc.team449.Constants.ShooterConstants.LEFT_FLYWHEEL_FOLLOWER_ID
import frc.team449.Constants.ShooterConstants.LEFT_FLYWHEEL_LEADER_ID
import frc.team449.Constants.ShooterConstants.RIGHT_FLYWHEEL_FOLLOWER_ID
import frc.team449.Constants.ShooterConstants.RIGHT_FLYWHEEL_LEADER_ID
import frc.team449.Constants.ShooterConstants.TOLERANCE_DEBOUNCE_TIME
import frc.team449.Constants.ShooterConstants.TOLERANCE_DEBOUNCE_TYPE
import kotlin.math.abs

class ShooterIOHardware : ShooterIO {
    // TODO: a little constants
    // TODO: interpolating double map for flywheel velocity + hood angle

    private val leftLeaderMotor = TalonFX(LEFT_FLYWHEEL_LEADER_ID)
    private val leftFollowerMotor = TalonFX(LEFT_FLYWHEEL_FOLLOWER_ID)
    private val rightLeaderMotor = TalonFX(RIGHT_FLYWHEEL_LEADER_ID)
    private val rightFollowerMotor = TalonFX(RIGHT_FLYWHEEL_FOLLOWER_ID)
    private val hoodMotor = TalonFX(HOOD_MOTOR_ID)

    private val flywheelVelocityRequest = VelocityVoltage(0.0).withSlot(0)
    private val flywheelVoltageRequest = VoltageOut(0.0)
    private val hoodPositionRequest = PositionVoltage(0.0).withSlot(0)
    private val hoodVoltageRequest = VoltageOut(0.0)

    private val toleranceDebouncer: Debouncer = Debouncer(TOLERANCE_DEBOUNCE_TIME, TOLERANCE_DEBOUNCE_TYPE)

    private val flywheelConfig: TalonFXConfiguration
    private val hoodConfig: TalonFXConfiguration
    private var leftLeaderMotorConnected = leftLeaderMotor.isAlive
    private var rightLeaderMotorConnected = rightLeaderMotor.isAlive
    private var leftFollowerMotorConnected = leftFollowerMotor.isAlive
    private var rightFollowerMotorConnected = rightFollowerMotor.isAlive
    private var hoodMotorConnected = hoodMotor.isAlive

    private val leftLeaderTemperature = leftLeaderMotor.deviceTemp
    private val leftLeaderMotorVoltage = leftLeaderMotor.motorVoltage
    private val leftLeaderSupplyCurrent = leftLeaderMotor.supplyCurrent
    private val leftLeaderStatorCurrent = leftLeaderMotor.statorCurrent

    private val rightLeaderTemperature = rightLeaderMotor.deviceTemp
    private val rightLeaderMotorVoltage = rightLeaderMotor.motorVoltage
    private val rightLeaderSupplyCurrent = rightLeaderMotor.supplyCurrent
    private val rightLeaderStatorCurrent = rightLeaderMotor.statorCurrent

    private val leftFollowerTemperature = leftFollowerMotor.deviceTemp
    private val leftFollowerMotorVoltage = leftFollowerMotor.motorVoltage
    private val leftFollowerSupplyCurrent = leftFollowerMotor.supplyCurrent
    private val leftFollowerStatorCurrent = leftFollowerMotor.statorCurrent

    private val rightFollowerTemperature = rightFollowerMotor.deviceTemp
    private val rightFollowerMotorVoltage = rightFollowerMotor.motorVoltage
    private val rightFollowerSupplyCurrent = rightFollowerMotor.supplyCurrent
    private val rightFollowerStatorCurrent = rightFollowerMotor.statorCurrent

    private val hoodTemperature = hoodMotor.deviceTemp
    private val hoodMotorVoltage = hoodMotor.motorVoltage
    private val hoodSupplyCurrent = hoodMotor.supplyCurrent
    private val hoodStatorCurrent = hoodMotor.statorCurrent
    private val hoodCurrentPos = hoodMotor.position
    private val hoodTargetPos = hoodMotor.closedLoopReference

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
            "hood motor disconnected (ID $HOOD_MOTOR_ID)",
            Alert.AlertType.kError
        )

    init {
        val flywheelCurrentLimitConfigs: CurrentLimitsConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimit(FLYWHEEL_SUPPLY_LIM)
                .withStatorCurrentLimit(FLYWHEEL_STATOR_LIM)

        val hoodCurrentLimitConfigs: CurrentLimitsConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimit(HOOD_SUPPLY_LIM)
                .withStatorCurrentLimit(HOOD_STATOR_LIM)

        val flywheelMotorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
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

        val flywheelSlot0Configs =
            Slot0Configs()
                .withKP(FLYWHEEL_KP)
                .withKI(FLYWHEEL_KI)
                .withKD(FLYWHEEL_KD)
                .withKS(FLYWHEEL_KS)
                .withKV(FLYWHEEL_KV)

        val hoodSlot0Configs =
            Slot0Configs()
                .withKP(HOOD_KP)
                .withKI(HOOD_KI)
                .withKD(HOOD_KD)
                .withKS(HOOD_KS)
                .withKG(HOOD_KG)
                .withKV(HOOD_KV)

        flywheelConfig =
            TalonFXConfiguration()
                .withCurrentLimits(flywheelCurrentLimitConfigs)
                .withMotorOutput(flywheelMotorOutput)
                .withFeedback(flywheelFeedback)
                .withSlot0(flywheelSlot0Configs)

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
                .withSlot0(hoodSlot0Configs)

        hoodMotor.configurator.apply(hoodConfig)

        BaseStatusSignal.setUpdateFrequencyForAll(
            4.0, //  doesn't need to update that often
            leftLeaderTemperature,
            leftLeaderSupplyCurrent,
            rightLeaderTemperature,
            rightLeaderSupplyCurrent,
            leftFollowerTemperature,
            leftFollowerMotorVoltage,
            leftFollowerSupplyCurrent,
            rightFollowerTemperature,
            rightFollowerMotorVoltage,
            rightFollowerSupplyCurrent,
            hoodTemperature,
            hoodMotorVoltage,
            hoodSupplyCurrent,
            hoodTargetPos
        )

        BaseStatusSignal.setUpdateFrequencyForAll(
            150.0,
            leftLeaderMotorVoltage,
            rightLeaderMotorVoltage,
        )
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            leftLeaderStatorCurrent,
            rightLeaderStatorCurrent,
            leftFollowerStatorCurrent,
            rightFollowerStatorCurrent,
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
        hoodMotorConnected = hoodMotor.isAlive

        leftLeaderMotorDisconnectedAlert.set(!leftLeaderMotorConnected)
        rightLeaderMotorDisconnectedAlert.set(!rightLeaderMotorConnected)
        leftFollowerMotorDisconnectedAlert.set(!leftFollowerMotorConnected)
        rightFollowerMotorDisconnectedAlert.set(!rightFollowerMotorConnected)
        hoodMotorDisconnectedAlert.set(!hoodMotorConnected)

        inputs.leftVoltage = leftLeaderMotorVoltage.getValue().`in`(Volts)
        inputs.leftSupplyCurrent = leftLeaderSupplyCurrent.getValue().`in`(Amps)
        inputs.leftStatorCurrent = leftLeaderStatorCurrent.getValue().`in`(Amps)
        inputs.leftTemperature = leftLeaderTemperature.getValue().`in`(Celsius)
        inputs.leftMotorIsConnected = leftLeaderMotorConnected
        inputs.leftFollowerMotorIsConnected = leftFollowerMotorConnected

        inputs.rightVoltage = rightLeaderMotorVoltage.getValue().`in`(Volts)
        inputs.rightSupplyCurrent = rightLeaderSupplyCurrent.getValue().`in`(Amps)
        inputs.rightStatorCurrent = rightLeaderStatorCurrent.getValue().`in`(Amps)
        inputs.rightTemperature = rightLeaderTemperature.getValue().`in`(Celsius)
        inputs.rightMotorIsConnected = rightLeaderMotorConnected
        inputs.rightFollowerMotorIsConneted = rightFollowerMotorConnected

        inputs.hoodVoltage = hoodMotorVoltage.getValue().`in`(Volts)
        inputs.hoodSupplyCurrent = hoodSupplyCurrent.getValue().`in`(Amps)
        inputs.hoodStatorCurrent = hoodStatorCurrent.getValue().`in`(Amps)
        inputs.hoodTemperature = hoodTemperature.getValue().`in`(Celsius)
        inputs.hoodMotorIsConnected = hoodMotorConnected
        inputs.hoodCurrentPos = hoodCurrentPos.getValue().`in`(Radians)
        inputs.hoodTargetPos = hoodTargetPos.value
        inputs.leftFlywheelVelocity = leftLeaderMotor.velocity.value.`in`(RadiansPerSecond)
        inputs.rightFlywheelVelocity = rightLeaderMotor.velocity.value.`in`(RadiansPerSecond)
    }

    override fun setFlywheelVelocity(velocity: AngularVelocity) {
        leftLeaderMotor.setControl(VelocityVoltage(velocity).withSlot(0))
        rightLeaderMotor.setControl(VelocityVoltage(velocity).withSlot(0))
    }

    override fun setHoodPosition(angle: Angle) {
        hoodMotor.setControl(PositionVoltage(angle).withSlot(0))
    }

    override fun inTolerance(): Boolean {
        return toleranceDebouncer.calculate(abs(hoodMotor.closedLoopError.value) < HOOD_TOLERANCE.`in`(Radians))
    }

    override fun getHoodPosition(): Angle {
        return hoodMotor.position.value
    }

    override fun setHoodVoltage(voltage: Double) {
        hoodMotor.setVoltage(voltage)
    }

    override fun stopHoodVoltage() {
        hoodMotor.setVoltage(0.0)
    }

    override fun getHoodStatorCurrent(): Current {
        return hoodMotor.statorCurrent.value
    }

    override fun resetHoodPosition(angle: Angle) {
        hoodMotor.setPosition(angle)
    }

    override fun setFlywheelVoltage(voltage: Double) {
        leftLeaderMotor.setVoltage(voltage)
        rightLeaderMotor.setVoltage(voltage)
    }
}
