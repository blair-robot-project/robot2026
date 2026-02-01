package frc.team449.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants

class ShooterIOHardware : ShooterIO {
    // TODO: p much all the constants

    private val leftLeaderMotor = TalonFX(Constants.ShooterConstants.LEFT_FLYWHEEL_LEADER_ID)
    private val leftFollowerMotor = TalonFX(Constants.ShooterConstants.LEFT_FLYWHEEL_FOLLOWER_ID)

    private val rightLeaderMotor = TalonFX(Constants.ShooterConstants.RIGHT_FLYWHEEL_LEADER_ID)
    private val rightFollowerMotor = TalonFX(Constants.ShooterConstants.RIGHT_FLYWHEEL_FOLLOWER_ID)

    private val hoodMotor = TalonFX(Constants.ShooterConstants.HOOD_MOTOR_ID)

    private val flywheelConfig: TalonFXConfiguration
    private val hoodConfig: TalonFXConfiguration

    private var leftLeaderMotorConnected: Boolean = true
    private var rightLeaderMotorConnected: Boolean = true
    private var hoodMotorConnected: Boolean = true

    private val lmotorVoltage: StatusSignal<Voltage>
    private val lsupplyCurrent: StatusSignal<Current>
    private val lstatorCurrent: StatusSignal<Current>
    private val ltemperature: StatusSignal<Temperature>

    private val rmotorVoltage: StatusSignal<Voltage>
    private val rsupplyCurrent: StatusSignal<Current>
    private val rstatorCurrent: StatusSignal<Current>
    private val rtemperature: StatusSignal<Temperature>

    private val hmotorVoltage: StatusSignal<Voltage>
    private val hsupplyCurrent: StatusSignal<Current>
    private val hstatorCurrent: StatusSignal<Current>
    private val htemperature: StatusSignal<Temperature>
    private val hcurrentPos: StatusSignal<Angle>
    private val htargetPos: StatusSignal<Double>

    private val leftMotorDisconnectedAlert =
        Alert(
            "Climb motor disconnected (ID ${Constants.ShooterConstants.LEFT_FLYWHEEL_LEADER_ID})",
            Alert.AlertType.kError,
        )
    private val rightMotorDisconnectedAlert =
        Alert(
            "Climb motor disconnected (ID ${Constants.ShooterConstants.RIGHT_FLYWHEEL_LEADER_ID})",
            Alert.AlertType.kError,
        )
    private val hoodMotorDisconnectedAlert =
        Alert(
            "Hood motor disconnected (ID ${Constants.ShooterConstants.HOOD_MOTOR_ID}",
            Alert.AlertType.kError
        )

    init {
        val currentLimitConfigs: CurrentLimitsConfigs =
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

        val motorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        val hoodMotorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        val feedback =
            FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.ShooterConstants.FLYWHEEL_GEARING)

        val hoodFeedback =
            FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.ShooterConstants.HOOD_GEARING)

        val motionMagicConfigs =
            MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(Constants.ShooterConstants.HOOD_CRUISE_VELOCITY)
                .withMotionMagicAcceleration(Constants.ShooterConstants.HOOD_ACCELERATION)

        flywheelConfig =
            TalonFXConfiguration()
                .withCurrentLimits(currentLimitConfigs)
                .withMotorOutput(motorOutput)
                .withFeedback(feedback)

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
                .withMotionMagic(motionMagicConfigs)

        ltemperature = leftLeaderMotor.deviceTemp
        lmotorVoltage = leftLeaderMotor.motorVoltage
        lsupplyCurrent = leftLeaderMotor.supplyCurrent
        lstatorCurrent = leftLeaderMotor.statorCurrent

        rtemperature = rightLeaderMotor.deviceTemp
        rmotorVoltage = rightLeaderMotor.motorVoltage
        rsupplyCurrent = rightLeaderMotor.supplyCurrent
        rstatorCurrent = rightLeaderMotor.statorCurrent

        htemperature = hoodMotor.deviceTemp
        hmotorVoltage = hoodMotor.motorVoltage
        hsupplyCurrent = hoodMotor.supplyCurrent
        hstatorCurrent = hoodMotor.statorCurrent
        hcurrentPos = hoodMotor.position
        htargetPos = hoodMotor.closedLoopReference

        BaseStatusSignal.setUpdateFrequencyForAll(
            150.0, //  doesn't need to update that often
            ltemperature,
            lmotorVoltage,
            lsupplyCurrent,
            lstatorCurrent,
            rtemperature,
            rmotorVoltage,
            rsupplyCurrent,
            rstatorCurrent,
            htemperature,
            hmotorVoltage,
            hsupplyCurrent,
            hstatorCurrent,
            hcurrentPos
        )

        ParentDevice.optimizeBusUtilizationForAll(leftLeaderMotor, leftFollowerMotor, rightLeaderMotor, rightFollowerMotor)
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        BaseStatusSignal.refreshAll(
            lmotorVoltage,
            lsupplyCurrent,
            lstatorCurrent,
            ltemperature,
            rmotorVoltage,
            rsupplyCurrent,
            rstatorCurrent,
            rtemperature,
        )

        leftLeaderMotorConnected =
            BaseStatusSignal.isAllGood(
                lmotorVoltage,
                lsupplyCurrent,
                lstatorCurrent,
                ltemperature,
            )

        rightLeaderMotorConnected =
            BaseStatusSignal.isAllGood(
                rmotorVoltage,
                rsupplyCurrent,
                rstatorCurrent,
                rtemperature,
            )

        leftMotorDisconnectedAlert.set(!leftLeaderMotorConnected)
        rightMotorDisconnectedAlert.set(!rightLeaderMotorConnected)
        hoodMotorDisconnectedAlert.set(!hoodMotorConnected)

        inputs.leftVoltage = lmotorVoltage.getValue().`in`(Units.Volts)
        inputs.leftSupplyCurrent = lsupplyCurrent.getValue().`in`(Units.Amps)
        inputs.leftStatorCurrent = lstatorCurrent.getValue().`in`(Units.Amps)
        inputs.leftTemperature = ltemperature.getValue().`in`(Units.Celsius)
        inputs.leftMotorIsConnected = leftLeaderMotorConnected

        inputs.rightVoltage = rmotorVoltage.getValue().`in`(Units.Volts)
        inputs.rightSupplyCurrent = rsupplyCurrent.getValue().`in`(Units.Amps)
        inputs.rightStatorCurrent = rstatorCurrent.getValue().`in`(Units.Amps)
        inputs.rightTemperature = rtemperature.getValue().`in`(Units.Celsius)
        inputs.rightMotorIsConnected = rightLeaderMotorConnected

        inputs.hoodVoltage = hmotorVoltage.getValue().`in`(Units.Volts)
        inputs.hoodSupplyCurrent = hsupplyCurrent.getValue().`in`(Units.Amps)
        inputs.hoodStatorCurrent = hstatorCurrent.getValue().`in`(Units.Amps)
        inputs.hoodTemperature = htemperature.getValue().`in`(Units.Celsius)
        inputs.hoodMotorIsConnected = hoodMotorConnected
        inputs.hoodCurrentPos = hcurrentPos.getValue().`in`(Units.Radians)
        inputs.hoodTargetPos = htargetPos.value
    }

    override fun run(voltage: Double) {
        leftLeaderMotor.setVoltage(voltage)
        rightLeaderMotor.setVoltage(voltage)
    }

    override fun stop() {
        leftLeaderMotor.stopMotor()
        rightLeaderMotor.stopMotor()
    }
}
