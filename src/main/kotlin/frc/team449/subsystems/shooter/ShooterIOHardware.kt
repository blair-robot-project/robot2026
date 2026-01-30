package frc.team449.subsystems.shooter

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
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

    private val config: TalonFXConfiguration

    private var leftMotorConnected: Boolean = true
    private var rightMotorConnected: Boolean = true

    private val lmotorVoltage: StatusSignal<Voltage>
    private val lsupplyCurrent: StatusSignal<Current>
    private val lstatorCurrent: StatusSignal<Current>
    private val ltemperature: StatusSignal<Temperature>

    private val rmotorVoltage: StatusSignal<Voltage>
    private val rsupplyCurrent: StatusSignal<Current>
    private val rstatorCurrent: StatusSignal<Current>
    private val rtemperature: StatusSignal<Temperature>

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

    init {
        val currentLimitConfigs: CurrentLimitsConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Constants.ShooterConstants.SUPPLY_LIM)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(Constants.ShooterConstants.STATOR_LIM)

        val motorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        val feedback =
            FeedbackConfigs()
                .withSensorToMechanismRatio(Constants.ShooterConstants.GEARING)

        val motionMagicConfigs =
            MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(2.0)
                .withMotionMagicAcceleration(5.0)

        config =
            TalonFXConfiguration()
                .withCurrentLimits(currentLimitConfigs)
                .withMotorOutput(motorOutput)
                .withFeedback(feedback)
                .withMotionMagic(motionMagicConfigs)

        leftLeaderMotor.configurator.apply(config)
        leftFollowerMotor.configurator.apply(config)
        rightLeaderMotor.configurator.apply(config)
        rightFollowerMotor.configurator.apply(config)

        ltemperature = leftLeaderMotor.deviceTemp
        lmotorVoltage = leftLeaderMotor.motorVoltage
        lsupplyCurrent = leftLeaderMotor.supplyCurrent
        lstatorCurrent = leftLeaderMotor.statorCurrent

        rtemperature = leftLeaderMotor.deviceTemp
        rmotorVoltage = leftLeaderMotor.motorVoltage
        rsupplyCurrent = leftLeaderMotor.supplyCurrent
        rstatorCurrent = leftLeaderMotor.statorCurrent

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

        leftMotorConnected =
            BaseStatusSignal.isAllGood(
                lmotorVoltage,
                lsupplyCurrent,
                lstatorCurrent,
                ltemperature,
            )

        rightMotorConnected =
            BaseStatusSignal.isAllGood(
                rmotorVoltage,
                rsupplyCurrent,
                rstatorCurrent,
                rtemperature,
            )

        leftMotorDisconnectedAlert.set(!leftMotorConnected)

        inputs.leftVoltage = lmotorVoltage.getValue().`in`(Units.Volts)
        inputs.leftSupplyCurrent = lsupplyCurrent.getValue().`in`(Units.Amps)
        inputs.leftStatorCurrent = lstatorCurrent.getValue().`in`(Units.Amps)
        inputs.leftTemperature = ltemperature.getValue().`in`(Units.Celsius)

        inputs.rightVoltage = rmotorVoltage.getValue().`in`(Units.Volts)
        inputs.rightSupplyCurrent = rsupplyCurrent.getValue().`in`(Units.Amps)
        inputs.rightStatorCurrent = rstatorCurrent.getValue().`in`(Units.Amps)
        inputs.rightTemperature = rtemperature.getValue().`in`(Units.Celsius)
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