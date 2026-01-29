package frc.team449.subsystems.intake.roller

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import frc.team449.Constants.IntakeConstants.ROLLER_KP
import frc.team449.Constants.IntakeConstants.ROLLER_KV
import frc.team449.Constants.IntakeConstants.ROLLER_MOTOR_ID
import frc.team449.Constants.IntakeConstants.ROLLER_MOTOR_ID2
import frc.team449.Constants.IntakeConstants.ROLLER_STATOR_LIMIT
import frc.team449.Constants.IntakeConstants.ROLLER_SUPPLY_LIMIT

class RollerIOHardware : RollerIO {
    private val motor = TalonFX(ROLLER_MOTOR_ID)
    private val motor2 = TalonFX(ROLLER_MOTOR_ID2)
    private var config = TalonFXConfiguration()

    private val motorConnected: Boolean = true
    private val velocity: StatusSignal<AngularVelocity>
    private val motorVoltage: StatusSignal<Voltage>
    private val supplyCurrent: StatusSignal<Current>
    private val statorCurrent: StatusSignal<Current>
    private val temperature: StatusSignal<Temperature>

    init {
        val currentLimitConfigs: CurrentLimitsConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(ROLLER_SUPPLY_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(ROLLER_STATOR_LIMIT)

        val motorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        val slot0Configs: Slot0Configs =
            Slot0Configs()
                .withKP(ROLLER_KP)
                .withKV(ROLLER_KV)

        config =
            TalonFXConfiguration()
                .withCurrentLimits(currentLimitConfigs)
                .withMotorOutput(motorOutput)
                .withSlot0(slot0Configs)

        motor.configurator.apply(config)
        motor2.configurator.apply(config)

        velocity = motor.velocity
        temperature = motor.deviceTemp
        motorVoltage = motor.motorVoltage
        supplyCurrent = motor.supplyCurrent
        statorCurrent = motor.statorCurrent

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            motorVoltage,
            supplyCurrent,
            statorCurrent,
            velocity,
        )

        ParentDevice.optimizeBusUtilizationForAll(motor)
    }

    override fun updateInputs(rollerInputs: RollerIO.RollerIOInputs) {
        BaseStatusSignal.refreshAll(
            velocity,
            motorVoltage,
            supplyCurrent,
            statorCurrent,
            temperature,
        )
        rollerInputs.voltage = motorVoltage.getValue().`in`(Units.Volts)
        rollerInputs.supplyCurrent = supplyCurrent.getValue().`in`(Units.Amps)
        rollerInputs.statorCurrent = statorCurrent.getValue().`in`(Units.Amps)
        rollerInputs.temperature = temperature.getValue().`in`(Units.Celsius)
        rollerInputs.motorIsConnected =
            BaseStatusSignal.isAllGood(
                velocity,
                motorVoltage,
                supplyCurrent,
                statorCurrent,
                temperature,
            )
    }

    override fun setVelocity(velocity: Double) {
        motor.setControl(
            VelocityVoltage(velocity)
                .withSlot(0),
        )
        motor2.setControl(
            VelocityVoltage(velocity)
                .withSlot(0),
        )
    }

    fun stop() {
        motor.stopMotor()
        motor2.stopMotor()
    }
}
