package frc.team449.subsystems.climb

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
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
import frc.team449.Constants.IntakeConstants.PIVOT_STATOR_LIMIT
import frc.team449.Constants.IntakeConstants.PIVOT_SUPPLY_LIMIT
import frc.team449.subsystems.intake.pivot.PivotIO

class PivotIOHardware : PivotIO {
    // TODO: ALL THE NUMBERS...

    private val motor = TalonFX(Constants.IntakeConstants.PIVOT_MOTOR_ID) // x44
    private val config: TalonFXConfiguration

    private val motorConnected: Boolean
    private val motorVoltage: StatusSignal<Voltage>
    private val supplyCurrent: StatusSignal<Current>
    private val statorCurrent: StatusSignal<Current>
    private val temperature: StatusSignal<Temperature>

    private var targetVoltage = 0.0

    private val motorDisconnectedAlert =
        Alert(
            "Pivot motor disconnected (ID ${Constants.IntakeConstants.PIVOT_MOTOR_ID})",
            Alert.AlertType.kError,
        )

    init {
        val currentLimitConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(PIVOT_SUPPLY_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(PIVOT_STATOR_LIMIT)

        val motorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        config =
            TalonFXConfiguration()
                .withCurrentLimits(currentLimitConfigs)
                .withMotorOutput(motorOutput)

        motor.configurator.apply(config)

        motorVoltage = motor.motorVoltage
        supplyCurrent = motor.supplyCurrent
        statorCurrent = motor.statorCurrent
        temperature = motor.deviceTemp

        motorConnected =
            BaseStatusSignal.isAllGood(
                motorVoltage,
                supplyCurrent,
                statorCurrent,
                temperature,
            )

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            motorVoltage,
            supplyCurrent,
            statorCurrent,
        )
        ParentDevice.optimizeBusUtilizationForAll(motor)
    }

    override fun updateInputs(pivotInputs: PivotIO.PivotIOInputs) {
        BaseStatusSignal.refreshAll(
            motorVoltage,
            supplyCurrent,
            statorCurrent,
            temperature,
        )
        pivotInputs.targetVoltage = targetVoltage
        pivotInputs.currentVoltage = motorVoltage.getValue().`in`(Units.Volts)
        pivotInputs.supplyCurrent = supplyCurrent.getValue().`in`(Units.Amps)
        pivotInputs.statorCurrent = statorCurrent.getValue().`in`(Units.Amps)
        pivotInputs.temperature = temperature.getValue().`in`(Units.Celsius)
        pivotInputs.motorIsConnected =
            BaseStatusSignal.isAllGood(
                motorVoltage,
                supplyCurrent,
                statorCurrent,
                temperature,
            )
        motorDisconnectedAlert.set(!motorConnected)
    }

    override fun setVoltage(voltage: Double) {
        targetVoltage = voltage
        motor.setVoltage(targetVoltage)
    }
}
