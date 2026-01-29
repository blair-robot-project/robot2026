package frc.team449.subsystems.climb

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
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import frc.team449.Constants
import frc.team449.Constants.IntakeConstants.PIVOT_STATOR_LIMIT
import frc.team449.Constants.IntakeConstants.PIVOT_SUPPLY_LIMIT
import frc.team449.subsystems.intake.pivot.PivotIO

class PivotIOHardware : PivotIO {
    // TODO: ALL THE NUMBERS...

    private val motor = TalonFX(Constants.IntakeConstants.PIVOT_MOTOR_ID)
    private val motor2 = TalonFX(Constants.IntakeConstants.PIVOT_MOTOR_ID2)

    private val config: TalonFXConfiguration

    private val motorConnected: Boolean
    private val currentPos: StatusSignal<Angle>
    private val motorVoltage: StatusSignal<Voltage>
    private val supplyCurrent: StatusSignal<Current>
    private val statorCurrent: StatusSignal<Current>
    private val temperature: StatusSignal<Temperature>

    init {
        val currentLimitConfigs: CurrentLimitsConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(PIVOT_SUPPLY_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(PIVOT_STATOR_LIMIT)

        val motorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        val feedback =
            FeedbackConfigs()
                .withSensorToMechanismRatio(
                    1.0,
                )

        val motionMagicConfigs =
            MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(3.0)
                .withMotionMagicAcceleration(5.0)

        config =
            TalonFXConfiguration()
                .withCurrentLimits(currentLimitConfigs)
                .withMotorOutput(motorOutput)
                .withFeedback(feedback)
                .withMotionMagic(motionMagicConfigs)

        motor.configurator.apply(config)
        motor2.configurator.apply(config)

        currentPos = motor.position
        temperature = motor.deviceTemp
        motorVoltage = motor.motorVoltage
        supplyCurrent = motor.supplyCurrent
        statorCurrent = motor.statorCurrent
        motorConnected =
            BaseStatusSignal.isAllGood(
                currentPos,
                motorVoltage,
                supplyCurrent,
                statorCurrent,
                temperature,
            )

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            motorVoltage,
            supplyCurrent,
            statorCurrent,
            currentPos,
        )

        ParentDevice.optimizeBusUtilizationForAll(motor)
    }

    override fun updateInputs(pivotInputs: PivotIO.PivotIOInputs) {
        BaseStatusSignal.refreshAll(
            currentPos,
            motorVoltage,
            supplyCurrent,
            statorCurrent,
            temperature,
        )

        pivotInputs.currentAngle = currentPos.getValue().`in`(Units.Degrees)
        pivotInputs.voltage = motorVoltage.getValue().`in`(Units.Volts)
        pivotInputs.supplyCurrent = supplyCurrent.getValue().`in`(Units.Amps)
        pivotInputs.statorCurrent = statorCurrent.getValue().`in`(Units.Amps)
        pivotInputs.temperature = temperature.getValue().`in`(Units.Celsius)
    }

    private var request = MotionMagicVoltage(0.0)

    override fun setAngle(angle: Angle) {
        motor.setControl(request.withPosition(angle))
        motor2.setControl(request.withPosition(angle))
    }
}
