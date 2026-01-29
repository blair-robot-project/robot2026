package frc.team449.subsystems.intake.pivot

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

class PivotIOHardware : PivotIO {
    private val lead = TalonFX(1)
    private val follower = TalonFX(2)

    private val config: TalonFXConfiguration

    init {

        val currentLimitConfigs: CurrentLimitsConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40.0)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(80.0)

        val motorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        val feedback =
            FeedbackConfigs()
                .withSensorToMechanismRatio(1.0)

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
    }

    override fun updateInputs(pivotInputs: PivotIO.PivotIOInputs) {
    }
}
