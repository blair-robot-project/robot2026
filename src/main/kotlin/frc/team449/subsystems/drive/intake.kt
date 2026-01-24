package frc.team449.subsystems.drive

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj.motorcontrol.Talon
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.InstantCommand

class Intake {
    private val motor: TalonFX = TalonFX(25)
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

        config =
            TalonFXConfiguration()
                .withCurrentLimits(currentLimitConfigs)
                .withMotorOutput(motorOutput)

        motor.configurator.apply(config)
    }

    fun runIntake(): Command = InstantCommand({ motor.setVoltage(5.0) })

    fun stop(): Command = InstantCommand({ motor.stopMotor() })
}
