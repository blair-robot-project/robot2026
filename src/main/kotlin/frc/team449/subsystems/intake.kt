package frc.team449.subsystems

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.Logger

class Intake : SubsystemBase() {
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

        val slot0Configs: Slot0Configs =
            Slot0Configs()
                .withKP(10.0)
                .withKV(0.1)
        config =
            TalonFXConfiguration()
                .withCurrentLimits(currentLimitConfigs)
                .withMotorOutput(motorOutput)
                .withSlot0(slot0Configs)

        motor.configurator.apply(config)
    }

    val request =
        VelocityVoltage(50.0)
            .withFeedForward(3.0)
            .withSlot(0)

    fun runIntake(): Command =
        InstantCommand({
            motor.setControl(request)

//            Logger.recordOutput("motor velocity ", motor.velocity.value)
//            Logger.recordOutput("target velocity ", request.Velocity)
//            Logger.recordOutput("motor voltage ", motor.motorVoltage.valueAsDouble)
//            Logger.recordOutput("motor current ", motor.supplyCurrent.valueAsDouble)
        }, this)

    fun stop(): Command = InstantCommand({ motor.stopMotor() }, this)

    fun run(): Command = InstantCommand({ motor.setVoltage(8.0) }, this)
}
