package frc.team449.subsystems.intake.roller

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.system.plant.LinearSystemId.identifyVelocitySystem
import edu.wpi.first.wpilibj.simulation.LinearSystemSim
import frc.team449.Constants.IntakeConstants.ROLLER_KP
import frc.team449.Constants.IntakeConstants.ROLLER_KV

class RollerIOSim : RollerIO {
    private val leftMotor = TalonFX(41)
    private val rightMotor = TalonFX(21)
    private val leftMotorSim = leftMotor.simState
    private val rightMotorSim = rightMotor.simState

    private val rollerSim =
        LinearSystemSim<N1, N1, N1>( // 1 state, 1 input, 1 output
            identifyVelocitySystem(
                ROLLER_KV,
                0.1,
            ),
        )

    init {
        leftMotor.configurator.apply(
            TalonFXConfiguration()
                .withSlot0(
                    Slot0Configs()
                        .withKP(ROLLER_KP)
                        .withKV(ROLLER_KV),
                ),
        )

        rightMotor.configurator.apply(
            TalonFXConfiguration()
                .withSlot0(
                    Slot0Configs()
                        .withKP(ROLLER_KP)
                        .withKV(ROLLER_KV),
                ),
        )
    }

    override fun updateInputs(rollerInputs: RollerIO.RollerIOInputs) {
        val appliedVoltage = leftMotorSim.motorVoltage

        rollerSim.setInput(appliedVoltage)
        rollerSim.update(0.02)

        leftMotorSim.setRotorVelocity(rollerSim.output[0, 0])
        rightMotorSim.setRotorVelocity(rollerSim.output[0, 0])

        rollerInputs.leftMotor.currentVelocity = rollerSim.output[0, 0]
        rollerInputs.leftMotor.voltage = appliedVoltage

        rollerInputs.rightMotor.currentVelocity = rollerSim.output[0, 0]
        rollerInputs.rightMotor.voltage = appliedVoltage
    }

    override fun setVelocity(velocity: Double) {
        leftMotor.setControl(
            VelocityVoltage(velocity)
                .withSlot(0),
        )
        rightMotor.setControl(
            VelocityVoltage(velocity)
                .withSlot(0),
        )
    }

    fun stop() {
        leftMotor.stopMotor()
        rightMotor.stopMotor()
    }
}
