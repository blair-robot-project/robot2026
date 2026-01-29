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
    private val motor = TalonFX(41)
    private val motorSim = motor.simState

    private val rollerSim =
        LinearSystemSim<N1, N1, N1>(
            identifyVelocitySystem(
                ROLLER_KV,
                0.1,
            ),
        )

    init {
        motor.configurator.apply(
            TalonFXConfiguration()
                .withSlot0(
                    Slot0Configs()
                        .withKP(ROLLER_KP)
                        .withKV(ROLLER_KV),
                ),
        )
    }

    override fun updateInputs(rollerInputs: RollerIO.RollerIOInputs) {
        val appliedVoltage = motorSim.motorVoltage

        rollerSim.setInput(appliedVoltage)
        rollerSim.update(0.02)

        motorSim.setRotorVelocity(rollerSim.output[0, 0])

        rollerInputs.currentVelocity = rollerSim.output[0, 0]
        rollerInputs.voltage = appliedVoltage
    }

    override fun setVelocity(velocity: Double) { // rps
        motor.setControl(VelocityVoltage(velocity).withSlot(0))
    }

    fun stop() {
        motor.stopMotor()
    }
}
