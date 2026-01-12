package frc.team449.subsystems.turret

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class TurretSubsystem(
    val io: TurretIO
): SubsystemBase() {
    private val inputs: TurretIOInputsAutoLogged = TurretIOInputsAutoLogged()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Turret", inputs)
    }

    fun setPosition(position: Angle): Command {
        return Commands.runOnce({
            io.requestPosition(position)
        })
    }
    fun setVoltage(voltage: Voltage): Command{
        return Commands.runOnce({
            io.requestVoltage(voltage)
        })
    }
    fun stop(): Command {
        return Commands.runOnce({
            io.requestVoltage(Volts.of(0.0))
        })
    }
}