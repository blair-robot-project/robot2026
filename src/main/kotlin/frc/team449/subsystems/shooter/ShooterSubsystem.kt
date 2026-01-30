package frc.team449.subsystems.shooter

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.inputs.LoggableInputs


class ShooterSubsystem(
    private val io: ShooterIO
) : SubsystemBase() {
    private val inputs: ShooterIO.ShooterIOInputs = ShooterIO.ShooterIOInputs()
    private var targetPos = 0.0

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Climb", inputs as LoggableInputs)
        Logger.recordOutput("Climb/targetPos", targetPos)
    }

    fun shoot() {
        io.run(Constants.ShooterConstants.SHOOTER_VOLTAGE)
    }

    fun stop() {
        io.stop()
    }
}