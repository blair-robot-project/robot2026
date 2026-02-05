package frc.team449.subsystems.intake
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants.IntakeConstants.PIVOT_INTAKE_VOLTAGE
import frc.team449.Constants.IntakeConstants.PIVOT_STOW_VOLTAGE
import frc.team449.Constants.IntakeConstants.RIGHT_ROLLER_INTAKE_VOLTAGE
import frc.team449.Constants.IntakeConstants.RIGHT_ROLLER_STOW_VOLTAGE
import org.littletonrobotics.junction.Logger

class Intake(
    private val io: IntakeIO
) : SubsystemBase() {
    private val inputs: IntakeIOInputsAutoLogged = IntakeIOInputsAutoLogged()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }
    fun intake(): Command =
        run {
            io.setVoltagePivot(PIVOT_INTAKE_VOLTAGE)
            io.setVoltageRoller(RIGHT_ROLLER_INTAKE_VOLTAGE)
        }

    fun deploy(): Command =
        runOnce {
            io.setVoltagePivot(-PIVOT_STOW_VOLTAGE)
            io.setVoltageRoller(-RIGHT_ROLLER_STOW_VOLTAGE)

        }
    fun stow(): Command =
        runOnce {
            io.setVoltagePivot(-PIVOT_STOW_VOLTAGE)
            io.setVoltageRoller(-RIGHT_ROLLER_STOW_VOLTAGE)

        }
    fun stop(): Command =
        run {
            io.setVoltagePivot(0.0)
            io.setVoltageRoller(0.0)
        }
}
