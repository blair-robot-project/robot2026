package frc.team449.subsystems.intake
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants.IntakeConstants.LEFT_ROLLER_INTAKE_VOLTAGE
import frc.team449.Constants.IntakeConstants.LEFT_ROLLER_STOW_VOLTAGE
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
            io.setVoltage(
                PIVOT_INTAKE_VOLTAGE,
                LEFT_ROLLER_INTAKE_VOLTAGE,
                RIGHT_ROLLER_INTAKE_VOLTAGE,
            )
        }

    fun stow(): Command =
        run {
            io.setVoltage(
                PIVOT_STOW_VOLTAGE,
                LEFT_ROLLER_STOW_VOLTAGE,
                RIGHT_ROLLER_STOW_VOLTAGE,
            )
        }
}
