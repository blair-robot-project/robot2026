package frc.team449.subsystems.intake
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants
import frc.team449.Constants.IntakeConstants
import org.littletonrobotics.junction.Logger
import kotlin.concurrent.timer

class Intake(
    private val io: IntakeIO
) : SubsystemBase() {
    private val inputs: IntakeIOInputsAutoLogged = IntakeIOInputsAutoLogged()
    private val currentHomingTimer = Timer()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }

    fun intake(): Command =
        runOnce {io.setRollerRequest(VoltageOut(IntakeConstants.INTAKE_VOLTAGE))}
    fun stopIntake(): Command =
        runOnce {io.setRollerRequest(VoltageOut(0.0))}
    fun outtake(): Command =
        runOnce {io.setRollerRequest(VoltageOut(IntakeConstants.OUTTAKE_VOLTAGE))}
    fun deploy(): Command =
        Commands.sequence(
            runOnce {
                io.setPivotRequest(VoltageOut(IntakeConstants.DEPLOY_VOLTAGE))
                currentHomingTimer.restart() },
            Commands.waitUntil {
                inputs.pivotStatorCurrent > IntakeConstants.CURRENT_HOMING_CURRENT_LIMIT &&
                        currentHomingTimer.get() > IntakeConstants.CURRENT_HOMING_TIME_LIMIT.`in`(Seconds) &&
                        inputs.pivotSpeed > IntakeConstants.CURRENT_HOMING_VEL_LIMIT
            },
            runOnce {
                io.setPivotRequest(VoltageOut(0.0))
                currentHomingTimer.stop()
            }
        )
    fun stow(): Command =
        Commands.sequence(
            runOnce {
                io.setPivotRequest(VoltageOut(IntakeConstants.STOW_VOLTAGE))
                currentHomingTimer.restart() },
            Commands.waitUntil {
                inputs.pivotStatorCurrent > IntakeConstants.CURRENT_HOMING_CURRENT_LIMIT &&
                        currentHomingTimer.get() > IntakeConstants.CURRENT_HOMING_TIME_LIMIT.`in`(Seconds) &&
                        inputs.pivotSpeed > IntakeConstants.CURRENT_HOMING_VEL_LIMIT
            },
            runOnce {
                io.setPivotRequest(VoltageOut(0.0))
                currentHomingTimer.stop()
            }
        )
    fun stopPivot(): Command =
        runOnce {
            io.setPivotRequest(VoltageOut(0.0))
        }
}
