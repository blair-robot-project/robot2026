package frc.team449.subsystems.intake
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Constants.IntakeConstants
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

class IntakeSubsystem(
    private val io: IntakeIO
) : SubsystemBase() {
    private val inputs: IntakeIOInputsAutoLogged = IntakeIOInputsAutoLogged()

    init {
        io.setPivotPosition(IntakeConstants.STOW_POSITION)
    }

    val currentHomingDebouncer = Debouncer(IntakeConstants.HOMING_DEBOUNCE_TIME, IntakeConstants.HOMING_DEBOUNCE_TYPE)
    val request = VoltageOut(0.0)
        .withEnableFOC(false)

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)

        // Hacky solution for now, replace if you find a better one.
        Logger.recordOutput("Intake/Current command", currentCommand?.name ?: "None")
    }

    override fun simulationPeriodic() {
        io.simulationPeriodic()
    }

    fun intake(): Command =
        runOnce {
            io.setRollerRequest(
                request.withOutput(IntakeConstants.INTAKE_VOLTAGE),
            )
        }

    fun stopIntake(): Command =
        runOnce {
            io.setRollerRequest(VoltageOut(0.0))
        }

    fun outtake(): Command =
        runOnce {
            io.setRollerRequest(
                request.withOutput(IntakeConstants.OUTTAKE_VOLTAGE),
            )
        }

    fun deploy(): Command =
        Commands.sequence(
            runOnce {
                currentHomingDebouncer.calculate(false)
                io.setPivotRequest(request.withOutput(IntakeConstants.DEPLOY_VOLTAGE))
            },
            WaitUntilCommand {
                currentHomingDebouncer.calculate(
                    inputs.pivotMotorStatorCurrent >
                        IntakeConstants.CURRENT_HOMING_CURRENT_LIMIT,
                ) &&
                    abs(inputs.pivotMotorVelocity.`in`(RadiansPerSecond)) <
                        IntakeConstants.CURRENT_HOMING_VEL_LIMIT.`in`(RadiansPerSecond)
            }.withTimeout(IntakeConstants.HOMING_TIME_OUT),
            runOnce {
                io.setPivotPosition(IntakeConstants.DEPLOY_POSITION)
                io.setPivotRequest(request.withOutput(IntakeConstants.DEPLOY_HOLD_VOLTAGE))
            }
        ).withName("Deploy")

    fun stow(): Command =
        Commands.sequence(
            runOnce {
                currentHomingDebouncer.calculate(false)
                io.setPivotRequest(request.withOutput(IntakeConstants.STOW_VOLTAGE))
            },
            WaitUntilCommand {
                currentHomingDebouncer.calculate(
                    inputs.pivotMotorStatorCurrent >
                        IntakeConstants.CURRENT_HOMING_CURRENT_LIMIT,
                ) &&
                    abs(inputs.pivotMotorVelocity.`in`(RadiansPerSecond)) <
                        IntakeConstants.CURRENT_HOMING_VEL_LIMIT.`in`(RadiansPerSecond)
            }.withTimeout(IntakeConstants.HOMING_TIME_OUT),
            runOnce {
                io.setPivotPosition(IntakeConstants.STOW_POSITION)
                io.setPivotRequest(request.withOutput(IntakeConstants.STOW_HOLD_VOLTAGE))
            }
        ).withName("Stow")
}
