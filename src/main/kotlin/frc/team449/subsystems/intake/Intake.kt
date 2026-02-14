package frc.team449.subsystems.intake
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.team449.Constants.IntakeConstants.CURRENT_HOMING_CURRENT_LIMIT
import frc.team449.Constants.IntakeConstants.CURRENT_HOMING_VEL_LIMIT
import frc.team449.Constants.IntakeConstants.DEPLOY_POSITION
import frc.team449.Constants.IntakeConstants.DEPLOY_VOLTAGE
import frc.team449.Constants.IntakeConstants.HOMING_DEBOUNCE_TIME
import frc.team449.Constants.IntakeConstants.HOMING_DEBOUNCE_TYPE
import frc.team449.Constants.IntakeConstants.HOMING_TIME_OUT
import frc.team449.Constants.IntakeConstants.INTAKE_VOLTAGE
import frc.team449.Constants.IntakeConstants.OUTTAKE_VOLTAGE
import frc.team449.Constants.IntakeConstants.STOW_POSITION
import frc.team449.Constants.IntakeConstants.STOW_VOLTAGE
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

class Intake(
    private val io: IntakeIO
) : SubsystemBase() {
    private val inputs: IntakeIOInputsAutoLogged = IntakeIOInputsAutoLogged()

    init {
        io.setPivotPosition(STOW_POSITION)
    }

    val currentHomingDebouncer = Debouncer(HOMING_DEBOUNCE_TIME, HOMING_DEBOUNCE_TYPE)

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
                VoltageOut(INTAKE_VOLTAGE),
            )
        }

    fun stopIntake(): Command =
        runOnce {
            io.setRollerRequest(VoltageOut(0.0))
        }

    fun outtake(): Command =
        runOnce {
            io.setRollerRequest(
                VoltageOut(OUTTAKE_VOLTAGE),
            )
        }

    fun deploy(): Command =
        Commands.sequence(
            runOnce {
                currentHomingDebouncer.calculate(false)
                io.setPivotRequest(VoltageOut(DEPLOY_VOLTAGE))
            },
            WaitUntilCommand {
                currentHomingDebouncer.calculate(
                    inputs.pivotMotorStatorCurrent >
                        CURRENT_HOMING_CURRENT_LIMIT,
                ) &&
                    abs(inputs.pivotMotorVelocity.`in`(RadiansPerSecond)) <
                        CURRENT_HOMING_VEL_LIMIT.`in`(RadiansPerSecond)
            }.withTimeout(HOMING_TIME_OUT),
            runOnce {
                io.setPivotRequest(VoltageOut(0.0))
                io.resetPivotPosition(DEPLOY_POSITION)
            },
            holdPivot(),
        ).withName("Deploy")

    fun stow(): Command =
        Commands.sequence(
            runOnce {
                currentHomingDebouncer.calculate(false)
                io.setPivotRequest(VoltageOut(STOW_VOLTAGE))
            },
            WaitUntilCommand {
                currentHomingDebouncer.calculate(
                    inputs.pivotMotorStatorCurrent >
                        CURRENT_HOMING_CURRENT_LIMIT,
                ) &&
                    abs(inputs.pivotMotorVelocity.`in`(RadiansPerSecond)) <
                        CURRENT_HOMING_VEL_LIMIT.`in`(RadiansPerSecond)
            }.withTimeout(HOMING_TIME_OUT),
            runOnce {
                io.setPivotRequest(VoltageOut(0.0))
                io.resetPivotPosition(STOW_POSITION)
            },
            holdPivot(),
        ).withName("Stow")

    fun holdPivot(): Command =
        runOnce {
            io.setPivotRequest(
                VoltageOut(0.0)
            )
        }.withName("Hold")
}
