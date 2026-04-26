package frc.team449.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Constants.IntakeConstants
import frc.team449.subsystems.intake.IntakeSubsystem
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier

class StatorStowCommand(
    private val intake: IntakeSubsystem,
    private val readyToShoot: BooleanSupplier
) : Command() {
    private enum class State { MOVING, PAUSED }

    private val timer = Timer()
    private var currentState = State.MOVING

    init {
        addRequirements(intake)
    }

    override fun initialize() {
        currentState = State.MOVING
        timer.restart()
    }

    override fun execute() {
        if (!readyToShoot.asBoolean) {
            intake.setPivotVoltageInternal(0.0)
            currentState = State.PAUSED
            return
        }

        val leftPivotStalling = intake.leftPivotStatorCurrentAmps > IntakeConstants.HOMING_CURRENT_AMPS
        val rightPivotStalling = intake.rightPivotStatorCurrentAmps > IntakeConstants.HOMING_CURRENT_AMPS

        when (currentState) {
            State.MOVING -> {
                intake.setPivotVoltageInternal(IntakeConstants.SLOW_STOW_VOLTS)
                intake.setRollerVoltageInternal(0.0)

                if (leftPivotStalling || rightPivotStalling) {
                    currentState = State.PAUSED
                    timer.restart()
                }
            }
            State.PAUSED -> {
                intake.setPivotVoltageInternal(0.0)

                if (timer.hasElapsed(IntakeConstants.PAUSE_TIME_SEC)) {
                    currentState = State.MOVING
                    timer.restart()
                }
            }
        }

        Logger.recordOutput("Intake/StatorStowState", currentState.name)
    }

    override fun isFinished(): Boolean = false
}
