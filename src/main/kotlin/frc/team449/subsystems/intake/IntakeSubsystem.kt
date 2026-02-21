package frc.team449.subsystems.intake
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants.IntakeConstants
import frc.team449.Constants.IntakeConstants.DEPLOY_POS_RADS
import frc.team449.Constants.IntakeConstants.INTAKE_VELOCITY
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier
import kotlin.math.abs

class IntakeSubsystem(
    private val io: IntakeIO,
) : SubsystemBase() {
    private val inputs: IntakeIOInputsAutoLogged = IntakeIOInputsAutoLogged()

    var intakeSimAngle: Double = 0.0

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)
    }

    // roller commands
    fun intake(): Command =
        runEnd(
            { io.setRollerVelocity(INTAKE_VELOCITY) },
            { io.setRollerVelocity(RotationsPerSecond.of(0.0)) },
        ).withName("Intake")

    fun outtake(): Command =
        runEnd(
            { io.setRollerVelocity(IntakeConstants.OUTTAKE_VELOCITY) },
            { io.setRollerVelocity(RotationsPerSecond.of(0.0)) },
        ).withName("Outtake")

    fun stopRollers(): Command =
        runOnce {
            io.setRollerVelocity(RotationsPerSecond.of(0.0))
        }.withName("Stop Rollers")

    // slam commands
    fun deploy(): Command =
        slamHoming(
            IntakeConstants.DEPLOY_VOLTS,
            IntakeConstants.DEPLOY_HOLD_VOLTS,
        ).withName("Deploy")

    fun stow(): Command =
        slamHoming(
            IntakeConstants.STOW_VOLTS,
            IntakeConstants.STOW_HOLD_VOLTS,
        ).withName("Stow")

    private fun slamHoming(
        moveVolts: Double,
        holdVolts: Double
    ): Command {
        val hardstopDebouncer =
            Debouncer(
                0.5, // s
            )

        return run {
            io.setPivotVoltage(moveVolts)
        }.until {
            val highCurrent = abs(inputs.leftPivotLeaderStatorCurrentAmps) > IntakeConstants.HOMING_CURRENT_AMPS
            val lowVelocity = abs(inputs.leftPivotLeaderVelocityRadPerSec) > IntakeConstants.HOMING_VELOCITY_RAD_PER_SEC
            hardstopDebouncer.calculate(highCurrent && lowVelocity)
        }.andThen(
            run {
                io.setPivotVoltage(holdVolts)
            },
        )
    }

    fun isIntakeDeployed(): Boolean = abs(DEPLOY_POS_RADS - inputs.leftPivotLeaderPositionRad) <= 0.2

    override fun simulationPeriodic() {
        if (io is IntakeIOSim) {
            io.simulationPeriodic()
            intakeSimAngle = io.pivotSim.angleRads
        }
    }

    fun isSimIntaking(): BooleanSupplier =
        {
            isIntakeDeployed() &&
                    abs(INTAKE_VELOCITY.`in`(RotationsPerSecond) - inputs.leftRollerLeaderVelocityRadPerSec) <= 10.0
        }
}
