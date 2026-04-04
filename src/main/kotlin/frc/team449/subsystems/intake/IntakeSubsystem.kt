package frc.team449.subsystems.intake

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team449.Constants.IntakeConstants
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

class IntakeSubsystem(
    private val io: IntakeIO
) : SubsystemBase() {
    private val inputs: IntakeIOInputsAutoLogged = IntakeIOInputsAutoLogged()

    var pivotIsDeployed: Boolean = false
        private set
    var pivotTargetAngleRads: Double = 0.0
        private set
    var rollerTargetVolts: Double = 0.0
        private set

    val pivotAngle: Double
        get() = inputs.leftPivotPositionRads

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)

        pivotIsDeployed = determinePivotDeployedState()
        Logger.recordOutput("Intake/PivotIsDeployed", pivotIsDeployed)
        Logger.recordOutput("Intake/RollerTargetVolts", rollerTargetVolts)
        Logger.recordOutput("Intake/RollersRunning", (inputs.leftRollerLeaderVelocityRadsPerSec > 10.0))
        Logger.recordOutput("Intake/ActiveCommand", currentCommand?.name ?: "None")
    }

    fun setRollerVoltage(rollerVolts: Double): Command =
        runOnce {
            rollerTargetVolts = rollerVolts
            io.setRollerVoltage(rollerTargetVolts)
        }
            .withName("ROLLER-VOLTS")

    fun stopRollers(): Command =
        runOnce {
            rollerTargetVolts = 0.0
            io.setRollerVoltage(0.0)
        }
            .withName("ROLLER-STOP")

    fun setPivotAngle(pivotAngle: Angle): Command =
        runOnce {
            pivotTargetAngleRads = pivotAngle.`in`(Radians)
            io.setPivotAngle(pivotAngle)
        }
            .withName("PIVOT-ANGLE")

    fun setPivotVoltage(pivotVolts: Double): Command =
        runOnce {
            io.setPivotVoltage(pivotVolts)
        }
            .withName("PIVOT-VOLTS")

    fun deploy(): Command = slamHoming(targetIsDeployed = true).withName("PIVOT-DEPLOY")
    fun stow(): Command = slamHoming(targetIsDeployed = false).withName("PIVOT-STOW")

    private fun slamHoming(
        targetIsDeployed: Boolean
    ): Command =
        defer {
            val hardstopDebouncer = Debouncer(IntakeConstants.HOMING_DEBOUNCE_TIME)
            val moveVolts = if (targetIsDeployed) IntakeConstants.DEPLOY_VOLTS else IntakeConstants.STOW_VOLTS
            val holdVolts = if (targetIsDeployed) IntakeConstants.DEPLOY_HOLD_VOLTS else IntakeConstants.STOW_HOLD_VOLTS
            val pivotAngleRads: Double = if (targetIsDeployed) IntakeConstants.DEPLOY_POS_RADS else IntakeConstants.STOW_POS_RADS

            run {
                io.setPivotVoltage(moveVolts)
            }
                .until {
                    val highCurrent = abs(inputs.leftPivotStatorCurrentAmps) > IntakeConstants.HOMING_CURRENT_AMPS
                    val lowVelocity = abs(inputs.leftPivotVelocityRadsPerSec) < IntakeConstants.HOMING_VELOCITY_RADS_PER_SEC
                    hardstopDebouncer.calculate(highCurrent && lowVelocity)
                }
                .andThen(
                    runOnce {
                        io.resetPivotAngle(Radians.of(pivotAngleRads))
                        io.setPivotVoltage(holdVolts)
                    }
                )
        }

    private fun determinePivotDeployedState(): Boolean {
        return abs(inputs.leftPivotPositionRads - IntakeConstants.DEPLOY_POS_RADS) < 0.1 && abs(inputs.rightPivotPositionRads - IntakeConstants.DEPLOY_POS_RADS) < 0.1
    }

    // Checks Left Pivot Leader to see if it's in a tolerable range
    fun pivotAtTolerance(): Boolean {
        val target: Double =
            if (pivotIsDeployed) {
                IntakeConstants.DEPLOY_POS_RADS
            } else {
                IntakeConstants.STOW_POS_RADS
            }
        return abs(pivotAngle - target) <=
            IntakeConstants.PIVOT_TOLERANCE_POS_RADS
    }

    // Checks the rollerVelocity is in a tolerable range
    fun rollerAtTolerance(): Boolean =
        abs(inputs.leftRollerLeaderAppliedVolts - rollerTargetVolts) <= 1.0 &&
            abs(inputs.rightRollerFollowerAppliedVolts - rollerTargetVolts) <= 1.0

    val sysIDPivot =
        SysIdRoutine(
            SysIdRoutine.Config(
                null,
                Volts.of(6.0),
                Seconds.of(20.0),
            ) { state: SysIdRoutineLog.State ->
                Logger.recordOutput(
                    "SysIdPivot",
                    state.toString(),
                )
            },
            Mechanism(
                { voltage: Voltage -> io.setPivotVoltage(voltage.`in`(Volts)) },
                null,
                this,
            ),
        )
}
