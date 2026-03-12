package frc.team449.subsystems.intake

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants.IntakeConstants
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

class IntakeSubsystem(
    private val io: IntakeIO
) : SubsystemBase() {
    private val inputs: IntakeIOInputsAutoLogged = IntakeIOInputsAutoLogged()

    // boolean over position logging increases speed and is easier to read
    var pivotIsDeployed: Boolean = false
    var rollerTargetVolts: Double = 0.0

    val pivotAngle: Double
        get() = inputs.leftPivotLeaderPositionRad

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)

        Logger.recordOutput("Intake/PivotIsDeployed", pivotIsDeployed)
        Logger.recordOutput("Intake/RollerTargetVolts", rollerTargetVolts)
        Logger.recordOutput("Intake/RollersRunning", (inputs.leftRollerLeaderVelocityRadPerSec > 10.0))
    }

    fun intake(): Command =
        this
            .run {
                rollerTargetVolts = 11.0
                io.setRollerVoltage(11.0)
            }
            .withName("Intake")

    fun outtake(): Command =
        this
            .run {
                rollerTargetVolts = -4.0
                io.setRollerVoltage(-4.0)
            }
            .withName("Outtake")

    fun stopRollers(): Command =
        this
            .runOnce {
                rollerTargetVolts = 0.0
                io.setRollerVoltage(0.0)
            }
            .withName("StopRoller")

    fun deploy(): Command =
        slamHoming(
            true,
            IntakeConstants.DEPLOY_VOLTS,
            IntakeConstants.DEPLOY_HOLD_VOLTS,
        )
            .withName("Deploy")

    fun stow(): Command =
        slamHoming(
            false,
            IntakeConstants.STOW_VOLTS,
            IntakeConstants.STOW_HOLD_VOLTS,
        )
            .withName("Stow")

    fun setSupplyLimits(pivotSupplyLimitAmps: Double, rollerSupplyLimitAmps: Double) {
        io.setSupplyLimits(pivotSupplyLimitAmps, rollerSupplyLimitAmps)
    }

    private fun slamHoming(
        isDeployed: Boolean,
        moveVolts: Double,
        holdVolts: Double
    ): Command =
        this.defer {
            pivotIsDeployed = isDeployed
            val hardstopDebouncer = Debouncer(IntakeConstants.HOMING_DEBOUNCE_TIME)

            this
                .run {
                    io.setPivotVoltage(moveVolts)
                }.until {
                    val highCurrent = abs(inputs.leftPivotLeaderStatorCurrentAmps) > IntakeConstants.HOMING_CURRENT_AMPS
                    val lowVelocity = abs(inputs.leftPivotLeaderVelocityRadPerSec) < IntakeConstants.HOMING_VELOCITY_RAD_PER_SEC
                    hardstopDebouncer.calculate(highCurrent && lowVelocity)
                }.andThen(
                    runOnce {
                        io.setPivotVoltage(holdVolts)
                    },
                )
        }
}
