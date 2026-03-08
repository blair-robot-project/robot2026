package frc.team449.subsystems.intake

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants.IntakeConstants
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

class IntakeSubsystem(
    private val io: IntakeIO
) : SubsystemBase() {
    private val inputs: IntakeIOInputsAutoLogged = IntakeIOInputsAutoLogged() // should not be public

    // boolean over position logging increases speed and is easier to read
    var pivotIsDeployed: Boolean = false
    var rollerTargetVelocityRadPerSec: Double = 0.0

    val intakeSimAngle: Double
        get() = inputs.leftPivotLeaderPositionRad

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)

        Logger.recordOutput("Intake/PivotIsDeployed", pivotIsDeployed)
        Logger.recordOutput("Intake/RollerTargetVelocityRadPerSec", rollerTargetVelocityRadPerSec)
        Logger.recordOutput("Intake/RollersRunning", (inputs.leftRollerLeaderVelocityRadPerSec > 10.0))
    }

    // roller commands
    fun intake(): Command =
        this
            .runOnce {
                rollerTargetVelocityRadPerSec = IntakeConstants.INTAKE_VELOCITY.`in`(RadiansPerSecond)
                io.setRollerVelocity(IntakeConstants.INTAKE_VELOCITY)
            }.withName("Intake")

    fun outtake(): Command =
        this
            .runOnce {
                rollerTargetVelocityRadPerSec = IntakeConstants.OUTTAKE_VELOCITY.`in`(RadiansPerSecond)
                io.setRollerVelocity(IntakeConstants.OUTTAKE_VELOCITY)
            }.withName("Outtake")

    fun stopRollers(): Command =
        this
            .runOnce {
                rollerTargetVelocityRadPerSec = 0.0
                io.setRollerVoltage(0.0)
            }.withName("Stop Rollers")

    // slam commands
    fun deploy(): Command =
        slamHoming(
            true,
            IntakeConstants.DEPLOY_VOLTS,
            IntakeConstants.DEPLOY_HOLD_VOLTS,
        ).withName("Deploy")

    fun repeatedlyDeploy(): Command =
        slamHoming(
            true,
            IntakeConstants.DEPLOY_VOLTS,
            IntakeConstants.DEPLOY_HOLD_VOLTS,
        ).withName("Deploy").withTimeout(1.0).andThen(
            if (abs(inputs.leftPivotLeaderPositionRad - IntakeConstants.DEPLOY_POS_RADS) > 0.12) {
                stow().withTimeout(0.5).andThen(
                    slamHoming(
                        true,
                        IntakeConstants.DEPLOY_VOLTS,
                        IntakeConstants.DEPLOY_HOLD_VOLTS
                    )
                )
            } else {
                Commands.none()
            }
        ).repeatedly()

    fun stow(): Command =
        slamHoming(
            false,
            IntakeConstants.STOW_VOLTS,
            IntakeConstants.STOW_HOLD_VOLTS,
        ).withName("Stow")

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
