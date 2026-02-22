package frc.team449.subsystems.intake
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants.IntakeConstants
import frc.team449.Constants.IntakeConstants.DEPLOY_POS_RADS
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier
import java.util.function.IntSupplier
import kotlin.math.abs

class IntakeSubsystem(
    private val io: IntakeIO
) : SubsystemBase() {
    private val inputs: IntakeIOInputsAutoLogged = IntakeIOInputsAutoLogged()

    // boolean over position logging increases speed and is easier to read
    private var pivotDeployedState: Boolean = false
    private var rollerTargetVelocityRadPerSec: Double = 0.0

    val intakeSimAngle: Double
        get() = inputs.leftPivotLeaderPositionRad

    private var simIntakeThrottle = 0

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)

        Logger.recordOutput("Intake/PivotDeployedState", pivotDeployedState)
        Logger.recordOutput("Intake/RollerTargetVelocityRadPerSec", rollerTargetVelocityRadPerSec)
    }

    // roller commands
    fun intake(): Command =
        this.runOnce {
            rollerTargetVelocityRadPerSec = IntakeConstants.INTAKE_VELOCITY.`in`(RadiansPerSecond)
            io.setRollerVelocity(IntakeConstants.INTAKE_VELOCITY)
        }
            .withName("Intake")

    fun outtake(): Command =
        this.runOnce {
            rollerTargetVelocityRadPerSec = IntakeConstants.OUTTAKE_VELOCITY.`in`(RadiansPerSecond)
            io.setRollerVelocity(IntakeConstants.OUTTAKE_VELOCITY)
        }
            .withName("Outtake")

    fun stopRollers(): Command =
        this.runOnce {
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

    fun stow(): Command =
        slamHoming(
            false,
            IntakeConstants.STOW_VOLTS,
            IntakeConstants.STOW_HOLD_VOLTS,
        ).withName("Stow")

    private fun slamHoming(
        deployedState: Boolean,
        moveVolts: Double,
        holdVolts: Double
    ): Command {
        return this.defer {
            pivotDeployedState = deployedState
            val hardstopDebouncer = Debouncer(0.25)

            this.run {
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

    fun isIntakeDeployed(): Boolean = abs(DEPLOY_POS_RADS - inputs.leftPivotLeaderPositionRad) <= 0.2

    fun isSimIntaking(ballCount: IntSupplier): BooleanSupplier =
        {
            if (simIntakeThrottle < IntakeConstants.BPS_RATE_LIMIT && Math.random() > IntakeConstants.SIMULATED_BALL_INTAKING_MISS_CHANCE) {
                simIntakeThrottle += 1
            }
            simIntakeThrottle >= IntakeConstants.BPS_RATE_LIMIT && isIntakeDeployed() && ballCount.asInt < IntakeConstants.SIMULATED_BALL_INTAKE_LIMIT
            //                && abs(IntakeConstants.INTAKE_VELOCITY.`in`(RotationsPerSecond) - inputs.leftRollerLeaderVelocityRadPerSec) <= 25.0
        }
}
