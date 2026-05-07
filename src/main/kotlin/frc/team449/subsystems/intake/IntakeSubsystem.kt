package frc.team449.subsystems.intake

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team449.Constants.IntakeConstants
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

class IntakeSubsystem(
    private val io: IntakeIO
) : SubsystemBase() {
    private val inputs: IntakeIOInputsAutoLogged = IntakeIOInputsAutoLogged()

    @AutoLogOutput(key = "Intake/PivotIsDeployed")
    var pivotIsDeployed: Boolean = false
        get() = determinePivotDeployedState()
        private set

    @AutoLogOutput(key = "Intake/PivotTargetRads")
    var pivotTargetAngleRads: Double = 0.0
        private set

    @AutoLogOutput(key = "Intake/RollerTargetVolts")
    var rollerTargetVolts: Double = 0.0
        private set

    val pivotAngle: Double
        get() = inputs.leftPivotPositionRads

    val leftPivotStatorCurrentAmps: Double
        get() = inputs.leftPivotStatorCurrentAmps

    val rightPivotStatorCurrentAmps: Double
        get() = inputs.rightPivotStatorCurrentAmps

    val leftPivotDisconnectedAlert =
        Alert("Left Pivot Disconnected (ID ${IntakeConstants.LEFT_PIVOT_ID}).", Alert.AlertType.kError)
    val rightPivotDisconnectedAlert =
        Alert("Right Pivot Disconnected (ID ${IntakeConstants.RIGHT_PIVOT_ID}).", Alert.AlertType.kError)
    val leftRollerLeaderDisconnectedAlert =
        Alert("Left Roller Disconnected (ID ${IntakeConstants.LEFT_ROLLER_LEADER_ID}).", Alert.AlertType.kError)
    val rightRollerFollowerDisconnectedAlert =
        Alert("Right Roller Disconnected (ID ${IntakeConstants.RIGHT_ROLLER_FOLLOWER_ID}).", Alert.AlertType.kError)

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Intake", inputs)

        leftPivotDisconnectedAlert.set(!inputs.leftPivotConnected)
        rightPivotDisconnectedAlert.set(!inputs.rightPivotConnected)
        leftRollerLeaderDisconnectedAlert.set(!inputs.leftRollerLeaderConnected)
        rightRollerFollowerDisconnectedAlert.set(!inputs.rightRollerFollowerConnected)

        Logger.recordOutput("Intake/RollersRunning", (inputs.leftRollerLeaderVelocityRadsPerSec > 10.0))
        Logger.recordOutput("Intake/ActiveCommand", currentCommand?.name ?: "None")
    }

    fun setRollerVoltageInternal(rollerVolts: Double) {
        rollerTargetVolts = rollerVolts
        io.setRollerVoltage(rollerTargetVolts)
    }

    fun setRollerVoltage(rollerVolts: Double): Command =
        runOnce {
            setRollerVoltageInternal(rollerVolts)
        }

    fun stopRollers(): Command =
        runOnce {
            rollerTargetVolts = 0.0
            io.setRollerVoltage(0.0)
        }

    fun setPivotAngleInternal(pivotAngle: Angle) {
        pivotTargetAngleRads = pivotAngle.`in`(Radians)
        io.setPivotAngle(pivotAngle)
    }

    fun setPivotAngle(pivotAngle: Angle): Command =
        runOnce {
            setPivotAngleInternal(pivotAngle)
        }

    fun setPivotVoltageInternal(pivotVolts: Double) =
        io.setPivotVoltage(pivotVolts)

    fun setPivotVoltage(pivotVolts: Double): Command =
        runOnce {
            setPivotVoltageInternal(pivotVolts)
        }

    fun deploy(): Command = slamHoming(
        IntakeConstants.DEPLOY_VOLTS,
        targetIsDeployed = true
    )

    fun stow(): Command = slamHoming(
        IntakeConstants.STOW_VOLTS,
        targetIsDeployed = false
    )

    fun stowSlow(): Command = slamHoming(
        IntakeConstants.SLOW_STOW_VOLTS,
        targetIsDeployed = false
    )

    private fun slamHoming(
        moveVolts: Double,
        targetIsDeployed: Boolean
    ): Command =
        defer {
            val hardstopDebouncer = Debouncer(IntakeConstants.HOMING_DEBOUNCE_TIME)
            val pivotAngleRads: Double = if (targetIsDeployed) IntakeConstants.DEPLOY_POS_RADS else IntakeConstants.STOW_POS_RADS

            run {
                io.setPivotVoltage(moveVolts)
            }.until {
                val highCurrent = abs(inputs.leftPivotStatorCurrentAmps) > IntakeConstants.HOMING_CURRENT_AMPS
                val lowVelocity = abs(inputs.leftPivotVelocityRadsPerSec) < IntakeConstants.HOMING_VELOCITY_RADS_PER_SEC
                hardstopDebouncer.calculate(highCurrent && lowVelocity)
            }.andThen(
                runOnce {
                    io.resetPivotAngle(Radians.of(pivotAngleRads))
                },
            )
        }

    private fun determinePivotDeployedState(): Boolean =
        abs(inputs.leftPivotPositionRads - IntakeConstants.DEPLOY_POS_RADS) < 0.1 &&
            abs(inputs.rightPivotPositionRads - IntakeConstants.DEPLOY_POS_RADS) < 0.1

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
