package frc.team449.subsystems.shooter

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team449.Constants.ShooterConstants
import frc.team449.util.LoggedTunableNumber
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

class ShooterSubsystem(
    private val io: ShooterIO
) : SubsystemBase() {
    private val inputs: ShooterIOInputsAutoLogged = ShooterIOInputsAutoLogged()

    var flywheelTargetVelocityRadsPerSec: Double = 0.0
        private set
    var hoodTargetAngleRad: Double = 0.0
        private set

    private val tunableFlywheelVelocity = LoggedTunableNumber("Shooter/Tuning/FlywheelTargetRadsPerSec", 0.0)
    private val tunableHoodAngle = LoggedTunableNumber("Shooter/Tuning/HoodTargetRads", 0.0)
    private val tuningModeActive = LoggedTunableNumber("Shooter/Tuning/ModeActive", 0.0) // 1.0 = active

    val hoodAngle: Double
        get() = inputs.hoodAngleRad

    val shooterJamTrigger: Trigger = Trigger { abs(inputs.leftTopLeaderStatorCurrentAmps) > (ShooterConstants.FLYWHEEL_STATOR_LIM - 10.0) }
        .debounce(0.25)

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Shooter", inputs)

        if (tuningModeActive.get() == 1.0) {
            if (tunableFlywheelVelocity.hasChanged(hashCode()) || tunableHoodAngle.hasChanged(hashCode())) {
                flywheelTargetVelocityRadsPerSec = tunableFlywheelVelocity.get()
                hoodTargetAngleRad = tunableHoodAngle.get()

                io.setFlywheelVelocity(RadiansPerSecond.of(flywheelTargetVelocityRadsPerSec))
                io.setHoodAngle(Radians.of(hoodTargetAngleRad))
            }
        }

        Logger.recordOutput("Shooter/FlywheelTargetRadPerSec", flywheelTargetVelocityRadsPerSec)
        Logger.recordOutput("Shooter/HoodTargetRads", hoodTargetAngleRad)
        Logger.recordOutput("Shooter/FlywheelAtTolerance", isFlywheelAtTolerance())
        Logger.recordOutput("Shooter/HoodAtTolerance", isHoodAtTolerance())
        Logger.recordOutput("Shooter/ActiveCommand", currentCommand?.name ?: "None")
    }

    fun setFlywheelVelocity(flywheelVelocity: AngularVelocity): Command =
        runOnce {
            flywheelTargetVelocityRadsPerSec = flywheelVelocity.`in`(RadiansPerSecond)
            io.setFlywheelVelocity(flywheelVelocity)
        }
            .withName("FLYWHEEL-VEL")

    fun stopFlywheel(): Command =
        runOnce {
            flywheelTargetVelocityRadsPerSec = 0.0
            io.setFlywheelVoltage(0.0)
        }
            .withName("FLYWHEEL-STOP")

    fun setHoodAngle(hoodAngle: Angle): Command =
        runOnce {
            hoodTargetAngleRad = hoodAngle.`in`(Radians)
            io.setHoodAngle(hoodAngle)
        }
            .withName("HOOD-ANGLE")

    fun isFlywheelAtTolerance(): Boolean {
        if (flywheelTargetVelocityRadsPerSec < 5.0) return false
        val leftError = abs(inputs.leftTopLeaderVelocityRadsPerSec - flywheelTargetVelocityRadsPerSec)

        return leftError < ShooterConstants.FLYWHEEL_VELOCITY_TOLERANCE_RAD_PER_SEC
    }

    fun isHoodAtTolerance(): Boolean {
        val error = abs(inputs.hoodAngleRad - hoodTargetAngleRad)
        val isAtPos = error < ShooterConstants.HOOD_TOLERANCE_RAD

        return isAtPos
    }

    fun homeHood(): Command =
        this.defer {
            val homingDebouncer = Debouncer(ShooterConstants.HOMING_DEBOUNCE_TIME)

            SequentialCommandGroup(
                run {
                    io.setHoodVoltage(ShooterConstants.HOMING_VOLTAGE)
                }.until {
                    val highCurrent = abs(inputs.hoodStatorCurrentAmps) > ShooterConstants.HOMING_CURRENT_AMPS
                    val lowVelocity = abs(inputs.hoodVelocityRadPerSec) < ShooterConstants.HOMING_VELOCITY_RAD_PER_SEC
                    homingDebouncer.calculate(highCurrent && lowVelocity)
                },
                runOnce {
                    io.setHoodVoltage(0.0)
                    io.resetHoodAngle(ShooterConstants.MIN_HOOD_ANGLE)
                }
            )
        }
            .withName("HOOD-HOME")

    val sysIDFlywheel =
        SysIdRoutine(
            SysIdRoutine.Config(
                null,
                Volts.of(8.0),
                Seconds.of(20.0),
            ) { state: SysIdRoutineLog.State ->
                Logger.recordOutput(
                    "SysIdFlywheel",
                    state.toString(),
                )
            },
            Mechanism(
                { voltage: Voltage -> io.setFlywheelVoltage(voltage.`in`(Volts)) },
                null,
                this,
            ),
        )

    val sysIDHood =
        SysIdRoutine(
            SysIdRoutine.Config(
                null,
                Volts.of(6.0),
                Seconds.of(20.0),
            ) { state: SysIdRoutineLog.State ->
                Logger.recordOutput(
                    "SysIdHood",
                    state.toString(),
                )
            },
            Mechanism(
                { voltage: Voltage -> io.setHoodVoltage(voltage.`in`(Volts)) },
                null,
                this,
            ),
        )
}
