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

    var flywheelTargetVelocityRadPerSec: Double = 0.0
    private var hoodTargetAngleRad: Double = 0.0

    private val tunableFlywheelVelocity = LoggedTunableNumber("Shooter/Tuning/FlywheelTargetRadPerSec", 0.0)
    private val tunableHoodAngle = LoggedTunableNumber("Shooter/Tuning/HoodTargetRads", 0.0)
    private val tuningModeActive = LoggedTunableNumber("Shooter/Tuning/ModeActive", 0.0) // 1.0 = active

    val hoodAngle: Double
        get() = inputs.hoodAngleRad

    private val hoodDebouncer: Debouncer = Debouncer(ShooterConstants.TOLERANCE_DEBOUNCE_TIME)

    val shooterJamTrigger: Trigger = Trigger { abs(inputs.leftLeaderStatorCurrentAmps) > (ShooterConstants.FLYWHEEL_STATOR_LIM - 10.0) || abs(inputs.rightLeaderStatorCurrentAmps) > (ShooterConstants.FLYWHEEL_STATOR_LIM - 10.0) }
        .debounce(0.25)

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Shooter", inputs)

        if (tuningModeActive.get() == 1.0) {
            if (tunableFlywheelVelocity.hasChanged(hashCode()) || tunableHoodAngle.hasChanged(hashCode())) {
                flywheelTargetVelocityRadPerSec = tunableFlywheelVelocity.get()
                hoodTargetAngleRad = tunableHoodAngle.get()

                io.setFlywheelVelocity(RadiansPerSecond.of(flywheelTargetVelocityRadPerSec))
                io.setHoodAngle(Radians.of(hoodTargetAngleRad))
            }
        }

        Logger.recordOutput("Shooter/FlywheelTargetRadPerSec", flywheelTargetVelocityRadPerSec)
        Logger.recordOutput("Shooter/HoodTargetRads", hoodTargetAngleRad)

        Logger.recordOutput("Shooter/FlywheelAtTolerance", isFlywheelAtTolerance())
        Logger.recordOutput("Shooter/HoodAtTolerance", isHoodAtTolerance())
    }

    fun setAimCommand(hoodAngle: Angle, flywheelVelocity: AngularVelocity): Command =
        run {
            hoodTargetAngleRad = hoodAngle.`in`(Radians)
            flywheelTargetVelocityRadPerSec = flywheelVelocity.`in`(RadiansPerSecond)

            io.setHoodAngle(hoodAngle)
            io.setFlywheelVelocity(flywheelVelocity)
        }

    fun setFlywheelVelocity(velocity: AngularVelocity): Command =
        run {
            flywheelTargetVelocityRadPerSec = velocity.`in`(RadiansPerSecond)
            io.setFlywheelVelocity(velocity)
        }

    fun stopFlywheel(): Command =
        run {
            flywheelTargetVelocityRadPerSec = 0.0
            io.setFlywheelVoltage(0.0)
        }

    fun setHoodVoltage(volts: Double): Command =
        run {
            io.setHoodVoltage(volts)
        }

    fun setHoodAngle(angle: Angle): Command =
        run {
            hoodTargetAngleRad = angle.`in`(Radians)
            io.setHoodAngle(angle)
        }

    fun resetHoodAngle(angle: Angle): Command =
        runOnce {
            io.resetHoodAngle(angle)
        }

    fun isFlywheelAtTolerance(): Boolean {
        // if (target == 0.0) return false
        val leftError = abs(inputs.leftLeaderVelocityRadPerSec - flywheelTargetVelocityRadPerSec)
        val rightError = abs(inputs.rightLeaderVelocityRadPerSec - flywheelTargetVelocityRadPerSec)

        val isAtSpeed =
            leftError < ShooterConstants.FLYWHEEL_VELOCITY_TOLERANCE_RAD_PER_SEC &&
                rightError < ShooterConstants.FLYWHEEL_VELOCITY_TOLERANCE_RAD_PER_SEC

        return isAtSpeed
    }

    fun isHoodAtTolerance(): Boolean {
        val error = abs(inputs.hoodAngleRad - hoodTargetAngleRad)
        val isAtPos = error < ShooterConstants.HOOD_TOLERANCE_RAD

        return hoodDebouncer.calculate(isAtPos)
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
            .withName("Home Hood")

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
