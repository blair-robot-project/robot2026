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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import frc.team449.Constants.ShooterConstants
import frc.team449.util.LoggedTunableNumber
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier
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

    private val tunableLeftFlywheelKP = LoggedTunableNumber("Shooter/Tuning/LeftKP", ShooterConstants.LEFT_FLYWHEEL_KP)
    private val tunableLeftFlywheelKD = LoggedTunableNumber("Shooter/Tuning/LeftKD", ShooterConstants.LEFT_FLYWHEEL_KD)
    private val tunableRightFlywheelKP = LoggedTunableNumber("Shooter/Tuning/RightKP", ShooterConstants.RIGHT_FLYWHEEL_KP)
    private val tunableRightFlywheelKD = LoggedTunableNumber("Shooter/Tuning/RightKD", ShooterConstants.RIGHT_FLYWHEEL_KD)

    private val tunableHoodKP = LoggedTunableNumber("Shooter/Tuning/HoodKP", ShooterConstants.HOOD_KP)
    private val tunableHoodKD = LoggedTunableNumber("Shooter/Tuning/HoodKD", ShooterConstants.HOOD_KD)

    val hoodSimAngle: Double
        get() = inputs.hoodAngleRad

    private val flywheelDebouncer: Debouncer = Debouncer(ShooterConstants.TOLERANCE_DEBOUNCE_TIME)
    private val hoodDebouncer: Debouncer = Debouncer(ShooterConstants.TOLERANCE_DEBOUNCE_TIME)

    val shooterJamTrigger = Trigger { abs(inputs.leftLeaderStatorCurrentAmps) > (ShooterConstants.FLYWHEEL_STATOR_LIM - 10.0) || abs(inputs.rightLeaderStatorCurrentAmps) > (ShooterConstants.FLYWHEEL_STATOR_LIM - 10.0) }
        .debounce(0.5)

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Shooter", inputs)

        if (tuningModeActive.get() == 1.0) {
            updateTunableNumbers()

            flywheelTargetVelocityRadPerSec = tunableFlywheelVelocity.get()
            hoodTargetAngleRad = tunableHoodAngle.get()

            setFlywheelVelocity(RadiansPerSecond.of(flywheelTargetVelocityRadPerSec))
            setHoodAngle(Radians.of(hoodTargetAngleRad))
        }

        Logger.recordOutput("Shooter/FlywheelTargetRadPerSec", flywheelTargetVelocityRadPerSec)
        Logger.recordOutput("Shooter/HoodTargetRads", hoodTargetAngleRad)

        Logger.recordOutput("Shooter/FlywheelAtTolerance", isFlywheelAtTolerance())
        Logger.recordOutput("Shooter/HoodAtTolerance", isHoodAtTolerance())
    }

    fun updateTunableNumbers() {
        if (tunableLeftFlywheelKP.hasChanged(hashCode()) ||
            tunableLeftFlywheelKD.hasChanged(hashCode()) ||
            tunableRightFlywheelKP.hasChanged(hashCode()) ||
            tunableRightFlywheelKD.hasChanged(hashCode())
        ) {
            io.setFlywheelGains(tunableLeftFlywheelKP.get(), tunableLeftFlywheelKD.get(), tunableRightFlywheelKP.get(), tunableRightFlywheelKD.get())
        }

        if (tunableHoodKP.hasChanged(hashCode()) || tunableHoodKD.hasChanged(hashCode())) {
            io.setHoodGains(tunableHoodKP.get(), tunableHoodKD.get())
        }
    }

    fun setFlywheelAndHoodFromSuppliers(
        flywheelVelocitySupplier: Supplier<AngularVelocity>,
        hoodAngleSupplier: Supplier<Angle>
    ): Command =
        runOnce {
            val flywheelVelocity = flywheelVelocitySupplier.get()
            val hoodAngle = hoodAngleSupplier.get()

            flywheelTargetVelocityRadPerSec = flywheelVelocity.`in`(RadiansPerSecond)
            hoodTargetAngleRad = hoodAngle.`in`(Radians)

            setFlywheelVelocity(flywheelVelocity)
            setHoodAngle(hoodAngle)
        }

    fun setFlywheelVelocity(velocity: AngularVelocity): Command =
        runOnce {
            flywheelTargetVelocityRadPerSec = velocity.`in`(RadiansPerSecond)
            io.setFlywheelVelocity(velocity)
        }

    fun stopFlywheel(): Command =
        runOnce {
            flywheelTargetVelocityRadPerSec = 0.0
            io.setFlywheelVoltage(0.0)
        }

    fun setHoodVoltage(volts: Double): Command =
        runOnce {
            io.setHoodVoltage(volts)
        }

    fun setHoodAngle(angle: Angle): Command =
        runOnce {
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

        return flywheelDebouncer.calculate(isAtSpeed)
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
                setHoodVoltage(ShooterConstants.HOMING_VOLTAGE),
                WaitUntilCommand {
                    val highCurrent = abs(inputs.hoodStatorCurrentAmps) > ShooterConstants.HOMING_CURRENT_AMPS
                    val lowVelocity = abs(inputs.hoodVelocityRadPerSec) < ShooterConstants.HOMING_VELOCITY_RAD_PER_SEC
                    homingDebouncer.calculate(highCurrent && lowVelocity)
                },
                setHoodVoltage(0.0),
                resetHoodAngle(ShooterConstants.MIN_HOOD_ANGLE),
                setHoodAngle(ShooterConstants.MIN_HOOD_ANGLE)
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
}
