package frc.team449.subsystems.shooter
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import org.littletonrobotics.junction.AutoLog

interface ShooterIO {
    @AutoLog
    open class ShooterIOInputs {
        @JvmField var leftLeaderVelocityRadPerSec: Double = 0.0

        @JvmField var leftLeaderAppliedVolts: Double = 0.0

        @JvmField var leftLeaderSupplyCurrentAmps: Double = 0.0

        @JvmField var leftLeaderStatorCurrentAmps: Double = 0.0

        @JvmField var leftLeaderTempCelsius: Double = 0.0

        @JvmField var leftFollowerAppliedVolts: Double = 0.0

        @JvmField var leftFollowerSupplyCurrentAmps: Double = 0.0

        @JvmField var leftFollowerStatorCurrentAmps: Double = 0.0

        @JvmField var leftFollowerTempCelsius: Double = 0.0

        @JvmField var rightLeaderVelocityRadPerSec: Double = 0.0

        @JvmField var rightLeaderAppliedVolts: Double = 0.0

        @JvmField var rightLeaderSupplyCurrentAmps: Double = 0.0

        @JvmField var rightLeaderStatorCurrentAmps: Double = 0.0

        @JvmField var rightLeaderTempCelsius: Double = 0.0

        @JvmField var rightFollowerAppliedVolts: Double = 0.0

        @JvmField var rightFollowerSupplyCurrentAmps: Double = 0.0

        @JvmField var rightFollowerStatorCurrentAmps: Double = 0.0

        @JvmField var rightFollowerTempCelsius: Double = 0.0

        @JvmField var hoodPositionRad: Double = 0.0

        @JvmField var hoodVelocityRadPerSec: Double = 0.0

        @JvmField var hoodAppliedVolts: Double = 0.0

        @JvmField var hoodSupplyCurrentAmps: Double = 0.0

        @JvmField var hoodStatorCurrentAmps: Double = 0.0

        @JvmField var hoodTempCelsius: Double = 0.0
    }

    fun updateInputs(inputs: ShooterIOInputs) {}

    fun setFlywheelVelocity(velocity: AngularVelocity) {}

    fun setFlywheelVoltage(volts: Double) {}

    fun setHoodAngle(angle: Angle) {}

    fun setHoodVoltage(voltage: Double) {}

    fun resetHoodPosition(angle: Angle) {}
}
