package frc.team449.subsystems.shooter
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import frc.team449.Constants.ShooterConstants.HOOD_MIN_ANGLE
import org.littletonrobotics.junction.AutoLog

interface ShooterIO {
    @AutoLog
    open class ShooterIOInputs {
        @JvmField
        var leftVoltage: Double = 0.0

        @JvmField
        var leftSupplyCurrent: Double = 0.0

        @JvmField
        var leftStatorCurrent: Double = 0.0

        @JvmField
        var leftTemperature: Double = 0.0

        @JvmField
        var leftMotorIsConnected: Boolean = false

        @JvmField
        var leftFollowerMotorIsConnected: Boolean = false

        @JvmField
        var rightVoltage: Double = 0.0

        @JvmField
        var rightSupplyCurrent: Double = 0.0

        @JvmField
        var rightStatorCurrent: Double = 0.0

        @JvmField
        var rightTemperature: Double = 0.0

        @JvmField
        var rightMotorIsConnected: Boolean = false

        @JvmField
        var rightFollowerMotorIsConneted: Boolean = false

        @JvmField
        var flywheelVelocity = 0.0

        @JvmField
        var hoodVoltage: Double = 0.0

        @JvmField
        var hoodCurrentPos: Double = HOOD_MIN_ANGLE.`in`(Radians)

        @JvmField
        var hoodTargetPos: Double = HOOD_MIN_ANGLE.`in`(Radians)

        @JvmField
        var hoodSupplyCurrent: Double = 0.0

        @JvmField
        var hoodStatorCurrent: Double = 0.0

        @JvmField
        var hoodTemperature: Double = 0.0

        @JvmField
        var hoodMotorIsConnected: Boolean = false
    }

    fun updateInputs(inputs: ShooterIOInputs) {}

    fun runFlywheelAtVelocity(velocity: AngularVelocity) { }

    fun setHoodPosition(angle: Angle) { }

    fun atTolerance(): Boolean { return true }

    fun simPeriodic() {}

    fun getHoodPosition(): Angle { return Radians.of(0.0) }
}
