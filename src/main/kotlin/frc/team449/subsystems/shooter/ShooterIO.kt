package frc.team449.subsystems.shooter
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import frc.team449.Constants
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
        var hoodVoltage: Double = 0.0

        @JvmField
        var hoodCurrentPos: Double = Constants.ShooterConstants.HOOD_MIN_ANGLE.`in`(Radians)

        @JvmField
        var hoodTargetPos: Double = Constants.ShooterConstants.HOOD_MIN_ANGLE.`in`(Radians)

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

    fun run(voltage: Double) {}

    fun stop() {}

    fun setHood(angle: Angle) {}

    fun holdHood() {}

    fun atTolerance(): Boolean { return true }
}
