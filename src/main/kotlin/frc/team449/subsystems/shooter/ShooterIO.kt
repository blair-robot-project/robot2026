package frc.team449.subsystems.shooter
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.inputs.LoggableInputs

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
    }

    fun updateInputs(inputs: ShooterIOInputs) {}

    fun run(voltage: Double) {}

    fun stop() {}
}