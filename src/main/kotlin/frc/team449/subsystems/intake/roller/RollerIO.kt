package frc.team449.subsystems.intake.roller
import org.littletonrobotics.junction.AutoLog
import org.littletonrobotics.junction.LogTable
import org.littletonrobotics.junction.inputs.LoggableInputs

interface RollerIO {
//    open class RollerIOInputs {
//        @JvmField
//        var currentVelocity: Double = 0.0
//
//        @JvmField
//        var voltage: Double = 0.0
//
//        @JvmField
//        var supplyCurrent: Double = 0.0
//
//        @JvmField
//        var statorCurrent: Double = 0.0
//
//        @JvmField
//        var temperature: Double = 0.0
//
//        @JvmField
//        var motorIsConnected: Boolean = false
//    }

    @AutoLog
    open class MotorInputs : LoggableInputs {
        @JvmField var currentVelocity = 0.0

        @JvmField var voltage = 0.0

        @JvmField var supplyCurrent = 0.0

        @JvmField var statorCurrent = 0.0

        @JvmField var temperature = 0.0

        @JvmField var motorIsConnected = false

        override fun toLog(table: LogTable) {
            table.put("CurrentVelocity", currentVelocity)
            table.put("Voltage", voltage)
            table.put("SupplyCurrent", supplyCurrent)
            table.put("StatorCurrent", statorCurrent)
            table.put("Temperature", temperature)
            table.put("MotorIsConnected", motorIsConnected)
        }

        override fun fromLog(table: LogTable) {
            currentVelocity = table.get("CurrentVelocity", 0.0)
            voltage = table.get("Voltage", 0.0)
            supplyCurrent = table.get("SupplyCurrent", 0.0)
            statorCurrent = table.get("StatorCurrent", 0.0)
            temperature = table.get("Temperature", 0.0)
            motorIsConnected = table.get("MotorIsConnected", false)
        }
    }

    @AutoLog
    open class RollerIOInputs {
        @JvmField
        var leftMotor = MotorInputs()

        @JvmField
        var rightMotor = MotorInputs()
    }

    fun updateInputs(rollerInputs: RollerIOInputs) {}

    fun setVelocity(velocity: Double) {}
}
