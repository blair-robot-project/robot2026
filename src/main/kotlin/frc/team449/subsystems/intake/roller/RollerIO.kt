package frc.team449.subsystems.intake.roller
import frc.team449.subsystems.intake.pivot.PivotIO.PivotIOInputs
import org.littletonrobotics.junction.AutoLog

interface RollerIO {
    @AutoLog
    open class RollerIOInputs {
        @JvmField
        var velocity: Double = 0.0

        @JvmField
        var voltage: Double = 0.0

        @JvmField
        var supplyCurrent: Double = 0.0

        @JvmField
        var statorCurrent: Double = 0.0

        @JvmField
        var temperature: Double = 0.0

        @JvmField
        var motorIsConnected: Boolean = false
    }

    fun updateInputs(rollerInputs: RollerIOInputs) {}

    fun setVelocity(velocity: Double) {}
}
