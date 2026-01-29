package frc.team449.subsystems.intake.pivot

import edu.wpi.first.units.measure.Angle
import org.littletonrobotics.junction.AutoLog

interface PivotIO {
    @AutoLog
    open class PivotIOInputs {
        @JvmField
        var currentAngle: Double = 0.0

        @JvmField
        var targetAngle: Double = 0.0

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

    fun updateInputs(pivotInputs: PivotIOInputs) {}

    fun setAngle(angle: Angle) {}
}
