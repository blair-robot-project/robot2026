package frc.team449.subsystems.intake.pivot

import org.littletonrobotics.junction.AutoLog

interface PivotIO {
    @AutoLog
    open class PivotIOInputs {
        @JvmField
        var currentVoltage: Double = 0.0

        @JvmField
        var targetVoltage: Double = 0.0

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

    fun setVoltage(voltage: Double) {}
}
