package frc.team449.subsystems.intake

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import org.littletonrobotics.junction.AutoLog

interface IntakeIO {
    @AutoLog
    open class IntakeIOInputs {
        @JvmField var pivotAppliedVolts: Double = 0.0

        @JvmField var pivotCurrentState: String = "None" // "STOW", "DEPLOY", etc.

        @JvmField var pivotPositionRad: Double = 0.0

        @JvmField var pivotVelocityRadPerSec: Double = 0.0

        @JvmField var pivotSupplyCurrentAmps: Double = 0.0

        @JvmField var pivotStatorCurrentAmps: Double = 0.0

        @JvmField var pivotTempCelsius: Double = 0.0

        @JvmField var pivotFollowerAppliedVolts: Double = 0.0

        @JvmField var pivotFollowerSupplyCurrentAmps: Double = 0.0

        @JvmField var pivotFollowerStatorCurrentAmps: Double = 0.0

        @JvmField var pivotFollowerTempCelsius: Double = 0.0

        // rollers

        @JvmField var rollerAppliedVolts: Double = 0.0

        @JvmField var rollerControlMode: String = "None"

        @JvmField var rollerVelocityRadPerSec: Double = 0.0

        @JvmField var rollerSupplyCurrentAmps: Double = 0.0

        @JvmField var rollerStatorCurrentAmps: Double = 0.0

        @JvmField var rollerTempCelsius: Double = 0.0

        @JvmField var rollerFollowerAppliedVolts: Double = 0.0

        @JvmField var rollerFollowerSupplyCurrentAmps: Double = 0.0

        @JvmField var rollerFollowerStatorCurrentAmps: Double = 0.0

        @JvmField var rollerFollowerTempCelsius: Double = 0.0
    }

    fun updateInputs(inputs: IntakeIOInputs) {}

    fun setPivotVoltage(volts: Double) {}

    fun setRollerVelocity(velocity: AngularVelocity) {}
}
