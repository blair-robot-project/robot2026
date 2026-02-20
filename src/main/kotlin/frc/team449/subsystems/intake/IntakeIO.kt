package frc.team449.subsystems.intake

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import org.littletonrobotics.junction.AutoLog

interface IntakeIO {
    @AutoLog
    open class IntakeIOInputs {
        @JvmField var leftPivotAppliedVolts: Double = 0.0

        @JvmField var leftPivotCurrentState: String = "None" // "STOW", "DEPLOY", etc.

        @JvmField var leftPivotPositionRad: Double = 0.0

        @JvmField var leftPivotVelocityRadPerSec: Double = 0.0

        @JvmField var leftPivotSupplyCurrentAmps: Double = 0.0

        @JvmField var leftPivotStatorCurrentAmps: Double = 0.0

        @JvmField var leftPivotTempCelsius: Double = 0.0

        @JvmField var rightPivotAppliedVolts: Double = 0.0

        @JvmField var rightPivotSupplyCurrentAmps: Double = 0.0

        @JvmField var rightPivotStatorCurrentAmps: Double = 0.0

        @JvmField var rightPivotTempCelsius: Double = 0.0

        // leftRollers

        @JvmField var leftRollerAppliedVolts: Double = 0.0

        @JvmField var leftRollerControlMode: String = "None"

        @JvmField var leftRollerVelocityRadPerSec: Double = 0.0

        @JvmField var leftRollerSupplyCurrentAmps: Double = 0.0

        @JvmField var leftRollerStatorCurrentAmps: Double = 0.0

        @JvmField var leftRollerTempCelsius: Double = 0.0

        @JvmField var rightRollerFollowerAppliedVolts: Double = 0.0

        @JvmField var rightRollerFollowerSupplyCurrentAmps: Double = 0.0

        @JvmField var rightRollerFollowerStatorCurrentAmps: Double = 0.0

        @JvmField var rightRollerFollowerTempCelsius: Double = 0.0
    }

    fun updateInputs(inputs: IntakeIOInputs) {}

    fun setPivotVoltage(volts: Double) {}

    fun setRollerVelocity(velocity: AngularVelocity) {}
}
