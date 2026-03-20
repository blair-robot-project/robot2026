package frc.team449.subsystems.intake

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import org.littletonrobotics.junction.AutoLog

interface IntakeIO {
    @AutoLog
    open class IntakeIOInputs {
        @JvmField var leftPivotLeaderAppliedVolts: Double = 0.0

        @JvmField var leftPivotLeaderPositionRad: Double = 0.0

        @JvmField var leftPivotLeaderVelocityRadPerSec: Double = 0.0

        @JvmField var leftPivotLeaderSupplyCurrentAmps: Double = 0.0

        @JvmField var leftPivotLeaderStatorCurrentAmps: Double = 0.0

        @JvmField var leftPivotLeaderTempCelsius: Double = 0.0

        @JvmField var rightPivotFollowerAppliedVolts: Double = 0.0

        @JvmField var rightPivotFollowerSupplyCurrentAmps: Double = 0.0

        @JvmField var rightPivotFollowerStatorCurrentAmps: Double = 0.0

        @JvmField var rightPivotFollowerTempCelsius: Double = 0.0

        @JvmField var leftRollerLeaderAppliedVolts: Double = 0.0

        @JvmField var leftRollerLeaderVelocityRadPerSec: Double = 0.0

        @JvmField var leftRollerLeaderSupplyCurrentAmps: Double = 0.0

        @JvmField var leftRollerLeaderStatorCurrentAmps: Double = 0.0

        @JvmField var leftRollerLeaderTempCelsius: Double = 0.0

        @JvmField var rightRollerFollowerAppliedVolts: Double = 0.0

        @JvmField var rightRollerFollowerSupplyCurrentAmps: Double = 0.0

        @JvmField var rightRollerFollowerStatorCurrentAmps: Double = 0.0

        @JvmField var rightRollerFollowerTempCelsius: Double = 0.0
    }

    fun updateInputs(inputs: IntakeIOInputs) {}

    fun setPivotVoltage(volts: Double) {}

    fun setPivotAngle(angle: Angle) {}

    fun resetPivotAngle(angle: Angle) {}

    fun setRollerVelocity(velocity: AngularVelocity) {}

    fun setRollerVoltage(volts: Double) {}

    fun setSupplyLimits(pivotSupplyLimitAmps: Double, rollerSupplyLimitAmps: Double) {}
}
