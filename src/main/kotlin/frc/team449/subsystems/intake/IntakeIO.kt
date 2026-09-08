package frc.team449.subsystems.intake

import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import org.littletonrobotics.junction.AutoLog

interface IntakeIO {
    @AutoLog
    open class IntakeIOInputs {
        @JvmField var leftPivotConnected: Boolean = false

        @JvmField var leftPivotAppliedVolts: Double = 0.0

        @JvmField var leftPivotPositionRads: Double = 0.0

        @JvmField var leftPivotVelocityRadsPerSec: Double = 0.0

        @JvmField var leftPivotSupplyCurrentAmps: Double = 0.0

        @JvmField var leftPivotStatorCurrentAmps: Double = 0.0

        @JvmField var leftPivotTempCelsius: Double = 0.0

        @JvmField var rightPivotConnected: Boolean = false

        @JvmField var rightPivotAppliedVolts: Double = 0.0

        @JvmField var rightPivotPositionRads: Double = 0.0

        @JvmField var rightPivotVelocityRadsPerSec: Double = 0.0

        @JvmField var rightPivotSupplyCurrentAmps: Double = 0.0

        @JvmField var rightPivotStatorCurrentAmps: Double = 0.0

        @JvmField var rightPivotTempCelsius: Double = 0.0

        @JvmField var leftRollerLeaderConnected: Boolean = false

        @JvmField var leftRollerLeaderAppliedVolts: Double = 0.0

        @JvmField var leftRollerLeaderVelocityRadsPerSec: Double = 0.0

        @JvmField var leftRollerLeaderSupplyCurrentAmps: Double = 0.0

        @JvmField var leftRollerLeaderStatorCurrentAmps: Double = 0.0

        @JvmField var leftRollerLeaderTempCelsius: Double = 0.0

        @JvmField var rightRollerFollowerConnected: Boolean = false

        @JvmField var rightRollerFollowerAppliedVolts: Double = 0.0

        @JvmField var rightRollerFollowerVelocityRadsPerSec: Double = 0.0

        @JvmField var rightRollerFollowerSupplyCurrentAmps: Double = 0.0

        @JvmField var rightRollerFollowerStatorCurrentAmps: Double = 0.0

        @JvmField var rightRollerFollowerTempCelsius: Double = 0.0
    }

    fun updateInputs(inputs: IntakeIOInputs) {}

    fun setPivotVoltage(pivotVolts: Double) {}

    fun setPivotAngle(pivotAngle: Angle) {}

    fun resetPivotAngle(pivotAngle: Angle) {}

    fun setRollerVoltage(rollerVolts: Double) {}
}
