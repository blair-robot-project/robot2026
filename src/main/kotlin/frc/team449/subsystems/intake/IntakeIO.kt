package frc.team449.subsystems.intake

import org.littletonrobotics.junction.AutoLog

interface IntakeIO {
    @AutoLog
    open class IntakeIOInputs {
        @JvmField var currentPivotVoltage: Double = 0.0

        @JvmField var currentLeftRollerVoltage: Double = 0.0

        @JvmField var currentRightRollerVoltage: Double = 0.0

        @JvmField var pivotSupplyCurrent: Double = 0.0

        @JvmField var leftRollerSupplyCurrent: Double = 0.0

        @JvmField var rightRollerSupplyCurrent: Double = 0.0

        @JvmField var pivotStatorCurrent: Double = 0.0

        @JvmField var leftRollerStatorCurrent: Double = 0.0

        @JvmField var rightRollerStatorCurrent: Double = 0.0
    }

    fun updateInputs(inputs: IntakeIOInputs) {}

    fun setVoltagePivot(
        pivotVoltage: Double
    ){}
    fun setVoltageRoller(
        rightRollerVoltage: Double
    ) {}
}
