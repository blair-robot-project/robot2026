package frc.team449.subsystems.intake
import edu.wpi.first.math.system.plant.DCMotor

class IntakeIOSim : IntakeIO {
    // instant set to target
    private val pivotMotor = DCMotor.getKrakenX44(1)
    private val leftRollerMotor = DCMotor.getKrakenX60(1)
    private val rightRollerMotor = DCMotor.getKrakenX60(1)

    private var targetPivotVoltage: Double = 0.0
    private var targetLeftVoltage: Double = 0.0
    private var targetRightVoltage: Double = 0.0

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        inputs.currentPivotVoltage = targetPivotVoltage
        inputs.currentLeftRollerVoltage = targetLeftVoltage
        inputs.currentRightRollerVoltage = targetRightVoltage

        inputs.targetPivotVoltage = targetPivotVoltage
        inputs.targetLeftRollerVoltage = targetLeftVoltage
        inputs.targetRightRollerVoltage = targetRightVoltage

        inputs.pivotSupplyCurrent = pivotMotor.getCurrent(0.0, targetPivotVoltage)
        inputs.leftRollerSupplyCurrent = leftRollerMotor.getCurrent(0.0, targetLeftVoltage)
        inputs.rightRollerSupplyCurrent = rightRollerMotor.getCurrent(0.0, targetRightVoltage)
    }

    override fun setVoltage(
        pivotVoltage: Double,
        leftRollerVoltage: Double,
        rightRollerVoltage: Double,
    ) {
        targetPivotVoltage = pivotVoltage
        targetLeftVoltage = leftRollerVoltage
        targetRightVoltage = rightRollerVoltage
    }
}
