package frc.team449.subsystems.intake
import edu.wpi.first.math.system.plant.DCMotor

class IntakeIOSim : IntakeIO {
    // instant set to target
    private val pivotMotor = DCMotor.getKrakenX44(1)
    private val leftRollerMotor = DCMotor.getKrakenX60(1)
    private val rightRollerMotor = DCMotor.getKrakenX60(1)

    private var requestedPivotVoltage: Double = 0.0
    private var requestedLeftVoltage: Double = 0.0
    private var requestedRightVoltage: Double = 0.0

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        inputs.currentPivotVoltage = requestedPivotVoltage
        inputs.currentLeftRollerVoltage = requestedLeftVoltage
        inputs.currentRightRollerVoltage = requestedRightVoltage

        inputs.pivotSupplyCurrent = pivotMotor.getCurrent(0.0, requestedPivotVoltage)
        inputs.leftRollerSupplyCurrent = leftRollerMotor.getCurrent(0.0, requestedLeftVoltage)
        inputs.rightRollerSupplyCurrent = rightRollerMotor.getCurrent(0.0, requestedRightVoltage)
    }

    override fun setVoltage(
        pivotVoltage: Double,
        leftRollerVoltage: Double,
        rightRollerVoltage: Double
    ) {
        requestedPivotVoltage = pivotVoltage
        requestedLeftVoltage = leftRollerVoltage
        requestedRightVoltage = rightRollerVoltage
    }
}
