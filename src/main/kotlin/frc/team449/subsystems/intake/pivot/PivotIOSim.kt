package frc.team449.subsystems.intake.pivot

import edu.wpi.first.math.system.plant.DCMotor

class PivotIOSim : PivotIO {
    // instant set to target, nothing much
    private val motor = DCMotor.getKrakenX44(1)
    private var targetVoltage = 0.0

    override fun setVoltage(voltage: Double) {
        targetVoltage = voltage
    }

    override fun updateInputs(pivotInputs: PivotIO.PivotIOInputs) {
        pivotInputs.currentVoltage = targetVoltage
        pivotInputs.targetVoltage = targetVoltage

        val current =
            motor.getCurrent(
                0.0,
                targetVoltage,
            )

        pivotInputs.supplyCurrent = current
    }
}
