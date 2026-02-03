package frc.team449.subsystems.indexer

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.simulation.DCMotorSim

class IndexerIOSim : IndexerIO {
    private val leftGearbox = DCMotor.getKrakenX44(1)
    private val rightGearbox = DCMotor.getKrakenX60(1)

    // TODO: MOI and gearing
    private val leftPlant =
        LinearSystemId.createDCMotorSystem(
            leftGearbox,
            0.001,
            1.0,
        )
    private val rightPlant =
        LinearSystemId.createDCMotorSystem(
            leftGearbox,
            0.001,
            1.0,
        )

    var leftMotorSim = DCMotorSim(leftPlant, leftGearbox)
    var rightMotorSim = DCMotorSim(rightPlant, rightGearbox)

    private var leftVoltSim: Double = 0.0
    private var rightVoltSim: Double = 0.0

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        leftMotorSim.update(0.02)
        rightMotorSim.update(0.02)

        leftMotorSim.setInput(leftVoltSim)
        rightMotorSim.setInput(rightVoltSim)

        inputs.leftVoltage = leftMotorSim.inputVoltage
        inputs.rightVoltage = rightMotorSim.inputVoltage

        inputs.leftVelocity = leftMotorSim.angularVelocity.`in`(Units.RotationsPerSecond)
        inputs.rightVelocity = rightMotorSim.angularVelocity.`in`(Units.RotationsPerSecond)
    }

    override fun setVoltage(
        leftVoltage: Double,
        rightVoltage: Double
    ) {
        leftVoltSim = leftVoltage
        rightVoltSim = rightVoltage
    }
}
