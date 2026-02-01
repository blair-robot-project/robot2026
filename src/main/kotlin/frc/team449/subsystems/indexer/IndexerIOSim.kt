package frc.team449.subsystems.indexer

import edu.wpi.first.math.system.plant.DCMotor

// implement from indexer io thing
class IndexerIOSim : IndexerIO {
    private val leftIndexerMotorModel = DCMotor.getKrakenX44(1)
    private val rightIndexerMotorModel = DCMotor.getKrakenX44(1)
    private var leftVoltSim: Double = 0.0
    private var rightVoltSim: Double = 0.0
    private var leftCurrentSim: Double = 0.0
    private var rightCurrentSim: Double = 0.0


    // add current supply to this function
    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        // Voltage inputs (assigned from your sim variables)
        inputs.leftVoltage = leftVoltSim
        inputs.rightVoltage = rightVoltSim

        inputs.supplyCurrentLeft = leftCurrentSim
        inputs.statorCurrentLeft = leftCurrentSim

        inputs.supplyCurrentRight = rightCurrentSim
        inputs.statorCurrentRight = rightCurrentSim
    }

    override fun setVoltage(leftVoltage: Double, rightVoltage: Double) {
        // include this.leftVoltSim if param is leftVoltSim
        leftVoltSim = leftVoltage
        rightVoltSim = rightVoltage
    }
}
