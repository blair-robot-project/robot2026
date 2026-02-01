package frc.team449.subsystems.indexer

import edu.wpi.first.math.system.plant.DCMotor


//implement from indexer io thing
class IndexerIOSim : IndexerIO {
    private val leftIndexerMotorModel = DCMotor.getKrakenX60(1)
    private val rightIndexerMotorModel = DCMotor.getKrakenX60(1)
    private var leftVoltSim: Double = 0.0
    private var rightVoltSim: Double = 0.0
    //add current supply to this function
    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        inputs.leftVoltage = leftVoltSim
        inputs.rightVoltage = rightVoltSim
    }

    override fun setVoltage(voltage: Double, voltage2: Double) {
        //include this.leftVoltSim if param is leftVoltSim
        leftVoltSim = voltage
        rightVoltSim = voltage2
    }

    override fun resetPosition() {
        leftVoltSim = 0.0
        rightVoltSim = 0.0
    }
}