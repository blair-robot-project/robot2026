package frc.team449.subsystems.indexer


//implement from indexer io thing
class IndexerIOSim : IndexerIO {
    var leftVoltSim: Double = 0.0
    var rightVoltSim: Double = 0.0

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        inputs.leftVoltage = leftVoltSim
        inputs.rightVoltage = rightVoltSim
        //useless
        inputs.motorIsConnected = true
    }

    override fun setVoltage(voltage: Double, voltage2: Double) {
        this.leftVoltSim = voltage
        this.rightVoltSim = voltage2
    }
}