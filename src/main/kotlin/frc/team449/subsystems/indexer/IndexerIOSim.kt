package frc.team449.subsystems.indexer

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.simulation.DCMotorSim

class IndexerIOSim : IndexerIO {
    private val topGearbox = DCMotor.getKrakenX44(1)
    private val sideGearbox = DCMotor.getKrakenX60(1)
    private val bottomGearbox = DCMotor.getKrakenX60(1)

    // TODO: MOI and gearing
    private val topPlant =
        LinearSystemId.createDCMotorSystem(
            topGearbox,
            0.001,
            1.0,
        )

    private val sidePlant =
        LinearSystemId.createDCMotorSystem(
            sideGearbox,
            0.001,
            1.0,
        )

    private val bottomPlant =
        LinearSystemId.createDCMotorSystem(
            bottomGearbox,
            0.001,
            1.0,
        )


    private var topMotorSim = DCMotorSim(topPlant, topGearbox)
    private var sideMotorSim = DCMotorSim(sidePlant, sideGearbox)
    private var bottomMotorSim = DCMotorSim(bottomPlant, bottomGearbox)

    private var topVoltSim: Double = 0.0
    private var sideVoltSim: Double = 0.0
    private var bottomVoltSim: Double = 0.0

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        topMotorSim.update(0.02)
        sideMotorSim.update(0.02)
        bottomMotorSim.update(0.02)

        topMotorSim.setInput(topVoltSim)
        sideMotorSim.setInput(sideVoltSim)
        bottomMotorSim.setInput(bottomVoltSim)

        inputs.topVoltage = topMotorSim.inputVoltage
        inputs.topStatorCurrent = topMotorSim.currentDrawAmps

        inputs.sideVoltage = sideMotorSim.inputVoltage
        inputs.sideStatorCurrent = sideMotorSim.currentDrawAmps

        inputs.bottomVoltage = bottomMotorSim.inputVoltage
        inputs.bottomStatorCurrent = bottomMotorSim.currentDrawAmps
    }

    override fun setVoltage(
        topVoltage: Double,
        sideVoltage: Double,
        bottomVoltage: Double
    ) {
        topVoltSim = topVoltage
        sideVoltSim = sideVoltage
        bottomVoltSim = bottomVoltage
    }

}
