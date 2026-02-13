package frc.team449.subsystems.indexer

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
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
            0.01,
            1.0,
        )

    private var topMotorSim = DCMotorSim(topPlant, topGearbox)
    private var sideMotorSim = DCMotorSim(sidePlant, sideGearbox)
    private var bottomMotorSim = DCMotorSim(bottomPlant, bottomGearbox)

    override fun setVoltage(
        topVoltage: Double,
        sideVoltage: Double,
        bottomVoltage: Double
    ) {
        topMotorSim.setInput(sideVoltage)
        sideMotorSim.setInput(sideVoltage)
        bottomMotorSim.setInput(bottomVoltage)
    }

    override fun setIndexerVelocity(
        topVel: AngularVelocity,
        sideVel: AngularVelocity,
        bottomVel: AngularVelocity
    ) {
        topMotorSim.setInput(topVel.`in`(Units.RadiansPerSecond))
        sideMotorSim.setInput(sideVel.`in`(Units.RadiansPerSecond))
        bottomMotorSim.setInput(bottomVel.`in`(Units.RadiansPerSecond))
    }

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        topMotorSim.update(0.02)
        sideMotorSim.update(0.02)
        bottomMotorSim.update(0.02)

        inputs.topVoltage = topMotorSim.inputVoltage
        inputs.topVelocity = topMotorSim.angularVelocityRadPerSec
        inputs.topStatorCurrent = topMotorSim.currentDrawAmps

        inputs.sideVoltage = sideMotorSim.inputVoltage
        inputs.sideVelocity = sideMotorSim.angularVelocityRadPerSec
        inputs.sideStatorCurrent = sideMotorSim.currentDrawAmps

        inputs.bottomVoltage = bottomMotorSim.inputVoltage
        inputs.bottomVelocity = bottomMotorSim.angularVelocityRadPerSec
        inputs.bottomStatorCurrent = bottomMotorSim.currentDrawAmps
    }
}
