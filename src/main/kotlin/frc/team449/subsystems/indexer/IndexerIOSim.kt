package frc.team449.subsystems.indexer

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.team449.Constants
import frc.team449.Constants.IndexerConstants

class IndexerIOSim : IndexerIOHardware() {
    var floorSim =
        FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                IndexerConstants.FLOOR_MOI_KG_MM,
                IndexerConstants.FLOOR_GEARING,
            ),
            DCMotor.getKrakenX60(1),
        )

    var topSim =
        FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                IndexerConstants.TOP_MOI_KG_MM,
                IndexerConstants.TOP_GEARING,
            ),
            DCMotor.getKrakenX60(1),
        )

    private val floorMotorSim = floor.simState
    private val topMotorSim = top.simState

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        super.updateInputs(inputs)

        // update floor indexer sim
        floorMotorSim.setSupplyVoltage(12.0)
        floorSim.setInput(floorMotorSim.motorVoltage)
        floorSim.update(Constants.LOOP_TIME)
        floorMotorSim.setRotorVelocity(Units.radiansToRotations(floorSim.angularVelocity.`in`(RadiansPerSecond)) * IndexerConstants.FLOOR_GEARING)

        // update top indexer sim
        topMotorSim.setSupplyVoltage(12.0)
        topSim.setInput(topMotorSim.motorVoltage)
        topSim.update(Constants.LOOP_TIME)
        topMotorSim.setRotorVelocity(Units.radiansToRotations(topSim.angularVelocity.`in`(RadiansPerSecond)) * IndexerConstants.TOP_GEARING)
    }
}
