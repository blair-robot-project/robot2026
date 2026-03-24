package frc.team449.subsystems.indexer

import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import frc.team449.Constants
import frc.team449.Constants.IndexerConstants.FLOOR_GEARING
import frc.team449.Constants.IndexerConstants.TOP_GEARING
import frc.team449.Constants.IndexerConstants.TOP_MOI_KG_MM
import frc.team449.Constants.IndexerConstants.WEDGE_GEARING
import frc.team449.Constants.IndexerConstants.WEDGE_MOI_KG_MM

class IndexerIOSim : IndexerIOHardware() {
    var wedgeSim =
        FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                WEDGE_MOI_KG_MM,
                WEDGE_GEARING,
            ),
            DCMotor.getKrakenX60(1),
        )

    var floorSim =
        FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                0.00000000001,
                FLOOR_GEARING,
            ),
            DCMotor.getKrakenX60(1),
        )

    var topSim =
        FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                TOP_MOI_KG_MM,
                TOP_GEARING,
            ),
            DCMotor.getKrakenX60(1),
        )

    private val wedgeMotorSim = wedgeIndexer.simState
    private val floorMotorSim = floorIndexer.simState
    private val topMotorSim = topIndexer.simState

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        wedgeMotorSim.setSupplyVoltage(12.0)
        wedgeSim.setInput(wedgeMotorSim.motorVoltage)
        wedgeSim.update(Constants.LOOP_TIME)
        wedgeMotorSim.setRotorVelocity(Units.radiansToRotations(wedgeSim.angularVelocity.`in`(RadiansPerSecond)) * WEDGE_GEARING)

        floorMotorSim.setSupplyVoltage(12.0)
        floorSim.setInput(floorMotorSim.motorVoltage)
        floorSim.update(Constants.LOOP_TIME)
        floorMotorSim.setRotorVelocity(Units.radiansToRotations(floorSim.angularVelocity.`in`(RadiansPerSecond)) * FLOOR_GEARING)

        topMotorSim.setSupplyVoltage(12.0)
        topSim.setInput(topMotorSim.motorVoltage)
        topSim.update(Constants.LOOP_TIME)
        topMotorSim.setRotorVelocity(Units.radiansToRotations(topSim.angularVelocity.`in`(RadiansPerSecond)) * TOP_GEARING)

        super.updateInputs(inputs)
    }
}
