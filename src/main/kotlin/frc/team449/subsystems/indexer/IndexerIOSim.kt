package frc.team449.subsystems.indexer

import com.ctre.phoenix6.sim.TalonFXSimState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.team449.Constants.IndexerConstants.BOTTOM_INDEXER_GEARING
import frc.team449.Constants.IndexerConstants.BOTTOM_INDEXER_MOI
import frc.team449.Constants.IndexerConstants.SIDE_INDEXER_GEARING
import frc.team449.Constants.IndexerConstants.SIDE_INDEXER_MOI
import frc.team449.Constants.IndexerConstants.TOP_INDEXER_GEARING
import frc.team449.Constants.IndexerConstants.TOP_INDEXER_MOI

class IndexerIOSim : IndexerIOHardware() {
    private val topGearbox = DCMotor.getKrakenX44(1)
    private val sideGearbox = DCMotor.getKrakenX60(1)
    private val bottomGearbox = DCMotor.getKrakenX44(1)

    private val topPlant = LinearSystemId.createDCMotorSystem(topGearbox, TOP_INDEXER_MOI, TOP_INDEXER_GEARING)
    private val sidePlant = LinearSystemId.createDCMotorSystem(sideGearbox, SIDE_INDEXER_MOI, SIDE_INDEXER_GEARING)
    private val bottomPlant = LinearSystemId.createDCMotorSystem(bottomGearbox, BOTTOM_INDEXER_MOI, BOTTOM_INDEXER_GEARING)

    private var topIndexerSim = DCMotorSim(topPlant, topGearbox)
    private var sideIndexerSim = DCMotorSim(sidePlant, sideGearbox)
    private var bottomIndexerSim = DCMotorSim(bottomPlant, bottomGearbox)

    init {
        val topIndexerMotorSim = topIndexerMotor.simState
        val sideIndexerMotorSim = sideIndexerMotor.simState
        val bottomIndexerMotorSim = bottomIndexerMotor.simState

        topIndexerMotorSim.setMotorType(TalonFXSimState.MotorType.KrakenX44)
        sideIndexerMotorSim.setMotorType(TalonFXSimState.MotorType.KrakenX60)
        bottomIndexerMotorSim.setMotorType(TalonFXSimState.MotorType.KrakenX44)
    }

    override fun simPeriodic() {
        val topIndexerMotorSim = topIndexerMotor.simState
        val sideIndexerMotorSim = sideIndexerMotor.simState
        val bottomIndexerMotorSim = bottomIndexerMotor.simState

        topIndexerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage())
        sideIndexerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage())
        bottomIndexerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage())

        topIndexerSim.setInput(topIndexerMotorSim.motorVoltageMeasure.`in`(Volts))
        sideIndexerSim.setInput(sideIndexerMotorSim.motorVoltageMeasure.`in`(Volts))
        bottomIndexerSim.setInput(bottomIndexerMotorSim.motorVoltageMeasure.`in`(Volts))

        topIndexerSim.update(0.02)
        sideIndexerSim.update(0.02)
        bottomIndexerSim.update(0.02)
    }
}
