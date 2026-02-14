package frc.team449.subsystems.indexer

import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.simulation.DCMotorSim
import frc.team449.Constants.IndexerConstants.BOTTOM_INDEXER_GEARING
import frc.team449.Constants.IndexerConstants.BOTTOM_INDEXER_ID
import frc.team449.Constants.IndexerConstants.BOTTOM_INDEXER_MOI
import frc.team449.Constants.IndexerConstants.SIDE_INDEXER_GEARING
import frc.team449.Constants.IndexerConstants.SIDE_INDEXER_ID
import frc.team449.Constants.IndexerConstants.SIDE_INDEXER_MOI
import frc.team449.Constants.IndexerConstants.TOP_INDEXER_GEARING
import frc.team449.Constants.IndexerConstants.TOP_INDEXER_ID
import frc.team449.Constants.IndexerConstants.TOP_INDEXER_MOI

class IndexerIOSim : IndexerIO {
    private val topGearbox = DCMotor.getKrakenX44(1)
    private val sideGearbox = DCMotor.getKrakenX60(1)
    private val bottomGearbox = DCMotor.getKrakenX44(1)

    val topIndexer: TalonFX = TalonFX(TOP_INDEXER_ID)
    val bottomIndexer: TalonFX = TalonFX(BOTTOM_INDEXER_ID)
    val sideIndexer: TalonFX = TalonFX(SIDE_INDEXER_ID)

    // TODO: MOI and gearing
    private val topPlant = LinearSystemId.createDCMotorSystem(topGearbox, TOP_INDEXER_MOI, TOP_INDEXER_GEARING)
    private val sidePlant = LinearSystemId.createDCMotorSystem(sideGearbox, SIDE_INDEXER_MOI, SIDE_INDEXER_GEARING)
    private val bottomPlant = LinearSystemId.createDCMotorSystem(bottomGearbox, BOTTOM_INDEXER_MOI, BOTTOM_INDEXER_GEARING)

    private var topMotorSim = DCMotorSim(topPlant, topGearbox)
    private var sideMotorSim = DCMotorSim(sidePlant, sideGearbox)
    private var bottomMotorSim = DCMotorSim(bottomPlant, bottomGearbox)

    override fun setVoltage(
        topVoltage: Double,
        sideVoltage: Double,
        bottomVoltage: Double
    ) {
        topMotorSim.setInput(topVoltage)
        sideMotorSim.setInput(sideVoltage)
        bottomMotorSim.setInput(bottomVoltage)
    }

    override fun setIndexerVelocity(
        topVel: AngularVelocity,
        sideVel: AngularVelocity,
        bottomVel: AngularVelocity
    ) {
        topIndexer.setControl(VelocityVoltage(topVel))
        sideIndexer.setControl(VelocityVoltage(sideVel))
        bottomIndexer.setControl(VelocityVoltage(bottomVel))
    }

    override fun simPeriodic() {
        val topVoltage = topIndexer.motorVoltage.value.`in`(Volts)
        val sideVoltage = sideIndexer.motorVoltage.value.`in`(Volts)
        val bottomVoltage = bottomIndexer.motorVoltage.value.`in`(Volts)

        topMotorSim.setInput(topVoltage)
        sideMotorSim.setInput(sideVoltage)
        bottomMotorSim.setInput(bottomVoltage)

        topMotorSim.update(0.02)
        sideMotorSim.update(0.02)
        bottomMotorSim.update(0.02)
    }

    override fun updateInputs(inputs: IndexerIO.IndexerInputs) {
        inputs.topVoltage = topIndexer.motorVoltage.value.`in`(Volts)
        inputs.topVelocity = topMotorSim.angularVelocityRadPerSec
        inputs.topStatorCurrent = topMotorSim.currentDrawAmps

        inputs.sideVoltage = sideIndexer.motorVoltage.value.`in`(Volts)
        inputs.sideVelocity = sideMotorSim.angularVelocityRadPerSec
        inputs.sideStatorCurrent = sideMotorSim.currentDrawAmps

        inputs.bottomVoltage = bottomIndexer.motorVoltage.value.`in`(Volts)
        inputs.bottomVelocity = bottomMotorSim.angularVelocityRadPerSec
        inputs.bottomStatorCurrent = bottomMotorSim.currentDrawAmps
    }
}
