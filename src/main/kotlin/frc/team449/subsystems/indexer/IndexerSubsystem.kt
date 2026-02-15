package frc.team449.subsystems.indexer
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants.IndexerConstants.BOTTOM_INDEXER_BACKWARD_VEL
import frc.team449.Constants.IndexerConstants.BOTTOM_INDEXER_FORWARD_VEL
import frc.team449.Constants.IndexerConstants.SIDE_INDEXER_BACKWARD_VEL
import frc.team449.Constants.IndexerConstants.SIDE_INDEXER_FORWARD_VEL
import frc.team449.Constants.IndexerConstants.TOP_INDEXER_BACKWARD_VEL
import frc.team449.Constants.IndexerConstants.TOP_INDEXER_FORWARD_VEL
import org.littletonrobotics.junction.Logger

/**
 * @file Indexer.kt
 * @brief This file contains functions for the indexer
 * @details This includes motor control and sensor control/definition functions for the indexer
 * @author Sean Zhang
*/

class IndexerSubsystem(
    private val io: IndexerIO
) : SubsystemBase() {
    private val inputs: IndexerInputsAutoLogged = IndexerInputsAutoLogged()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)
    }

    fun runIndexerForwards(): Command =
        runOnce {
            io.setIndexerVelocity(
                TOP_INDEXER_FORWARD_VEL,
                SIDE_INDEXER_FORWARD_VEL,
                BOTTOM_INDEXER_FORWARD_VEL,
            )
        }

    fun runIndexerBackwards(): Command =
        runOnce {
            io.setIndexerVelocity(
                TOP_INDEXER_BACKWARD_VEL,
                SIDE_INDEXER_BACKWARD_VEL,
                BOTTOM_INDEXER_BACKWARD_VEL,
            )
        }

    fun runIndexerAtVelocity(
        topVel: AngularVelocity,
        sideVel: AngularVelocity,
        bottomVel: AngularVelocity
    ): Command =
        runOnce {
            io.setIndexerVelocity(topVel, sideVel, bottomVel)
        }

    // stops motor
    fun stop(): Command =
        run {
            io.setVoltage(
                0.0,
                0.0,
                0.0,
            )
        }

    override fun simulationPeriodic() {
        io.simPeriodic()
    }
}
