package frc.team449.subsystems.indexer
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

/**
 * @file Indexer.kt
 * @brief This file contains functions for the indexer
 * @details This includes motor control and sensor control/definition functions for the indexer
 * @author Sean Zhang
*/

class Indexer(
    private val io: IndexerIO
) : SubsystemBase() {
    private val inputs: IndexerInputsAutoLogged = IndexerInputsAutoLogged()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)
    }

    // sets voltage of motor
    fun setVoltage(
        topVoltage: Double,
        sideVoltage: Double,
        bottomVoltage: Double
    ): Command =
        run {
            io.setVoltage(
                topVoltage,
                sideVoltage,
                bottomVoltage,
            )
        }

    // stops motor
    fun stop(): Command =
        run {
            io.setVoltage(0.0, 0.0, 0.0)
        }

    fun setIndexerVelocity(
        topVel: AngularVelocity,
        sideVel: AngularVelocity,
        bottomVel: AngularVelocity
    ): Command =
        runOnce {
            io.setIndexerVelocity(topVel, sideVel, bottomVel)
        }
}
