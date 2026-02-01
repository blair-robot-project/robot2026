package frc.team449.subsystems.indexer
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

    // error

    private val inputs: IndexerInputsAutoLogged = IndexerInputsAutoLogged()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)
    }

    // sets voltage of motor
    // might be a while true? or something
    public fun setVoltage(leftVoltage: Double, rightVoltage: Double) = run{
        io.setVoltage(
            leftVoltage,
            rightVoltage
        )
    }

    // stops motor
    fun stop() =run{
        io.setVoltage(0.0, 0.0)
    }
}
