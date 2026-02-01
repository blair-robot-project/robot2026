package frc.team449.subsystems.indexer
import frc.team449.Constants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger


/**
 * @file Indexer.kt
 * @brief This file contains functions for the indexer
 * @details This includes motor control and sensor control/definition functions for the indexer
 * @author Sean Zhang
*/

class Indexer (
    private val io: IndexerIO,

): SubsystemBase() {

    //we need line 23 through 30 but idk how to implement correctly
    /*private val inputs: inputs = IndexerIOInputsAutoLogged()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)
    }
    */


    //sets voltage of motor
    //might be a while true? or something
    private fun setVoltage(voltage: Double, voltage2: Double) {
        io.setVoltage(
            voltage,
            voltage2
        )
    }

    //stops motor
    fun stop(): Command = runOnce {
        io.setVoltage(0.0, 0.0)
    }

    //reset pos of indexer
    fun resetPos() {
        io.resetPosition()

    }




}
