package frc.team449.subsystems.indexer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.subsystems.indexer.IndexerConstants
/*
import au.grapplerobotics.interfaces.LaserCanInterface
import au.grapplerobotics.LaserCan
import au.grapplerobotics.simulation.MockLaserCan
*/

/**
 * @file Indexer.kt
 * @brief This file contains functions for the indexer
 * @details This includes motor control and sensor control/definition functions for the indexer
 * @author Sean Zhang
*/

class Indexer (
    val indexer: TalonFX, // kraken x60
    //lasercan def
    //private val conveyorSensor: LaserCanInterface,
): SubsystemBase() {
    private fun setVoltage(
        vararg motors: TalonFX,
        voltage: Double,
    ): Command =
        runOnce {
            motors.forEach { it.setVoltage(voltage) }
        }
}
    /*
    more lasercan def
    private var allSensorsConfigured = true
    private var lasercanConfigured = listOf<Boolean>()
    */


    /*
    init{
        try{
            initiation stuff
        }
    }
     */
//resets position of indexer
fun resetPos() {
    indexer.setPosition(0.0)
}