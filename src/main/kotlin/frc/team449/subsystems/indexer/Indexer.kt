package frc.team449.subsystems.indexer
import frc.team449.Constants
import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj2.command.Command
import au.grapplerobotics.interfaces.LaserCanInterface
import au.grapplerobotics.LaserCan
import au.grapplerobotics.simulation.MockLaserCan


/**
 * @file Indexer.kt
 * @brief This file contains functions for the indexer
 * @details This includes motor control and sensor control/definition functions for the indexer
 * @author Sean Zhang
*/

class Indexer (
    //define motor port in robotcontainer.kt
    val indexer: TalonFX = TalonFX(Constants.IndexerConstants.INDEXER_ID), // kraken x60
    val indexer2: TalonFX = TalonFX(Constants.IndexerConstants.INDEXER_ID_2), //kraken x60
    private val indexSensor: LaserCanInterface,

): SubsystemBase() {
    private val sensors =
        listOf(
            indexSensor
        )
    private var allSensorsConfigured = true
    private var lasercanConfigured = listOf<Boolean>()

    init {
        try {
            indexSensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_20MS)
            for (sensor in sensors) {
                if (sensor != sensors) {
                    sensor.setTimingBudget(LaserCanInterface.TimingBudget.TIMING_BUDGET_33MS)
                }
                sensor.setRegionOfInterest(LaserCanInterface.RegionOfInterest(8, 8, 4, 4))
                sensor.setRangingMode(LaserCanInterface.RangingMode.SHORT)
                lasercanConfigured.plus(true)
            }
        } catch (_: Exception) {
            lasercanConfigured.plus(false)
            allSensorsConfigured = false
        }
    }

    //sets voltage of motor
    private fun setVoltage(voltage: Double, voltage2: Double){
        indexer.setVoltage(voltage)
        indexer2.setVoltage(voltage2)
    }

    //stops motor
    fun stop(): Command=runOnce{
        indexer.setVoltage(0.0)
        indexer2.setVoltage(0.0)
    }

    //reset pos of indexer
    fun resetPos() {
        indexer.setPosition(0.0)
        indexer.setPosition(0.0)
    }

    //lasercan detected info
    private fun laserCanDetected(laserCan: LaserCanInterface): Boolean {
        val measurement = laserCan.measurement
        return measurement != null && (
                measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT &&
                        measurement.distance_mm <= Constants.IndexerConstants.INDEXER_DETECTION_THRESHOLD

                )
    }

    //t/f function to see if the ball is detected
    fun ballDetected(): Boolean = laserCanDetected(indexSensor)
    fun ballNotDetected(): Boolean = !ballDetected()


    fun runDetect(volt:Double): Command=runOnce{
        setVoltage(volt, volt)}.onlyIf { ballNotDetected() }.andThen(stop())
}
