package frc.team449.subsystems.indexer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

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

    var wedgeTargetVolts: Double = 0.0
    var floorTargetVolts: Double = 0.0
    var topTargetVolts: Double = 0.0

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)

        Logger.recordOutput("Indexer/FloorTargetVolts", floorTargetVolts)
        Logger.recordOutput("Indexer/WedgeTargetVolts", wedgeTargetVolts)
        Logger.recordOutput("Indexer/TopTargetVolts", topTargetVolts)
    }

    fun index(
        floorVolts: Double,
        wedgeVolts: Double,
        topVolts: Double
    ): Command =
        this.run {
            floorTargetVolts = floorVolts
            wedgeTargetVolts = wedgeVolts
            topTargetVolts = topVolts

            io.setIndexerVoltage(floorVolts, wedgeVolts, topVolts)
        }

    fun index(volts: Double): Command =
        index(
            volts,
            volts,
            volts,
        )

    fun stop(): Command =
        this.run {
            floorTargetVolts = 0.0
            wedgeTargetVolts = 0.0
            topTargetVolts = 0.0
            io.setIndexerVoltage(0.0, 0.0, 0.0)
        }

    fun indexerAtTolerance(): Boolean {
        val floor = abs(inputs.floorAppliedVolts - floorTargetVolts)
        val wedge = abs(inputs.wedgeAppliedVolts - wedgeTargetVolts)
        val top = abs(inputs.topAppliedVolts - topTargetVolts)

        val isAtTarget = floor < 1.0 && wedge < 1.0 && top < 1.0

        return isAtTarget
    }
}
