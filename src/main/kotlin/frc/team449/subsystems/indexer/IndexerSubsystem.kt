package frc.team449.subsystems.indexer
import edu.wpi.first.units.Units.RadiansPerSecond
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

class IndexerSubsystem(
    private val io: IndexerIO
) : SubsystemBase() {
    private val inputs: IndexerInputsAutoLogged = IndexerInputsAutoLogged()

    var wedgeTargetVelocityRadPerSec: Double = 0.0
    var floorTargetVelocityRadPerSec: Double = 0.0
    var topTargetVelocityRadPerSec: Double = 0.0

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)

        Logger.recordOutput("Indexer/FloorTargetRadPerSec", floorTargetVelocityRadPerSec)
        Logger.recordOutput("Indexer/WedgeTargetRadPerSec", wedgeTargetVelocityRadPerSec)
        Logger.recordOutput("Indexer/TopTargetVelocityRadPerSec", topTargetVelocityRadPerSec)
    }

    fun index(
        floorSpeed: AngularVelocity,
        wedgeSpeed: AngularVelocity,
        topSpeed: AngularVelocity
    ): Command =
        this.run {
            floorTargetVelocityRadPerSec = floorSpeed.`in`(RadiansPerSecond)
            wedgeTargetVelocityRadPerSec = wedgeSpeed.`in`(RadiansPerSecond)
            topTargetVelocityRadPerSec = topSpeed.`in`(RadiansPerSecond)

            io.setFloorSpeed(floorSpeed)
            io.setWedgeSpeed(wedgeSpeed)
            io.setTopSpeed(topSpeed)
        }

    fun index(surfaceSpeed: AngularVelocity): Command {
        return index(
            surfaceSpeed,
            surfaceSpeed,
            surfaceSpeed,
        )
    }

    fun stop(): Command =
        this.runOnce {
            floorTargetVelocityRadPerSec = 0.0
            wedgeTargetVelocityRadPerSec = 0.0
            topTargetVelocityRadPerSec = 0.0
            io.setIndexerVoltage(0.0, 0.0, 0.0)
        }

    fun setSupplyLimits(floorSupplyLimitAmps: Double, wedgeSupplyLimitAmps: Double, topSupplyLimitAmps: Double) {
        io.setSupplyLimits(floorSupplyLimitAmps, wedgeSupplyLimitAmps, topSupplyLimitAmps)
    }
}
