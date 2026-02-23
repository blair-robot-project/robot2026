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

    var indexerTargetVelocityRadPerSec: Double = 0.0

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)

        Logger.recordOutput("Indexer/IndexerTargetRadPerSec", indexerTargetVelocityRadPerSec)
    }

    fun index(
        wedgeSpeed: AngularVelocity,
        floorSpeed: AngularVelocity,
        topSpeed: AngularVelocity
    ): Command =
        this.run {
            io.setFloorSpeed(floorSpeed)
            io.setWedgeSpeed(wedgeSpeed)
            io.setTopSpeed(topSpeed)
        }

    fun index(surfaceSpeed: AngularVelocity): Command {
        indexerTargetVelocityRadPerSec = surfaceSpeed.`in`(RadiansPerSecond)

        return index(
            surfaceSpeed,
            surfaceSpeed,
            surfaceSpeed,
        )
    }

    fun stop(): Command = index(RadiansPerSecond.of(0.0))
}
