package frc.team449.subsystems.indexer
import edu.wpi.first.units.measure.AngularVelocity
import org.littletonrobotics.junction.AutoLog

interface IndexerIO {
    @AutoLog
    open class IndexerInputs {
        @JvmField var wedgeVelocityRadPerSec: Double = 0.0

        @JvmField var wedgeAppliedVolts: Double = 0.0

        @JvmField var wedgeSupplyCurrentAmps: Double = 0.0

        @JvmField var wedgeStatorCurrentAmps: Double = 0.0

        @JvmField var wedgeTempCelsius: Double = 0.0

        @JvmField var floorVelocityRadPerSec: Double = 0.0

        @JvmField var floorAppliedVolts: Double = 0.0

        @JvmField var floorSupplyCurrentAmps: Double = 0.0

        @JvmField var floorStatorCurrentAmps: Double = 0.0

        @JvmField var floorTempCelsius: Double = 0.0

        @JvmField var topVelocityRadPerSec: Double = 0.0

        @JvmField var topAppliedVolts: Double = 0.0

        @JvmField var topSupplyCurrentAmps: Double = 0.0

        @JvmField var topStatorCurrentAmps: Double = 0.0

        @JvmField var topTempCelsius: Double = 0.0
    }

    fun setFloorSpeed(floorSurfaceSpeed: AngularVelocity) {}

    fun setWedgeSpeed(wedgeSurfaceSpeed: AngularVelocity) {}

    fun setTopSpeed(topSurfaceSpeed: AngularVelocity) {}

    fun setIndexerVelocity(
        topVel: AngularVelocity,
        sideVel: AngularVelocity,
        bottomVel: AngularVelocity
    ) {}

    fun simPeriodic() {}

    fun updateInputs(inputs: IndexerInputs) {}
}
