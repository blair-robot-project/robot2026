package frc.team449.subsystems.fuelsimulator

import edu.wpi.first.math.geometry.Translation3d
import frc.robot.FuelSim
import org.littletonrobotics.junction.AutoLog

class FuelIO {

    val fuelSim = FuelSim()
    var ballCount = 0
    var simIntaking = false
    var ballLinearLaunchSpeed = 0.0

    @AutoLog
    open class FuelIOInputs {
        @JvmField var ballPositions: Array<Translation3d> = emptyArray()

        @JvmField var ballCount: Int = 0

        @JvmField var simIntaking: Boolean = false

        @JvmField var ballLaunchSpeedMetersPerSecond: Double = 0.0
    }

    fun updateInputs(inputs: FuelIOInputs) {
        inputs.ballPositions = fuelSim.fuels
        inputs.ballCount = ballCount
        inputs.simIntaking = simIntaking
        inputs.ballLaunchSpeedMetersPerSecond = ballLinearLaunchSpeed
    }
}
