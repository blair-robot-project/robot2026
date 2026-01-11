package frc.team449.util

import com.ctre.phoenix6.StatusCode
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj.Timer
import org.ironmaple.simulation.SimulatedArena
import java.util.function.Supplier

object PhoenixUtil {
    /** Attempts to run the command until no error is produced.  */
    fun tryUntilOk(maxAttempts: Int, command: Supplier<StatusCode>) {
        for (i in 0 until maxAttempts) {
            val error = command.get()
            if (error.isOK) break
        }
    }

    fun getSimulationOdometryTimeStamps(): DoubleArray {
        val odometryTimeStamps = DoubleArray(SimulatedArena.getSimulationSubTicksIn1Period())
        for (i in odometryTimeStamps.indices) {
            odometryTimeStamps[i] = (
                Timer.getFPGATimestamp() -
                    0.02 +
                    i * SimulatedArena.getSimulationDt().`in`(Seconds)
                )
        }

        return odometryTimeStamps
    }
}
