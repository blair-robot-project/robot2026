package frc.team449.util

import edu.wpi.first.math.MathUtil

object MathExtensions {
    fun Double.slewTowards(target: Double, rate: Double, dt: Double = 0.02): Double {
        val maxChange = rate * dt
        val error = target - this

        return this + MathUtil.clamp(error, -maxChange, maxChange)
    }
}
