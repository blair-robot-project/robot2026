package frc.team449.subsystems.shooter
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import org.littletonrobotics.junction.AutoLog

interface ShooterIO {
    @AutoLog
    open class ShooterIOInputs {
        @JvmField var leftTopLeaderConnected: Boolean = false

        @JvmField var leftTopLeaderAppliedVolts: Double = 0.0

        @JvmField var leftTopLeaderVelocityRadsPerSec: Double = 0.0

        @JvmField var leftTopLeaderSupplyCurrentAmps: Double = 0.0

        @JvmField var leftTopLeaderStatorCurrentAmps: Double = 0.0

        @JvmField var leftTopLeaderTempCelsius: Double = 0.0

        @JvmField var leftBottomFollowerConnected: Boolean = false

        @JvmField var leftBottomFollowerAppliedVolts: Double = 0.0

        @JvmField var leftBottomFollowerVelocityRadsPerSec: Double = 0.0

        @JvmField var leftBottomFollowerSupplyCurrentAmps: Double = 0.0

        @JvmField var leftBottomFollowerStatorCurrentAmps: Double = 0.0

        @JvmField var leftBottomFollowerTempCelsius: Double = 0.0

        @JvmField var rightTopFollowerConnected: Boolean = false

        @JvmField var rightTopFollowerAppliedVolts: Double = 0.0

        @JvmField var rightTopFollowerVelocityRadsPerSec: Double = 0.0

        @JvmField var rightTopFollowerSupplyCurrentAmps: Double = 0.0

        @JvmField var rightTopFollowerStatorCurrentAmps: Double = 0.0

        @JvmField var rightTopFollowerTempCelsius: Double = 0.0

        @JvmField var rightBottomFollowerConnected: Boolean = false

        @JvmField var rightBottomFollowerAppliedVolts: Double = 0.0

        @JvmField var rightBottomFollowerVelocityRadsPerSec: Double = 0.0

        @JvmField var rightBottomFollowerSupplyCurrentAmps: Double = 0.0

        @JvmField var rightBottomFollowerStatorCurrentAmps: Double = 0.0

        @JvmField var rightBottomFollowerTempCelsius: Double = 0.0

        @JvmField var hoodConnected: Boolean = false

        @JvmField var hoodAppliedVolts: Double = 0.0

        @JvmField var hoodAngleRad: Double = 0.0

        @JvmField var hoodVelocityRadPerSec: Double = 0.0

        @JvmField var hoodSupplyCurrentAmps: Double = 0.0

        @JvmField var hoodStatorCurrentAmps: Double = 0.0

        @JvmField var hoodTempCelsius: Double = 0.0
    }

    fun updateInputs(inputs: ShooterIOInputs) {}

    fun setFlywheelVoltage(flywheelVolts: Double) {}

    fun setFlywheelVelocity(flywheelVelocity: AngularVelocity) {}

    fun setHoodVoltage(hoodVolts: Double) {}

    fun setHoodAngle(hoodAngle: Angle) {}

    fun resetHoodAngle(hoodAngle: Angle) {}
}
