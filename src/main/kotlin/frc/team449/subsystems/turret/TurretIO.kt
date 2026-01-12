package frc.team449.subsystems.turret

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.AutoLog

interface TurretIO {
    @AutoLog
    open class TurretIOInputs {
        @JvmField var position: Angle = Radians.of(0.0) // relative to robot
        @JvmField var velocity: AngularVelocity = RadiansPerSecond.of(0.0)
        @JvmField var voltage: Voltage = Volts.of(0.0)
        @JvmField var current: Current = Amps.of(0.0)
    }

    fun updateInputs(inputs: TurretIOInputs) {}

    fun requestPosition(position: Angle)

    fun requestVoltage(voltage: Voltage)
}