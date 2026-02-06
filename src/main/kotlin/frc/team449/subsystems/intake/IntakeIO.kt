package frc.team449.subsystems.intake

import com.ctre.phoenix6.controls.ControlRequest
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import org.littletonrobotics.junction.AutoLog

interface IntakeIO {
    @AutoLog
    open class IntakeIOInputs {
        @JvmField var pivotVoltage: Voltage = Volts.of(0.0)

        @JvmField var followerRollerVoltage: Voltage = Volts.of(0.0)

        @JvmField var leaderRollerVoltage: Voltage = Volts.of(0.0)

        @JvmField var pivotSupplyCurrent: Current = Amps.of(0.0)

        @JvmField var followerSupplyCurrent: Current = Amps.of(0.0)

        @JvmField var leaderSupplyCurrent: Current = Amps.of(0.0)

        @JvmField var pivotStatorCurrent: Current = Amps.of(0.0)

        @JvmField var followerStatorCurrent: Current = Amps.of(0.0)

        @JvmField var leaderStatorCurrent: Current = Amps.of(0.0)

        @JvmField var pivotAngle: Angle = Radians.of(0.0)

        @JvmField var pivotSpeed: AngularVelocity = RadiansPerSecond.of(0.0)
    }

    fun updateInputs(inputs: IntakeIOInputs) {}

    fun setPivotRequest( request: ControlRequest ) {}

    fun setRollerRequest( request: ControlRequest ) {}

    fun isNoteInsideIntake(): Boolean = false

    fun launchNote() {}

    fun simulationPeriodic() {}
}
