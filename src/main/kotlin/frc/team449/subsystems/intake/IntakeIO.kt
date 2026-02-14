package frc.team449.subsystems.intake

import com.ctre.phoenix6.controls.ControlRequest
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import org.littletonrobotics.junction.AutoLog

interface IntakeIO {
    @AutoLog
    open class IntakeIOInputs {
        @JvmField var pivotMotorControlMode: String = "None"

        @JvmField var pivotMotorVoltage: Voltage = Volts.of(0.0)

        @JvmField var pivotMotorSupplyCurrent: Current = Amps.of(0.0)

        @JvmField var pivotMotorStatorCurrent: Current = Amps.of(0.0)

        @JvmField var pivotMotorPosition: Angle = Radians.of(0.0)

        @JvmField var pivotMotorVelocity: AngularVelocity = RadiansPerSecond.of(0.0)

        @JvmField var pivotMotorTemperature: Temperature = Celsius.of(0.0)

        @JvmField var pivotFollowerVoltage: Voltage = Volts.of(0.0)

        @JvmField var pivotFollowerSupplyCurrent: Current = Amps.of(0.0)

        @JvmField var pivotFollowerStatorCurrent: Current = Amps.of(0.0)

        @JvmField var pivotFollowerPosition: Angle = Radians.of(0.0)

        @JvmField var pivotFollowerVelocity: AngularVelocity = RadiansPerSecond.of(0.0)

        @JvmField var pivotFollowerTemperature: Temperature = Celsius.of(0.0)

        @JvmField var rollerMotorControlMode: String = "None"

        @JvmField var rollerMotorVoltage: Voltage = Volts.of(0.0)

        @JvmField var rollerMotorSupplyCurrent: Current = Amps.of(0.0)

        @JvmField var rollerMotorStatorCurrent: Current = Amps.of(0.0)

        @JvmField var rollerMotorVelocity: AngularVelocity = RadiansPerSecond.of(0.0)

        @JvmField var rollerMotorTemperature: Temperature = Celsius.of(0.0)

        @JvmField var rollerFollowerVoltage: Voltage = Volts.of(0.0)

        @JvmField var rollerFollowerSupplyCurrent: Current = Amps.of(0.0)

        @JvmField var rollerFollowerStatorCurrent: Current = Amps.of(0.0)

        @JvmField var rollerFollowerVelocity: AngularVelocity = RadiansPerSecond.of(0.0)

        @JvmField var rollerFollowerTemperature: Temperature = Celsius.of(0.0)
    }

    fun updateInputs(inputs: IntakeIOInputs) {}

    fun setPivotRequest(request: ControlRequest) {}

    fun setPivotPosition(position: Angle) {}

    fun setRollerRequest(request: ControlRequest) {}

    fun setRollerVelocity(velocity: AngularVelocity) {}

    fun simulationPeriodic() {}

    fun resetPivotPosition(position: Angle) {}
}
