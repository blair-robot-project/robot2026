package frc.team449.subsystems.intake
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants.IntakeConstants
import frc.team449.util.PhoenixUtil.tryUntilOk

open class IntakeIOHardware : IntakeIO {
    val pivotMotor = TalonFX(IntakeConstants.PIVOT_MOTOR_ID)
    val pivotFollower = TalonFX(IntakeConstants.PIVOT_FOLLOWER_ID)
    val rollerMotor = TalonFX(IntakeConstants.ROLLER_MOTOR_ID)
    val rollerFollower = TalonFX(IntakeConstants.ROLLER_FOLLOWER_ID)

    val pivotMotorVoltage = pivotMotor.motorVoltage
    val pivotMotorSupplyCurrent = pivotMotor.supplyCurrent
    val pivotMotorStatorCurrent = pivotMotor.statorCurrent
    val pivotMotorPosition = pivotMotor.position
    val pivotMotorVelocity = pivotMotor.velocity
    val pivotMotorTemperature = pivotMotor.deviceTemp

    val pivotFollowerVoltage = pivotMotor.motorVoltage
    val pivotFollowerSupplyCurrent = pivotMotor.supplyCurrent
    val pivotFollowerStatorCurrent = pivotMotor.statorCurrent
    val pivotFollowerPosition = pivotMotor.position
    val pivotFollowerVelocity = pivotMotor.velocity
    val pivotFollowerTemperature = pivotMotor.deviceTemp

    val rollerMotorVoltage = pivotMotor.motorVoltage
    val rollerMotorSupplyCurrent = pivotMotor.supplyCurrent
    val rollerMotorStatorCurrent = pivotMotor.statorCurrent
    val rollerMotorVelocity = pivotMotor.velocity
    val rollerMotorTemperature = pivotMotor.deviceTemp

    val rollerFollowerVoltage = pivotMotor.motorVoltage
    val rollerFollowerSupplyCurrent = pivotMotor.supplyCurrent
    val rollerFollowerStatorCurrent = pivotMotor.statorCurrent
    val rollerFollowerVelocity = pivotMotor.velocity
    val rollerFollowerTemperature = pivotMotor.deviceTemp

    val pivotMotorDisconnectedAlert = Alert("Pivot motor disconnected (ID ${IntakeConstants.PIVOT_MOTOR_ID})", Alert.AlertType.kError)
    val pivotFollowerDisconnectedAlert = Alert("Pivot motor disconnected (ID ${IntakeConstants.PIVOT_FOLLOWER_ID})", Alert.AlertType.kError)
    val rollerMotorDisconnectedAlert =
        Alert("Right Roller motor disconnected (ID ${IntakeConstants.ROLLER_MOTOR_ID})", Alert.AlertType.kError)
    val rollerFollowerDisconnectedAlert =
        Alert("Left Roller motor disconnected (ID ${IntakeConstants.ROLLER_FOLLOWER_ID})", Alert.AlertType.kError)

    init {
        tryUntilOk(5) { pivotMotor.configurator.apply(IntakeConstants.PIVOT_CONFIG, 0.25) }
        tryUntilOk(5) { pivotFollower.configurator.apply(IntakeConstants.PIVOT_CONFIG, 0.25) }
        tryUntilOk(5) { rollerMotor.configurator.apply(IntakeConstants.ROLLER_CONFIG, 0.25) }
        tryUntilOk(5) { rollerFollower.configurator.apply(IntakeConstants.ROLLER_CONFIG, 0.25) }
        rollerFollower.setControl(Follower(rollerMotor.deviceID, IntakeConstants.ROLLER_FOLLOWER_ALIGNMENT))
        pivotFollower.setControl(Follower(pivotMotor.deviceID, IntakeConstants.PIVOT_FOLLOWER_ALIGNMENT))

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            pivotMotorVoltage,
            pivotMotorSupplyCurrent,
            pivotMotorStatorCurrent,
            pivotMotorPosition,
            pivotMotorVelocity,
            pivotMotorTemperature,
            pivotFollowerVoltage,
            pivotFollowerSupplyCurrent,
            pivotFollowerStatorCurrent,
            pivotFollowerPosition,
            pivotFollowerVelocity,
            pivotFollowerTemperature,
            rollerMotorVoltage,
            rollerMotorSupplyCurrent,
            rollerMotorStatorCurrent,
            rollerMotorVelocity,
            rollerMotorTemperature,
            rollerFollowerVoltage,
            rollerFollowerSupplyCurrent,
            rollerFollowerStatorCurrent,
            rollerFollowerVelocity,
            rollerFollowerTemperature
        )

        ParentDevice.optimizeBusUtilizationForAll(
            pivotMotor,
            pivotFollower,
            rollerFollower,
            rollerMotor,
        )
    }

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        BaseStatusSignal.refreshAll(
            pivotMotorVoltage,
            pivotMotorSupplyCurrent,
            pivotMotorStatorCurrent,
            pivotMotorPosition,
            pivotMotorVelocity,
            pivotMotorTemperature,
            pivotFollowerVoltage,
            pivotFollowerSupplyCurrent,
            pivotFollowerStatorCurrent,
            pivotFollowerPosition,
            pivotFollowerVelocity,
            pivotFollowerTemperature,
            rollerMotorVoltage,
            rollerMotorSupplyCurrent,
            rollerMotorStatorCurrent,
            rollerMotorVelocity,
            rollerMotorTemperature,
            rollerFollowerVoltage,
            rollerFollowerSupplyCurrent,
            rollerFollowerStatorCurrent,
            rollerFollowerVelocity,
            rollerFollowerTemperature
        )

        inputs.pivotMotorControlMode = pivotMotor.controlMode.name
        inputs.pivotMotorVoltage = pivotMotorVoltage.value
        inputs.pivotMotorSupplyCurrent = pivotMotorSupplyCurrent.value
        inputs.pivotMotorStatorCurrent = pivotMotorStatorCurrent.value
        inputs.pivotMotorPosition = pivotMotorPosition.value
        inputs.pivotMotorVelocity = pivotMotorVelocity.value
        inputs.pivotMotorTemperature = pivotMotorTemperature.value

        inputs.pivotFollowerVoltage = pivotFollowerVoltage.value
        inputs.pivotFollowerSupplyCurrent = pivotFollowerSupplyCurrent.value
        inputs.pivotFollowerStatorCurrent = pivotFollowerStatorCurrent.value
        inputs.pivotFollowerPosition = pivotFollowerPosition.value
        inputs.pivotFollowerVelocity = pivotFollowerVelocity.value
        inputs.pivotFollowerTemperature = pivotFollowerTemperature.value

        inputs.rollerMotorControlMode = rollerMotor.controlMode.name
        inputs.rollerMotorVoltage = rollerMotorVoltage.value
        inputs.rollerMotorSupplyCurrent = rollerMotorSupplyCurrent.value
        inputs.rollerMotorStatorCurrent = rollerMotorStatorCurrent.value
        inputs.rollerMotorVelocity = rollerMotorVelocity.value
        inputs.rollerMotorTemperature = rollerMotorTemperature.value

        inputs.rollerFollowerVoltage = rollerFollowerVoltage.value
        inputs.rollerFollowerSupplyCurrent = rollerFollowerSupplyCurrent.value
        inputs.rollerFollowerStatorCurrent = rollerFollowerStatorCurrent.value
        inputs.rollerFollowerVelocity = rollerFollowerVelocity.value
        inputs.rollerFollowerTemperature = rollerFollowerTemperature.value

        pivotMotorDisconnectedAlert.set(!pivotMotor.isAlive)
        pivotFollowerDisconnectedAlert.set(!pivotFollower.isAlive)
        rollerFollowerDisconnectedAlert.set(!rollerFollower.isAlive)
        rollerMotorDisconnectedAlert.set(!rollerMotor.isAlive)
    }

    override fun setPivotRequest(request: ControlRequest) {
        pivotMotor.setControl(request)
    }

    override fun setPivotPosition(position: Angle) {
        pivotMotor.setControl(PositionVoltage(position).withSlot(0))
    }

    override fun setRollerVelocity(velocity: AngularVelocity) {
        rollerMotor.setControl(VelocityVoltage(velocity).withSlot(0))
    }

    override fun setRollerRequest(request: ControlRequest) {
        rollerMotor.setControl(request)
    }

    override fun resetPivotPosition(position: Angle) {
        pivotMotor.setPosition(position)
    }
}
