package frc.team449.subsystems.intake
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants.IntakeConstants
import frc.team449.util.PhoenixUtil.tryUntilOk

open class IntakeIOHardware : IntakeIO {
    val pivotMotor = TalonFX(IntakeConstants.PIVOT_MOTOR_ID)
    val pivotFollower = TalonFX(IntakeConstants.PIVOT_FOLLOWER_ID)
    val rollerMotor = TalonFX(IntakeConstants.ROLLER_MOTOR_ID)
    val rollerFollower = TalonFX(IntakeConstants.ROLLER_FOLLOWER_ID)

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
            pivotMotor.motorVoltage,
            pivotMotor.statorCurrent,
            pivotMotor.position,
            pivotMotor.velocity,
            pivotFollower.motorVoltage,
            pivotFollower.statorCurrent,
            pivotFollower.position,
            pivotFollower.velocity,
            rollerMotor.motorVoltage,
            rollerMotor.statorCurrent,
            rollerMotor.velocity,
            rollerFollower.motorVoltage,
            rollerFollower.statorCurrent,
            rollerFollower.velocity,
        )

        ParentDevice.optimizeBusUtilizationForAll(
            pivotMotor,
            pivotFollower,
            rollerFollower,
            rollerMotor,
        )
    }

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        inputs.pivotMotorVoltage = pivotMotor.motorVoltage.value
        inputs.pivotMotorSupplyCurrent = pivotMotor.supplyCurrent.value
        inputs.pivotMotorStatorCurrent = pivotMotor.statorCurrent.value
        inputs.pivotMotorPosition = pivotMotor.position.value
        inputs.pivotMotorVelocity = pivotMotor.velocity.value
        inputs.pivotMotorTemperature = pivotMotor.deviceTemp.value

        inputs.pivotFollowerVoltage = pivotFollower.motorVoltage.value
        inputs.pivotFollowerSupplyCurrent = pivotFollower.supplyCurrent.value
        inputs.pivotFollowerStatorCurrent = pivotFollower.statorCurrent.value
        inputs.pivotFollowerPosition = pivotFollower.position.value
        inputs.pivotFollowerVelocity = pivotFollower.velocity.value
        inputs.pivotFollowerTemperature = pivotFollower.deviceTemp.value

        inputs.rollerMotorVoltage = rollerMotor.motorVoltage.value
        inputs.rollerMotorSupplyCurrent = rollerMotor.supplyCurrent.value
        inputs.rollerMotorStatorCurrent = rollerMotor.statorCurrent.value
        inputs.rollerMotorVelocity = rollerMotor.velocity.value
        inputs.rollerMotorTemperature = rollerMotor.deviceTemp.value

        inputs.rollerFollowerVoltage = rollerFollower.motorVoltage.value
        inputs.rollerFollowerSupplyCurrent = rollerFollower.supplyCurrent.value
        inputs.rollerFollowerStatorCurrent = rollerFollower.statorCurrent.value
        inputs.rollerFollowerVelocity = rollerFollower.velocity.value
        inputs.rollerFollowerTemperature = rollerFollower.deviceTemp.value

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
