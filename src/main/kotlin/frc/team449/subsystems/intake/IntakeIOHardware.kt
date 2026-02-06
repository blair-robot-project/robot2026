package frc.team449.subsystems.intake
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.wpilibj.Alert
import frc.team449.util.PhoenixUtil.tryUntilOk
import com.ctre.phoenix6.controls.Follower
import edu.wpi.first.units.measure.Angle
import frc.team449.Constants.IntakeConstants


open class IntakeIOHardware : IntakeIO {
    val pivotMotor = TalonFX(IntakeConstants.PIVOT_MOTOR_ID)
    val pivotFollower = TalonFX(IntakeConstants.PIVOT_FOLLOWER_ID)
    val rollerMotor = TalonFX(IntakeConstants.ROLLER_MOTOR_ID)
    val rollerFollower = TalonFX(IntakeConstants.ROLLER_FOLLOWER_ID)


    val pivotMotorDisconnectedAlert = Alert("Pivot motor disconnected (ID ${IntakeConstants.PIVOT_MOTOR_ID})", Alert.AlertType.kError)
    val pivotFollowerDisconnectedAlert = Alert("Pivot motor disconnected (ID ${IntakeConstants.PIVOT_FOLLOWER_ID})", Alert.AlertType.kError)
    val rollerMotorDisconnectedAlert = Alert("Right Roller motor disconnected (ID ${IntakeConstants.ROLLER_MOTOR_ID})", Alert.AlertType.kError)
    val rollerFollowerDisconnectedAlert = Alert("Left Roller motor disconnected (ID ${IntakeConstants.ROLLER_FOLLOWER_ID})", Alert.AlertType.kError)

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
            rollerFollower.motorVoltage,
            rollerMotor.motorVoltage,
            pivotMotor.supplyCurrent,
            rollerFollower.supplyCurrent,
            rollerMotor.supplyCurrent,
            pivotMotor.statorCurrent,
            rollerFollower.statorCurrent,
            rollerMotor.statorCurrent,
            pivotMotor.position,
            pivotMotor.velocity
        )

        ParentDevice.optimizeBusUtilizationForAll(
            pivotMotor,
            rollerFollower,
            rollerMotor,
        )
    }

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        inputs.pivotVoltage = pivotMotor.motorVoltage.value
        inputs.followerRollerVoltage = rollerFollower.motorVoltage.value
        inputs.leaderRollerVoltage = rollerMotor.motorVoltage.value

        inputs.pivotSupplyCurrent = pivotMotor.supplyCurrent.value
        inputs.followerSupplyCurrent = rollerFollower.supplyCurrent.value
        inputs.leaderSupplyCurrent = rollerMotor.supplyCurrent.value

        inputs.pivotStatorCurrent = pivotMotor.statorCurrent.value
        inputs.followerStatorCurrent = rollerFollower.statorCurrent.value
        inputs.leaderStatorCurrent = rollerMotor.statorCurrent.value

        inputs.pivotAngle = pivotMotor.position.value
        inputs.pivotSpeed = pivotMotor.velocity.value

        pivotMotorDisconnectedAlert.set(!pivotMotor.isAlive)
        rollerFollowerDisconnectedAlert.set(!rollerFollower.isAlive)
        rollerMotorDisconnectedAlert.set(!rollerMotor.isAlive)
    }

    override fun setPivotRequest(request: ControlRequest) {
        pivotMotor.setControl(request)
    }

    override fun setPivotPosition(newPosition: Angle) {
        pivotMotor.setPosition(newPosition)
    }

    override fun setRollerRequest(request: ControlRequest) {
        rollerMotor.setControl(request)
    }

}
