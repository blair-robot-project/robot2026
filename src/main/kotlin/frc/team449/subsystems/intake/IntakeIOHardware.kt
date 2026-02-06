package frc.team449.subsystems.intake
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.ControlRequest
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.Alert
import frc.team449.util.PhoenixUtil.tryUntilOk
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.signals.MotorAlignmentValue
import frc.team449.Constants.IntakeConstants


open class IntakeIOHardware : IntakeIO {
    val pivotMotor = TalonFX(IntakeConstants.PIVOT_MOTOR_ID)
    val followerRollerMotor = TalonFX(IntakeConstants.LEFT_ROLLER_MOTOR_ID)
    val leaderRollerMotor = TalonFX(IntakeConstants.RIGHT_ROLLER_MOTOR_ID)

    private val pivotMotorDisconnectedAlert = Alert("Pivot motor disconnected (ID $IntakeConstants.PIVOT_MOTOR_ID)", Alert.AlertType.kError)
    private val followerMotorDisconnectedAlert = Alert("Left Roller motor disconnected (ID $IntakeConstants.LEFT_ROLLER_MOTOR_ID)", Alert.AlertType.kError)
    private val leaderMotorDisconnectedAlert = Alert("Right Roller motor disconnected (ID $IntakeConstants.RIGHT_ROLLER_MOTOR_ID)", Alert.AlertType.kError)

    init {
        tryUntilOk(5) { pivotMotor.configurator.apply(IntakeConstants.pivotConfig, 0.25) }
        tryUntilOk(5) { followerRollerMotor.configurator.apply(IntakeConstants.followerRollerConfig, 0.25) }
        tryUntilOk(5) { leaderRollerMotor.configurator.apply(IntakeConstants.leaderRollerConfig, 0.25) }
        followerRollerMotor.setControl(Follower(leaderRollerMotor.deviceID, IntakeConstants.followerMotorAlignment))

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            pivotMotor.motorVoltage,
            followerRollerMotor.motorVoltage,
            leaderRollerMotor.motorVoltage,
            pivotMotor.supplyCurrent,
            followerRollerMotor.supplyCurrent,
            leaderRollerMotor.supplyCurrent,
            pivotMotor.statorCurrent,
            followerRollerMotor.statorCurrent,
            leaderRollerMotor.statorCurrent,
            pivotMotor.position,
            pivotMotor.velocity
        )

        ParentDevice.optimizeBusUtilizationForAll(
            pivotMotor,
            followerRollerMotor,
            leaderRollerMotor,
        )
    }

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        inputs.pivotVoltage = pivotMotor.motorVoltage.value
        inputs.followerRollerVoltage = followerRollerMotor.motorVoltage.value
        inputs.leaderRollerVoltage = leaderRollerMotor.motorVoltage.value

        inputs.pivotSupplyCurrent = pivotMotor.supplyCurrent.value
        inputs.followerSupplyCurrent = followerRollerMotor.supplyCurrent.value
        inputs.leaderSupplyCurrent = leaderRollerMotor.supplyCurrent.value

        inputs.pivotStatorCurrent = pivotMotor.statorCurrent.value
        inputs.followerStatorCurrent = followerRollerMotor.statorCurrent.value
        inputs.leaderStatorCurrent = leaderRollerMotor.statorCurrent.value

        inputs.pivotAngle = pivotMotor.position.value
        inputs.pivotSpeed = pivotMotor.velocity.value

        pivotMotorDisconnectedAlert.set(!pivotMotor.isAlive)
        followerMotorDisconnectedAlert.set(!followerRollerMotor.isAlive)
        leaderMotorDisconnectedAlert.set(!leaderRollerMotor.isAlive)
    }

    override fun setPivotRequest(request: ControlRequest) {
        pivotMotor.setControl(request)
    }

    override fun setRollerRequest(request: ControlRequest) {
        leaderRollerMotor.setControl(request)
    }

}
