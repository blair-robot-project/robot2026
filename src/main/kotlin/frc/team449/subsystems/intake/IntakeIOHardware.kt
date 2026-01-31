package frc.team449.subsystems.intake

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants.IntakeConstants.DEFAULT_STATOR_LIMIT
import frc.team449.Constants.IntakeConstants.DEFAULT_SUPPLY_LIMIT
import frc.team449.Constants.IntakeConstants.LEFT_ROLLER_MOTOR_ID
import frc.team449.Constants.IntakeConstants.PIVOT_MOTOR_ID
import frc.team449.Constants.IntakeConstants.RIGHT_ROLLER_MOTOR_ID

class IntakeIOHardware : IntakeIO {
    private val pivotMotor = TalonFX(PIVOT_MOTOR_ID)
    private val leftRollerMotor = TalonFX(LEFT_ROLLER_MOTOR_ID)
    private val rightRollerMotor = TalonFX(RIGHT_ROLLER_MOTOR_ID)
    private var config = TalonFXConfiguration()

    private val pivotVoltage: StatusSignal<Voltage> = pivotMotor.motorVoltage
    private val leftRollerVoltage: StatusSignal<Voltage> = leftRollerMotor.motorVoltage
    private val rightRollerVoltage: StatusSignal<Voltage> = rightRollerMotor.motorVoltage

    private val pivotSupplyCurrent: StatusSignal<Current> = pivotMotor.supplyCurrent
    private val leftRollerSupplyCurrent: StatusSignal<Current> = leftRollerMotor.supplyCurrent
    private val rightRollerSupplyCurrent: StatusSignal<Current> = rightRollerMotor.supplyCurrent

    private val pivotStatorCurrent: StatusSignal<Current> = pivotMotor.statorCurrent
    private val leftRollerStatorCurrent: StatusSignal<Current> = leftRollerMotor.statorCurrent
    private val rightRollerStatorCurrent: StatusSignal<Current> = rightRollerMotor.statorCurrent

    private val pivotMotorConnected: Boolean =
        BaseStatusSignal.isAllGood(
            pivotVoltage,
            pivotSupplyCurrent,
            pivotSupplyCurrent,
        )
    private val leftRollerMotorConnected: Boolean =
        BaseStatusSignal.isAllGood(
            leftRollerVoltage,
            leftRollerSupplyCurrent,
            leftRollerSupplyCurrent,
        )
    private val rightRollerMotorConnected: Boolean =
        BaseStatusSignal.isAllGood(
            rightRollerVoltage,
            rightRollerSupplyCurrent,
            rightRollerSupplyCurrent,
        )

    private val pivotMotorDisconnectedAlert = Alert("Pivot motor disconnected (ID $PIVOT_MOTOR_ID)", Alert.AlertType.kError)
    private val leftRollerMotorDisconnectedAlert =
        Alert("Left Roller motor disconnected (ID $LEFT_ROLLER_MOTOR_ID)", Alert.AlertType.kError)
    private val rightRollerMotorDisconnectedAlert =
        Alert("right Roller motor disconnected (ID $RIGHT_ROLLER_MOTOR_ID)", Alert.AlertType.kError)

    private var targetPivotVoltage: Double = 0.0
    private var targetLeftVoltage: Double = 0.0
    private var targetRightVoltage: Double = 0.0

    init {
        val currentLimitConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(DEFAULT_SUPPLY_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(DEFAULT_STATOR_LIMIT)

        val motorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        config =
            TalonFXConfiguration()
                .withCurrentLimits(currentLimitConfigs)
                .withMotorOutput(motorOutput)

        pivotMotor.configurator.apply(config)
        leftRollerMotor.configurator.apply(config)
        rightRollerMotor.configurator.apply(config)

        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            pivotVoltage,
            leftRollerVoltage,
            rightRollerVoltage,
            pivotSupplyCurrent,
            leftRollerSupplyCurrent,
            rightRollerSupplyCurrent,
            pivotStatorCurrent,
            leftRollerStatorCurrent,
            rightRollerStatorCurrent,
        )

        ParentDevice.optimizeBusUtilizationForAll(
            pivotMotor,
            leftRollerMotor,
            rightRollerMotor,
        )
    }

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        inputs.currentPivotVoltage = pivotMotor.motorVoltage.value.`in`(Units.Volts)
        inputs.currentLeftRollerVoltage = leftRollerMotor.motorVoltage.value.`in`(Units.Volts)
        inputs.currentRightRollerVoltage = rightRollerMotor.motorVoltage.value.`in`(Units.Volts)

        inputs.targetPivotVoltage = targetPivotVoltage
        inputs.targetLeftRollerVoltage = targetLeftVoltage
        inputs.targetRightRollerVoltage = targetRightVoltage

        inputs.pivotSupplyCurrent = pivotMotor.supplyCurrent.value.`in`(Units.Amps)
        inputs.leftRollerSupplyCurrent = leftRollerMotor.supplyCurrent.value.`in`(Units.Amps)
        inputs.rightRollerSupplyCurrent = rightRollerMotor.supplyCurrent.value.`in`(Units.Amps)

        inputs.pivotStatorCurrent = pivotMotor.statorCurrent.value.`in`(Units.Amps)
        inputs.leftRollerStatorCurrent = leftRollerMotor.statorCurrent.value.`in`(Units.Amps)
        inputs.rightRollerStatorCurrent = rightRollerMotor.statorCurrent.value.`in`(Units.Amps)

        pivotMotorDisconnectedAlert.set(!pivotMotorConnected)
        leftRollerMotorDisconnectedAlert.set(!leftRollerMotorConnected)
        rightRollerMotorDisconnectedAlert.set(!rightRollerMotorConnected)
    }

    override fun setVoltage(
        pivotVoltage: Double,
        leftRollerVoltage: Double,
        rightRollerVoltage: Double,
    ) {
        targetPivotVoltage = pivotVoltage
        targetLeftVoltage = leftRollerVoltage
        targetRightVoltage = rightRollerVoltage

        pivotMotor.setVoltage(targetPivotVoltage)
        leftRollerMotor.setVoltage(targetLeftVoltage)
        rightRollerMotor.setVoltage(targetRightVoltage)
    }
}
