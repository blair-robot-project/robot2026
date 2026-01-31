package frc.team449.subsystems.intake.roller

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import frc.team449.Constants.IntakeConstants.LEFT_ROLLER_MOTOR_ID
import frc.team449.Constants.IntakeConstants.RIGHT_ROLLER_MOTOR_ID
import frc.team449.Constants.IntakeConstants.ROLLER_STATOR_LIMIT
import frc.team449.Constants.IntakeConstants.ROLLER_SUPPLY_LIMIT

class RollerIOHardware : RollerIO {
    private val leftMotor = TalonFX(LEFT_ROLLER_MOTOR_ID) // x60
    private val rightMotor = TalonFX(RIGHT_ROLLER_MOTOR_ID) // x60
    private var config = TalonFXConfiguration()

    private val motorConnected: Boolean = true
    private val leftVelocity: StatusSignal<AngularVelocity>
    private val leftVoltage: StatusSignal<Voltage>
    private val leftSupplyCurrent: StatusSignal<Current>
    private val leftStatorCurrent: StatusSignal<Current>
    private val leftTemperature: StatusSignal<Temperature>

    private val rightVelocity: StatusSignal<AngularVelocity>
    private val rightVoltage: StatusSignal<Voltage>
    private val rightSupplyCurrent: StatusSignal<Current>
    private val rightStatorCurrent: StatusSignal<Current>
    private val rightTemperature: StatusSignal<Temperature>

    private val motorDisconnectedAlertLeft =
        Alert(
            "Left roller motor disconnected (ID $LEFT_ROLLER_MOTOR_ID)",
            Alert.AlertType.kError,
        )
    private val motorDisconnectedAlertRight =
        Alert(
            "Right roller motor disconnected (ID $RIGHT_ROLLER_MOTOR_ID)",
            Alert.AlertType.kError,
        )

    init {
        val currentLimitConfigs: CurrentLimitsConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(ROLLER_SUPPLY_LIMIT)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(ROLLER_STATOR_LIMIT)

        val motorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        config =
            TalonFXConfiguration()
                .withCurrentLimits(currentLimitConfigs)
                .withMotorOutput(motorOutput)

        leftMotor.configurator.apply(config)
        rightMotor.configurator.apply(config)

        leftVelocity = leftMotor.velocity
        leftVoltage = leftMotor.motorVoltage
        leftSupplyCurrent = leftMotor.supplyCurrent
        leftStatorCurrent = leftMotor.statorCurrent
        leftTemperature = leftMotor.deviceTemp

        rightVelocity = leftMotor.velocity
        rightVoltage = leftMotor.motorVoltage
        rightSupplyCurrent = leftMotor.supplyCurrent
        rightStatorCurrent = leftMotor.statorCurrent
        rightTemperature = leftMotor.deviceTemp

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            leftVoltage,
            leftVelocity,
            leftSupplyCurrent,
            leftStatorCurrent,
            leftTemperature,
            rightVelocity,
            rightVoltage,
            rightSupplyCurrent,
            rightStatorCurrent,
            rightTemperature,
        )

        ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor)
    }

    override fun updateInputs(rollerInputs: RollerIO.RollerIOInputs) {
        BaseStatusSignal.refreshAll(
            leftVelocity,
            leftVoltage,
            leftSupplyCurrent,
            leftStatorCurrent,
            leftTemperature,
            rightVelocity,
            rightVoltage,
            rightSupplyCurrent,
            rightStatorCurrent,
            rightTemperature,
        )

        // way redundant, maybe should make a func for this

        rollerInputs.leftMotor.voltage = leftVoltage.getValue().`in`(Units.Volts)
        rollerInputs.leftMotor.supplyCurrent = leftSupplyCurrent.getValue().`in`(Units.Amps)
        rollerInputs.leftMotor.statorCurrent = leftStatorCurrent.getValue().`in`(Units.Amps)
        rollerInputs.leftMotor.temperature = leftTemperature.getValue().`in`(Units.Celsius)
        rollerInputs.leftMotor.motorIsConnected =
            BaseStatusSignal.isAllGood(
                leftVelocity,
                leftVoltage,
                leftSupplyCurrent,
                leftStatorCurrent,
                leftTemperature,
            )

        rollerInputs.rightMotor.voltage = rightVoltage.getValue().`in`(Units.Volts)
        rollerInputs.rightMotor.supplyCurrent = rightSupplyCurrent.getValue().`in`(Units.Amps)
        rollerInputs.rightMotor.statorCurrent = rightStatorCurrent.getValue().`in`(Units.Amps)
        rollerInputs.rightMotor.temperature = rightTemperature.getValue().`in`(Units.Celsius)
        rollerInputs.rightMotor.motorIsConnected =
            BaseStatusSignal.isAllGood(
                leftVelocity,
                leftVoltage,
                leftSupplyCurrent,
                leftStatorCurrent,
                leftTemperature,
            )

        motorDisconnectedAlertLeft.set(!motorConnected)
        motorDisconnectedAlertRight.set(!motorConnected)
    }

    override fun setVelocity(velocity: Double) {
        leftMotor.setControl(
            VelocityVoltage(velocity)
                .withSlot(0),
        )
        rightMotor.setControl(
            VelocityVoltage(velocity)
                .withSlot(0),
        )
    }

    fun stop() {
        leftMotor.stopMotor()
        rightMotor.stopMotor()
    }
}
