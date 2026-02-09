package frc.team449.subsystems.shooter

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.simulation.*
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import frc.team449.Constants.ShooterConstants.FLYWHEEL_GEARING
import frc.team449.Constants.ShooterConstants.FLYWHEEL_KD
import frc.team449.Constants.ShooterConstants.FLYWHEEL_KI
import frc.team449.Constants.ShooterConstants.FLYWHEEL_KP
import frc.team449.Constants.ShooterConstants.FLYWHEEL_KS
import frc.team449.Constants.ShooterConstants.FLYWHEEL_STATOR_LIM
import frc.team449.Constants.ShooterConstants.FLYWHEEL_SUPPLY_LIM
import frc.team449.Constants.ShooterConstants.HOOD_GEARING
import frc.team449.Constants.ShooterConstants.HOOD_LENGTH
import frc.team449.Constants.ShooterConstants.HOOD_MAX_ANGLE
import frc.team449.Constants.ShooterConstants.HOOD_MIN_ANGLE
import frc.team449.Constants.ShooterConstants.HOOD_MOMENT_OF_INERTIA
import frc.team449.Constants.ShooterConstants.HOOD_MOTOR_ID
import frc.team449.Constants.ShooterConstants.HOOD_KD
import frc.team449.Constants.ShooterConstants.HOOD_KG
import frc.team449.Constants.ShooterConstants.HOOD_KI
import frc.team449.Constants.ShooterConstants.HOOD_KP
import frc.team449.Constants.ShooterConstants.HOOD_KS
import frc.team449.Constants.ShooterConstants.HOOD_KV
import frc.team449.Constants.ShooterConstants.HOOD_STATOR_LIM
import frc.team449.Constants.ShooterConstants.HOOD_SUPPLY_LIM
import frc.team449.Constants.ShooterConstants.HOOD_TOLERANCE
import frc.team449.Constants.ShooterConstants.LEFT_FLYWHEEL_LEADER_ID
import frc.team449.Constants.ShooterConstants.RIGHT_FLYWHEEL_LEADER_ID
import kotlin.math.abs
import kotlin.math.pow

class ShooterIOSim : ShooterIO {

    private val hoodGearbox: DCMotor = DCMotor.getKrakenX60Foc(1)

    private val hoodMotor: TalonFX = TalonFX(HOOD_MOTOR_ID)

    private val mech: Mechanism2d = Mechanism2d(3.0, 3.0)
    private val mechRoot: MechanismRoot2d = mech.getRoot("shooter hood", 1.5, 0.5)

    private val hoodMechanism: MechanismLigament2d = mechRoot.append(
        MechanismLigament2d(
            "shooter hood",
            0.5,
            HOOD_MIN_ANGLE.`in`(Radians),
            6.0,
            Color8Bit(Color.kCyan)
        )
    )

    private val hoodSim: SingleJointedArmSim = SingleJointedArmSim(
        hoodGearbox,
        HOOD_GEARING,
        HOOD_MOMENT_OF_INERTIA,
        HOOD_LENGTH,
        HOOD_MIN_ANGLE.`in`(Radians),
        HOOD_MAX_ANGLE.`in`(Radians),
        true,
        HOOD_MIN_ANGLE.`in`(Radians),
        0.0,
        0.0
    )

    private val flywheelGearbox: DCMotor = DCMotor.getKrakenX60Foc(1)
    private val leftFlywheelMotor: TalonFX = TalonFX(LEFT_FLYWHEEL_LEADER_ID)
    private val rightFlywheelMotor: TalonFX = TalonFX(RIGHT_FLYWHEEL_LEADER_ID)

    // 1/2 MR^2
    val flywheelMomentOfInertia: Double = 0.5 * Units.lbsToKilograms(1.5) * Units.inchesToMeters(4.0).pow(2.0)

    private val flywheelPlant: LinearSystem<N1, N1, N1> =
        LinearSystemId.createFlywheelSystem(flywheelGearbox, flywheelMomentOfInertia, FLYWHEEL_GEARING)

    private val flywheelSim: FlywheelSim = FlywheelSim(flywheelPlant, flywheelGearbox, 0.0)

    init {
        SmartDashboard.putData("Shooter Mech2d", mech)
        hoodMotor.setPosition(HOOD_MIN_ANGLE)

        val flywheelCurrentLimitConfigs: CurrentLimitsConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimit(FLYWHEEL_SUPPLY_LIM)
                .withStatorCurrentLimit(FLYWHEEL_STATOR_LIM)

        val hoodCurrentLimitConfigs: CurrentLimitsConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimit(HOOD_SUPPLY_LIM)
                .withStatorCurrentLimit(HOOD_STATOR_LIM)

        val flywheelMotorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        val hoodMotorOutput =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive)

        val flywheelFeedback =
            FeedbackConfigs()
                .withSensorToMechanismRatio(FLYWHEEL_GEARING)

        val hoodFeedback =
            FeedbackConfigs()
                .withSensorToMechanismRatio(HOOD_GEARING)

        val flywheelSlot0Configs =
            Slot0Configs()
                .withKP(FLYWHEEL_KP)
                .withKI(FLYWHEEL_KI)
                .withKD(FLYWHEEL_KD)
                .withKS(FLYWHEEL_KS)

        val hoodSlot0Configs =
            Slot0Configs()
                .withKP(HOOD_KP)
                .withKI(HOOD_KI)
                .withKD(HOOD_KD)
                .withKS(HOOD_KS)
                .withKG(HOOD_KG)
                .withKV(HOOD_KV)

        val flywheelConfig =
            TalonFXConfiguration()
                .withCurrentLimits(flywheelCurrentLimitConfigs)
                .withMotorOutput(flywheelMotorOutput)
                .withFeedback(flywheelFeedback)
                .withSlot0(flywheelSlot0Configs)

        leftFlywheelMotor.configurator.apply(flywheelConfig)
        rightFlywheelMotor.configurator.apply(flywheelConfig)

        val hoodConfig =
            TalonFXConfiguration()
                .withCurrentLimits(hoodCurrentLimitConfigs)
                .withMotorOutput(hoodMotorOutput)
                .withFeedback(hoodFeedback)
                .withSlot0(hoodSlot0Configs)

        hoodMotor.configurator.apply(hoodConfig)
    }

    override fun runFlywheelAtVelocity(velocity: AngularVelocity) {
        // set flywheel voltage
        println(velocity)
        rightFlywheelMotor.setControl(VelocityVoltage(velocity).withSlot(0))
        leftFlywheelMotor.setControl(VelocityVoltage(velocity).withSlot(0))
    }

    override fun setHoodPosition(angle: Angle) {
        println("setting hood angle to ${angle.`in`(Radians)}")
        hoodMotor.setControl(PositionVoltage(angle))
    }

    override fun atTolerance(): Boolean {
        return abs(hoodMotor.position.value.`in`(Radians) - hoodMotor.position.value.`in`(Radians)) < HOOD_TOLERANCE.`in`(Radians)
    }

    override fun simPeriodic() {
        // hood stuff
        hoodSim.setInput(hoodMotor.motorVoltage.value.`in`(Volts))
        hoodSim.update(0.020) // 20ms
        hoodMotor.setPosition(Radians.of(hoodSim.angleRads))

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(hoodSim.currentDrawAmps)
        )
        hoodMechanism.angle = Units.radiansToDegrees(hoodSim.angleRads)

        // flywheel stuff
        val flywheelVoltage = (rightFlywheelMotor.motorVoltage.value.`in`(Volts) + leftFlywheelMotor.motorVoltage.value.`in`(Volts)) / 2 //average the two voltages
        flywheelSim.setInput(flywheelVoltage)
        flywheelSim.update(0.020) // ms
    }

    override fun getHoodPosition(): Angle {
        return hoodMotor.position.value
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        inputs.leftVoltage = leftFlywheelMotor.motorVoltage.value.`in`(Volts)
        inputs.leftSupplyCurrent = leftFlywheelMotor.supplyCurrent.value.`in`(Amps)
        inputs.leftStatorCurrent = leftFlywheelMotor.statorCurrent.value.`in`(Amps)
        inputs.leftTemperature = leftFlywheelMotor.deviceTemp.value.`in`(Celsius)
        inputs.leftMotorIsConnected = leftFlywheelMotor.isAlive
        inputs.leftFollowerMotorIsConnected = leftFlywheelMotor.isAlive

        inputs.rightVoltage = rightFlywheelMotor.motorVoltage.value.`in`(Volts)
        inputs.rightSupplyCurrent = rightFlywheelMotor.supplyCurrent.value.`in`(Amps)
        inputs.rightStatorCurrent = rightFlywheelMotor.supplyCurrent.value.`in`(Amps)
        inputs.rightTemperature = rightFlywheelMotor.deviceTemp.value.`in`(Celsius)
        inputs.rightMotorIsConnected = rightFlywheelMotor.isAlive
        inputs.rightFollowerMotorIsConneted = rightFlywheelMotor.isAlive

        inputs.hoodVoltage = hoodMotor.motorVoltage.value.`in`(Volts)
        inputs.hoodSupplyCurrent = hoodMotor.supplyCurrent.value.`in`(Amps)
        inputs.hoodStatorCurrent = hoodMotor.statorCurrent.value.`in`(Amps)
        inputs.hoodTemperature = hoodMotor.deviceTemp.value.`in`(Celsius)
        inputs.hoodMotorIsConnected = hoodMotor.isAlive
        inputs.hoodCurrentPos = hoodSim.angleRads
        inputs.hoodTargetPos = hoodMotor.position.value.`in`(Radians)

        inputs.flywheelVelocity = flywheelSim.angularVelocity.`in`(RadiansPerSecond)
    }
}
