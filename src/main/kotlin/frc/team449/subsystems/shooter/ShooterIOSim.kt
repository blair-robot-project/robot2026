package frc.team449.subsystems.shooter

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.filter.Debouncer
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
import edu.wpi.first.units.measure.Current
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
import frc.team449.Constants.ShooterConstants.FLYWHEEL_KV
import frc.team449.Constants.ShooterConstants.FLYWHEEL_MOI
import frc.team449.Constants.ShooterConstants.FLYWHEEL_STATOR_LIM
import frc.team449.Constants.ShooterConstants.FLYWHEEL_SUPPLY_LIM
import frc.team449.Constants.ShooterConstants.HOOD_GEARING
import frc.team449.Constants.ShooterConstants.HOOD_KD
import frc.team449.Constants.ShooterConstants.HOOD_KG
import frc.team449.Constants.ShooterConstants.HOOD_KI
import frc.team449.Constants.ShooterConstants.HOOD_KP
import frc.team449.Constants.ShooterConstants.HOOD_KS
import frc.team449.Constants.ShooterConstants.HOOD_KV
import frc.team449.Constants.ShooterConstants.HOOD_LENGTH
import frc.team449.Constants.ShooterConstants.HOOD_MAX_ANGLE
import frc.team449.Constants.ShooterConstants.HOOD_MIN_ANGLE
import frc.team449.Constants.ShooterConstants.HOOD_MOMENT_OF_INERTIA
import frc.team449.Constants.ShooterConstants.HOOD_MOTOR_ID
import frc.team449.Constants.ShooterConstants.HOOD_STATOR_LIM
import frc.team449.Constants.ShooterConstants.HOOD_SUPPLY_LIM
import frc.team449.Constants.ShooterConstants.HOOD_TOLERANCE
import frc.team449.Constants.ShooterConstants.LEFT_FLYWHEEL_FOLLOWER_ID
import frc.team449.Constants.ShooterConstants.LEFT_FLYWHEEL_LEADER_ID
import frc.team449.Constants.ShooterConstants.RIGHT_FLYWHEEL_FOLLOWER_ID
import frc.team449.Constants.ShooterConstants.RIGHT_FLYWHEEL_LEADER_ID
import frc.team449.Constants.ShooterConstants.TOLERANCE_DEBOUNCE_TIME
import frc.team449.Constants.ShooterConstants.TOLERANCE_DEBOUNCE_TYPE
import kotlin.math.abs

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
    private val toleranceDebouncer: Debouncer = Debouncer(TOLERANCE_DEBOUNCE_TIME, TOLERANCE_DEBOUNCE_TYPE)

    private val leftFlywheelGearbox: DCMotor = DCMotor.getKrakenX60Foc(2)
    private val rightFlywheelGearbox: DCMotor = DCMotor.getKrakenX60Foc(2)

    private val leftLeaderMotor: TalonFX = TalonFX(LEFT_FLYWHEEL_LEADER_ID)
    private val leftFollowerMotor: TalonFX = TalonFX(LEFT_FLYWHEEL_FOLLOWER_ID)

    private val rightLeaderMotor: TalonFX = TalonFX(RIGHT_FLYWHEEL_LEADER_ID)
    private val rightFollowerMotor: TalonFX = TalonFX(RIGHT_FLYWHEEL_FOLLOWER_ID)

    // 1/2 MR^2
    val leftFlywheelMomentOfInertia: Double = FLYWHEEL_MOI
    val rightFlywheelMomentOfInertia: Double = FLYWHEEL_MOI

    private val leftFlywheelPlant: LinearSystem<N1, N1, N1> =
        LinearSystemId.createFlywheelSystem(leftFlywheelGearbox, leftFlywheelMomentOfInertia, FLYWHEEL_GEARING)

    private val rightFlywheelPlant: LinearSystem<N1, N1, N1> =
        LinearSystemId.createFlywheelSystem(rightFlywheelGearbox, rightFlywheelMomentOfInertia, FLYWHEEL_GEARING)

    private val leftFlywheelSim: FlywheelSim = FlywheelSim(leftFlywheelPlant, leftFlywheelGearbox, 0.0)
    private val rightFlywheelSim: FlywheelSim = FlywheelSim(rightFlywheelPlant, rightFlywheelGearbox, 0.0)

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
                .withKV(FLYWHEEL_KV)

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

        leftLeaderMotor.configurator.apply(flywheelConfig)
        leftFollowerMotor.configurator.apply(flywheelConfig)
        rightLeaderMotor.configurator.apply(flywheelConfig)
        rightFollowerMotor.configurator.apply(flywheelConfig)

        leftFollowerMotor.setControl(Follower(leftLeaderMotor.deviceID, MotorAlignmentValue.Aligned))
        rightFollowerMotor.setControl(Follower(rightLeaderMotor.deviceID, MotorAlignmentValue.Aligned))

        val hoodConfig =
            TalonFXConfiguration()
                .withCurrentLimits(hoodCurrentLimitConfigs)
                .withMotorOutput(hoodMotorOutput)
                .withFeedback(hoodFeedback)
                .withSlot0(hoodSlot0Configs)

        hoodMotor.configurator.apply(hoodConfig)
    }

    override fun setFlywheelVelocity(velocity: AngularVelocity) {
        leftLeaderMotor.setControl(VelocityVoltage(velocity).withSlot(0))
        rightLeaderMotor.setControl(VelocityVoltage(velocity).withSlot(0))
    }

    override fun setHoodPosition(angle: Angle) {
        println("setting hood angle to ${angle.`in`(Radians)}")
        hoodMotor.setControl(PositionVoltage(angle))
    }

    override fun inTolerance(): Boolean {
        return toleranceDebouncer.calculate(abs(hoodMotor.closedLoopError.value) < HOOD_TOLERANCE.`in`(Radians))
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
        val rightFlywheelVoltage = (rightLeaderMotor.motorVoltage.value.`in`(Volts) + rightFollowerMotor.motorVoltage.value.`in`(Volts))
        leftFlywheelSim.setInput(rightFlywheelVoltage)
        leftFlywheelSim.update(0.020) // ms

        val leftFlywheelVoltage = (leftLeaderMotor.motorVoltage.value.`in`(Volts) + leftFollowerMotor.motorVoltage.value.`in`(Volts))
        rightFlywheelSim.setInput(leftFlywheelVoltage)
        rightFlywheelSim.update(0.020) // ms
    }

    override fun getHoodPosition(): Angle {
        return hoodMotor.position.value
    }

    override fun setHoodVoltage(voltage: Double) {
        hoodMotor.setVoltage(voltage)
    }

    override fun stopHoodVoltage() {
        hoodMotor.setVoltage(0.0)
    }

    override fun getHoodStatorCurrent(): Current {
        return hoodMotor.statorCurrent.value
    }

    override fun resetHoodPosition() {
        hoodMotor.setPosition(HOOD_MIN_ANGLE)
    }

    override fun setFlywheelVoltage(voltage: Double) {
        leftLeaderMotor.setVoltage(voltage)
        rightLeaderMotor.setVoltage(voltage)
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        inputs.leftVoltage = leftLeaderMotor.motorVoltage.value.`in`(Volts)
        inputs.leftSupplyCurrent = leftLeaderMotor.supplyCurrent.value.`in`(Amps)
        inputs.leftStatorCurrent = leftLeaderMotor.statorCurrent.value.`in`(Amps)
        inputs.leftTemperature = leftLeaderMotor.deviceTemp.value.`in`(Celsius)
        inputs.leftMotorIsConnected = leftLeaderMotor.isAlive
        inputs.leftFollowerMotorIsConnected = leftLeaderMotor.isAlive

        inputs.rightVoltage = rightLeaderMotor.motorVoltage.value.`in`(Volts)
        inputs.rightSupplyCurrent = rightLeaderMotor.supplyCurrent.value.`in`(Amps)
        inputs.rightStatorCurrent = rightLeaderMotor.supplyCurrent.value.`in`(Amps)
        inputs.rightTemperature = rightLeaderMotor.deviceTemp.value.`in`(Celsius)
        inputs.rightMotorIsConnected = rightLeaderMotor.isAlive
        inputs.rightFollowerMotorIsConneted = rightLeaderMotor.isAlive

        inputs.hoodVoltage = hoodMotor.motorVoltage.value.`in`(Volts)
        inputs.hoodSupplyCurrent = hoodMotor.supplyCurrent.value.`in`(Amps)
        inputs.hoodStatorCurrent = hoodMotor.statorCurrent.value.`in`(Amps)
        inputs.hoodTemperature = hoodMotor.deviceTemp.value.`in`(Celsius)
        inputs.hoodMotorIsConnected = hoodMotor.isAlive
        inputs.hoodCurrentPos = hoodSim.angleRads
        inputs.hoodTargetPos = hoodMotor.position.value.`in`(Radians)

        inputs.leftFlywheelVelocity = leftFlywheelSim.angularVelocity.`in`(RadiansPerSecond)
        inputs.rightFlywheelVelocity = rightFlywheelSim.angularVelocity.`in`(RadiansPerSecond)
    }
}
