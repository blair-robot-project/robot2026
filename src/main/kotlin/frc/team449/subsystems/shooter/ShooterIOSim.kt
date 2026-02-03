package frc.team449.subsystems.shooter

import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.*
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import frc.team449.Constants.ShooterConstants.FLYWHEEL_GEARING
import frc.team449.Constants.ShooterConstants.HOOD_ANGLE_ENCODER_DISTANCE_PER_PULSE
import frc.team449.Constants.ShooterConstants.HOOD_LENGTH
import frc.team449.Constants.ShooterConstants.HOOD_MASS
import frc.team449.Constants.ShooterConstants.HOOD_MAX_ANGLE
import frc.team449.Constants.ShooterConstants.HOOD_MIN_ANGLE
import frc.team449.Constants.ShooterConstants.HOOD_MOTOR_GEARING
import frc.team449.Constants.ShooterConstants.HOOD_MOTOR_ID
import frc.team449.Constants.ShooterConstants.HOOD_SIM_GRAVITY
import frc.team449.Constants.ShooterConstants.HOOD_SIM_KA
import frc.team449.Constants.ShooterConstants.HOOD_SIM_KD
import frc.team449.Constants.ShooterConstants.HOOD_SIM_KG
import frc.team449.Constants.ShooterConstants.HOOD_SIM_KI
import frc.team449.Constants.ShooterConstants.HOOD_SIM_KP
import frc.team449.Constants.ShooterConstants.HOOD_SIM_KS
import frc.team449.Constants.ShooterConstants.HOOD_SIM_KV
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

    private val hoodPIDController: PIDController = PIDController(HOOD_SIM_KP, HOOD_SIM_KI, HOOD_SIM_KD)
    private val hoodFeedforward: ArmFeedforward = ArmFeedforward(HOOD_SIM_KS, HOOD_SIM_KG, HOOD_SIM_KV, HOOD_SIM_KA)

    private val hoodMechanism: MechanismLigament2d = mechRoot.append(
        MechanismLigament2d(
            "shooter hood",
            0.5,
            HOOD_MIN_ANGLE.`in`(Radians),
            6.0,
            Color8Bit(Color.kCyan)
        )
    )

    private var hoodSetpoint = HOOD_MIN_ANGLE

    private val hoodSim: SingleJointedArmSim = SingleJointedArmSim(
        hoodGearbox,
        HOOD_MOTOR_GEARING,
        SingleJointedArmSim.estimateMOI(HOOD_LENGTH, HOOD_MASS),
        HOOD_LENGTH,
        HOOD_MIN_ANGLE.`in`(Radians),
        HOOD_MAX_ANGLE.`in`(Radians),
        HOOD_SIM_GRAVITY,
        HOOD_MIN_ANGLE.`in`(Radians),
        HOOD_ANGLE_ENCODER_DISTANCE_PER_PULSE,
        0.0
    )

    private val flywheelGearbox: DCMotor = DCMotor.getKrakenX60Foc(1)
    private val leftFlywheelMotor: TalonFX = TalonFX(LEFT_FLYWHEEL_LEADER_ID)
    private val rightFlywheelMotor: TalonFX = TalonFX(RIGHT_FLYWHEEL_LEADER_ID)

    // 1/2 MR^2
    val flywheelMomentOfInertia: Double = 0.5 * Units.lbsToKilograms(1.5) * Units.inchesToMeters(4.0).pow(2.0)

    private val flywheelPlant: LinearSystem<N1, N1, N1> =
        LinearSystemId.createFlywheelSystem(flywheelGearbox, FLYWHEEL_GEARING, flywheelMomentOfInertia)

    private val flywheelSim: FlywheelSim = FlywheelSim(flywheelPlant, flywheelGearbox, 0.0)

    init {
        SmartDashboard.putData("Shooter Mech2d", mech)
        hoodPIDController.setPID(HOOD_SIM_KP, HOOD_SIM_KI, HOOD_SIM_KD)
        hoodMotor.setPosition(HOOD_MIN_ANGLE)
    }

    override fun runFlywheelAtVelocity(velocity: AngularVelocity) {
        // set flywheel voltage
        println("running flywheels")
        rightFlywheelMotor.setControl(VelocityVoltage(velocity))
        leftFlywheelMotor.setControl(VelocityVoltage(velocity))
    }

    override fun setHoodPosition(angle: Angle) {
        println("setting hood angle to ${angle.`in`(Radians)}")
        hoodSetpoint = angle
    }

    override fun atTolerance(): Boolean {
        return abs(hoodMotor.position.value.`in`(Radians) - hoodSetpoint.`in`(Radians)) < HOOD_TOLERANCE.`in`(Radians)
    }

    override fun simPeriodic() {
        // hood stuff
        val feedForwardVoltage = if (HOOD_SIM_GRAVITY) hoodFeedforward.calculate(hoodSetpoint.`in`(Radians), 0.0) else 0.0

        val pidVoltage = hoodPIDController.calculate(
            hoodMotor.position.value.`in`(Radians),
            hoodSetpoint.`in`(Radians)
        )
        val voltageOutput = pidVoltage + feedForwardVoltage
        hoodMotor.setVoltage(
            voltageOutput
        )
        hoodSim.setInput(voltageOutput)
        hoodSim.update(0.020) // 20ms

        hoodMotor.setPosition(Radians.of(hoodSim.angleRads))

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(hoodSim.currentDrawAmps)
        )
        hoodMechanism.angle = Units.radiansToDegrees(hoodSim.angleRads)

        // flywheel stuff
        val flywheelVoltage = RobotController.getBatteryVoltage() * (rightFlywheelMotor.get() * 2 + leftFlywheelMotor.get() * 2) // multiplying my 2 to account for the follower
        flywheelSim.setInput(flywheelVoltage)
        flywheelSim.update(0.020) // ms
    }

    override fun getHoodPosition(): Angle {
        return hoodMotor.position.value
    }
}
