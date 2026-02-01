package frc.team449.subsystems.shooter
import com.ctre.phoenix6.hardware.TalonFX
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Encoder
import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.EncoderSim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import frc.team449.Constants
import frc.team449.Constants.ShooterConstants.HOOD_ANGLE_ENCODER_DISTANCE_PER_PULSE
import frc.team449.Constants.ShooterConstants.HOOD_ENCODER_A_CHANNEL
import frc.team449.Constants.ShooterConstants.HOOD_ENCODER_B_CHANNEL
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
import kotlin.math.abs

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
    private val hoodEncoder: Encoder = Encoder(HOOD_ENCODER_A_CHANNEL, HOOD_ENCODER_B_CHANNEL)
    private val hoodEncoderSim: EncoderSim = EncoderSim(hoodEncoder)

    private val flywheelGearbox: DCMotor = DCMotor.getKrakenX60Foc(4)

    init {
        SmartDashboard.putData("Shooter Mech2d", mech)
        hoodEncoder.distancePerPulse = HOOD_ANGLE_ENCODER_DISTANCE_PER_PULSE
        hoodPIDController.setPID(HOOD_SIM_KP, HOOD_SIM_KI, HOOD_SIM_KD)
    }

    override fun run(voltage: Double): Command {
        // set flywheel voltage
        return runOnce({
            print("running flywheels")
        })
    }

    override fun stop(): Command {
        // stop flywheel voltage
        return runOnce({
            print("stopping flywheels")
        })
    }

    override fun setHood(angle: Angle): Command {
        return runOnce({
            println("setting hood angle to ${angle.`in`(Radians)}")
            hoodSetpoint = angle
        })
    }

    override fun holdHood(): Command {
        return runOnce({
            hoodSetpoint = Radians.of(hoodSim.angleRads)
        })
    }

    override fun atTolerance(): Boolean {
        return abs(hoodEncoder.distance - hoodSetpoint.`in`(Radians)) < Constants.ShooterConstants.HOOD_TOLERANCE.`in`(Radians)
    }

    override fun simPeriodic() {
        val feedForwardVoltage = if (HOOD_SIM_GRAVITY) hoodFeedforward.calculate(hoodSetpoint.`in`(Radians), 0.0) else 0.0
        val pidVoltage = hoodPIDController.calculate(
            hoodEncoder.distance,
            hoodSetpoint.`in`(Radians)
        )
        val voltageOutput = pidVoltage + feedForwardVoltage
        hoodMotor.setVoltage(
            voltageOutput
        )

        hoodSim.setInput(voltageOutput)
        hoodSim.update(0.020) // 20ms

        hoodEncoderSim.distance = hoodSim.angleRads
        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(hoodSim.currentDrawAmps)
        )
        hoodMechanism.angle = Units.radiansToDegrees(hoodSim.angleRads)
    }
}
