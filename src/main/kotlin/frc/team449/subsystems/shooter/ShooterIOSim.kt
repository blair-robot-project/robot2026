package frc.team449.subsystems.shooter

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj.simulation.*
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import frc.team449.Constants
import frc.team449.Constants.ShooterConstants.FLYWHEEL_GEARING
import frc.team449.Constants.ShooterConstants.FLYWHEEL_MOI_KG_MM
import frc.team449.Constants.ShooterConstants.HOOD_GEARING
import frc.team449.Constants.ShooterConstants.HOOD_LENGTH
import frc.team449.Constants.ShooterConstants.HOOD_MOI_KG_MM
import frc.team449.Constants.ShooterConstants.MAX_HOOD_ANGLE
import frc.team449.Constants.ShooterConstants.MIN_HOOD_ANGLE

class ShooterIOSim : ShooterIOHardware() {
    private val flywheelSim: FlywheelSim =
        FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(4),
                FLYWHEEL_MOI_KG_MM,
                FLYWHEEL_GEARING,
            ),
            DCMotor.getKrakenX60(4),
        )

    val hoodSim: SingleJointedArmSim =
        SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            HOOD_GEARING,
            HOOD_MOI_KG_MM,
            HOOD_LENGTH,
            MIN_HOOD_ANGLE.`in`(Radians),
            MAX_HOOD_ANGLE.`in`(Radians),
            true,
            MIN_HOOD_ANGLE.`in`(Radians),
            0.0,
            0.0,
        )

    private val mech: Mechanism2d = Mechanism2d(3.0, 3.0)
    private val mechRoot: MechanismRoot2d = mech.getRoot("Hood", 1.5, 0.5)
    private val hoodMechanism: MechanismLigament2d =
        mechRoot.append(
            MechanismLigament2d(
                "Hood",
                0.5,
                MIN_HOOD_ANGLE.`in`(Radians),
                6.0,
                Color8Bit(Color.kCyan),
            ),
        )

    private val hoodSimState = hoodMotor.simState
    private val leftLeaderSimState = leftLeaderMotor.simState
    private val leftFollowerSimState = leftFollowerMotor.simState
    private val rightLeaderSimState = rightLeaderMotor.simState
    private val rightFollowerSimState = rightFollowerMotor.simState

    init {
        SmartDashboard.putData("Hood", mech)

        hoodMotor.configurator.apply(
            MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
        )
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        val totalCurrent = hoodSim.currentDrawAmps + flywheelSim.currentDrawAmps // * 2 // simulating two flywheels
        // something something dont use stator current alr stop yapping yall

        hoodSimState.setSupplyVoltage(12.0)
        leftLeaderSimState.setSupplyVoltage(12.0)
        leftFollowerSimState.setSupplyVoltage(12.0)
        rightLeaderSimState.setSupplyVoltage(12.0)
        rightFollowerSimState.setSupplyVoltage(12.0)

        hoodSim.setInput(hoodSimState.motorVoltage)
        hoodSim.update(Constants.LOOP_TIME)

        val hoodRotorPos = Units.radiansToRotations(hoodSim.angleRads) * HOOD_GEARING
        val hoodRotorVel = Units.radiansToRotations(hoodSim.velocityRadPerSec) * HOOD_GEARING

        hoodSimState.setRawRotorPosition(hoodRotorPos)
        hoodSimState.setRotorVelocity(hoodRotorVel)

        hoodMechanism.angle = Units.radiansToDegrees(hoodSim.angleRads)

        flywheelSim.setInput(leftLeaderSimState.motorVoltage)
        flywheelSim.update(Constants.LOOP_TIME)

        val rotorVel = Units.radiansToRotations(flywheelSim.angularVelocityRadPerSec) * FLYWHEEL_GEARING

        leftLeaderSimState.setRotorVelocity(rotorVel)
        leftFollowerSimState.setRotorVelocity(rotorVel)
        rightLeaderSimState.setRotorVelocity(rotorVel)
        rightFollowerSimState.setRotorVelocity(rotorVel)

        super.updateInputs(inputs)
    }
}
