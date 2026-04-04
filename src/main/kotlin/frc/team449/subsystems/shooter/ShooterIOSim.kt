package frc.team449.subsystems.shooter

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
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
import frc.team449.Constants.ShooterConstants

class ShooterIOSim : ShooterIOHardware() {
    private val flywheelSim: FlywheelSim =
        FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(4),
                ShooterConstants.FLYWHEEL_MOI_KG_MM,
                ShooterConstants.FLYWHEEL_GEARING,
            ),
            DCMotor.getKrakenX60(4),
        )

    val hoodSim: SingleJointedArmSim =
        SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            ShooterConstants.HOOD_GEARING,
            ShooterConstants.HOOD_MOI_KG_MM,
            ShooterConstants.HOOD_LENGTH,
            ShooterConstants.MIN_HOOD_ANGLE.`in`(Radians),
            ShooterConstants.MAX_HOOD_ANGLE.`in`(Radians),
            true,
            ShooterConstants.MIN_HOOD_ANGLE.`in`(Radians)
        )

    private val mech: Mechanism2d = Mechanism2d(3.0, 3.0)
    private val mechRoot: MechanismRoot2d = mech.getRoot("Hood", 1.5, 0.5)
    private val hoodMechanism: MechanismLigament2d =
        mechRoot.append(
            MechanismLigament2d(
                "Hood",
                0.5,
                ShooterConstants.MIN_HOOD_ANGLE.`in`(Radians),
                6.0,
                Color8Bit(Color.kCyan),
            ),
        )

    private val hoodSimState = hood.simState
    private val leftTopLeaderSimState = leftTopLeader.simState
    private val leftBottomFollowerSimState = leftBottomFollower.simState
    private val rightTopFollowerSimState = rightTopFollower.simState
    private val rightBottomFollowerSimState = rightBottomFollower.simState

    init {
        SmartDashboard.putData("Hood", mech)

        leftTopLeader.configurator.apply(
            Slot0Configs()
                .withKS(ShooterConstants.FLYWHEEL_KS)
                .withKV(ShooterConstants.FLYWHEEL_KV)
        )

        hood.configurator.apply(
            MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
        )

        hood.configurator.apply(
            Slot0Configs().withKP(250.0).withKV(ShooterConstants.HOOD_KV)
        )

        hoodMotor.configurator.apply(
            Slot0Configs().withKP(250.0).withKV(ShooterConstants.HOOD_KV)
        )
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        // update flywheel sim
        leftTopLeaderSimState.setSupplyVoltage(12.0)
        leftBottomFollowerSimState.setSupplyVoltage(12.0)
        rightTopFollowerSimState.setSupplyVoltage(12.0)
        rightBottomFollowerSimState.setSupplyVoltage(12.0)
        flywheelSim.setInput(leftTopLeaderSimState.motorVoltage)
        flywheelSim.update(Constants.LOOP_TIME)

        val rotorVel = Units.radiansToRotations(flywheelSim.angularVelocityRadPerSec) * ShooterConstants.FLYWHEEL_GEARING
        leftTopLeaderSimState.setRotorVelocity(rotorVel)
        leftBottomFollowerSimState.setRotorVelocity(rotorVel)
        rightTopFollowerSimState.setRotorVelocity(rotorVel)
        rightBottomFollowerSimState.setRotorVelocity(rotorVel)

        // update hood sim
        hoodSimState.setSupplyVoltage(12.0)
        hoodSim.setInput(hoodSimState.motorVoltage)
        hoodSim.update(Constants.LOOP_TIME)

        val hoodRotorPos = Units.radiansToRotations(hoodSim.angleRads) * ShooterConstants.HOOD_GEARING
        val hoodRotorVel = Units.radiansToRotations(hoodSim.velocityRadPerSec) * ShooterConstants.HOOD_GEARING
        hoodSimState.setRawRotorPosition(hoodRotorPos)
        hoodSimState.setRotorVelocity(hoodRotorVel)

        hoodMechanism.angle = Units.radiansToDegrees(hoodSim.angleRads)

        super.updateInputs(inputs)
    }
}
