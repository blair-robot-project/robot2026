package frc.team449.subsystems.shooter

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
import frc.team449.Constants.ShooterConstants.FLYWHEEL_MOI
import frc.team449.Constants.ShooterConstants.HOOD_GEARING
import frc.team449.Constants.ShooterConstants.HOOD_LENGTH
import frc.team449.Constants.ShooterConstants.HOOD_MOI
import frc.team449.Constants.ShooterConstants.MAX_HOOD_ANGLE
import frc.team449.Constants.ShooterConstants.MIN_HOOD_ANGLE

class ShooterIOSim : ShooterIOHardware() {
    private val flywheelSim: FlywheelSim =
        FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2),
                FLYWHEEL_MOI,
                FLYWHEEL_GEARING,
            ),
            DCMotor.getKrakenX60(2),
        )
    // two flywheel sims will have the exact same behavior -- unnecessary

    val hoodSim: SingleJointedArmSim =
        SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            HOOD_GEARING,
            HOOD_MOI,
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
    }

    fun simulationPeriodic() {
        val totalCurrent = hoodSim.currentDrawAmps + flywheelSim.currentDrawAmps // * 2 // simulating two flywheels
        val loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrent)
        RoboRioSim.setVInVoltage(loadedVoltage)

        hoodSimState.setSupplyVoltage(loadedVoltage)
        leftLeaderSimState.setSupplyVoltage(loadedVoltage)
        leftFollowerSimState.setSupplyVoltage(loadedVoltage)
        rightLeaderSimState.setSupplyVoltage(loadedVoltage)
        rightFollowerSimState.setSupplyVoltage(loadedVoltage)

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
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        super.updateInputs(inputs)
    }
}
