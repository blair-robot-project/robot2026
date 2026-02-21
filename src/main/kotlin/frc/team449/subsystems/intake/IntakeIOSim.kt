package frc.team449.subsystems.intake
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Volt
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import frc.team449.Constants
import frc.team449.Constants.IntakeConstants
import kotlin.math.abs

class IntakeIOSim : IntakeIOHardware() {
    val pivotSim =
        SingleJointedArmSim(
            DCMotor.getKrakenX44(2),
            IntakeConstants.PIVOT_GEARING_SENSOR_TO_MECH,
            IntakeConstants.PIVOT_MOI,
            IntakeConstants.ARM_LENGTH_METERS,
            IntakeConstants.STOW_POS_RADS,
            IntakeConstants.DEPLOY_POS_RADS,
            true,
            IntakeConstants.STOW_POS_RADS,
        )

    private val rollerSim =
        FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2),
                IntakeConstants.ROLLER_MOI,
                IntakeConstants.ROLLER_GEARING,
            ),
            DCMotor.getKrakenX60(2),
        )

    // mech2d stuff
    val mech = Mechanism2d(3.0, 3.0)
    val mechRoot: MechanismRoot2d = mech.getRoot("Intake Pivot", 1.0, 1.0)
    val pivotMechanism: MechanismLigament2d =
        mechRoot.append(
            MechanismLigament2d(
                "Intake Pivot Ligament",
                1.0,
                Units.radiansToDegrees(IntakeConstants.STOW_POS_RADS) + 90 - IntakeConstants.VIZ_OFFSET_DEG,
                4.0,
                Color8Bit(Color.kRed),
            ),
        )

    private val pivotLeaderSim = leftPivotLeader.simState
    private val pivotFollowerSim = rightPivotFollower.simState
    private val rollerLeaderSim = leftRollerLeader.simState
    private val rollerFollowerSim = rightRollerFollower.simState

    init {
        SmartDashboard.putData("Intake", mech)
    }

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        pivotLeaderSim.setSupplyVoltage(RobotController.getBatteryVoltage())
        pivotFollowerSim.setSupplyVoltage(RobotController.getBatteryVoltage())
        rollerLeaderSim.setSupplyVoltage(RobotController.getBatteryVoltage())
        rollerFollowerSim.setSupplyVoltage(RobotController.getBatteryVoltage())

        pivotSim.setInput(pivotLeaderSim.motorVoltage)
        pivotSim.update(Constants.LOOP_TIME)

        val pivotRotorPos = Units.radiansToRotations(pivotSim.angleRads) * IntakeConstants.PIVOT_GEARING_SENSOR_TO_MECH
        val pivotRotorVel = Units.radiansToRotations(pivotSim.velocityRadPerSec) * IntakeConstants.PIVOT_GEARING_SENSOR_TO_MECH

//        pivotLeaderSim.setRawRotorPosition(pivotRotorPos)
//        pivotLeaderSim.setRotorVelocity(pivotRotorVel)
//        pivotFollowerSim.setRawRotorPosition(pivotRotorPos)
//        pivotFollowerSim.setRotorVelocity(pivotRotorVel)

        rollerSim.setInput(rollerLeaderSim.motorVoltage)
        rollerSim.update(0.020)

        val rollerRotorVel = Units.radiansToRotations(rollerSim.angularVelocityRadPerSec) * IntakeConstants.ROLLER_GEARING

        rollerLeaderSim.setRotorVelocity(rollerRotorVel)
        rollerFollowerSim.setRotorVelocity(rollerRotorVel)

        // update viz
        pivotMechanism.angle = Units.radiansToDegrees(pivotSim.angleRads) + 90 - IntakeConstants.VIZ_OFFSET_DEG

        if (abs(rollerSim.angularVelocityRadPerSec) > 1.0) {
            pivotMechanism.color = Color8Bit(Color.kGreen)
        } else {
            pivotMechanism.color = Color8Bit(Color.kRed)
        }

        super.updateInputs(inputs)
    }
}
