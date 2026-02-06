package frc.team449.subsystems.intake
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.sim.TalonFXSimState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.util.Units
import org.ironmaple.simulation.IntakeSimulation
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj.util.Color8Bit
import frc.team449.Constants.IntakeConstants


class IntakeIOSim() : IntakeIOHardware() {
    // instant set to target
    private val pivotGearbox = DCMotor.getKrakenX44(1)
    private val pivotSim = SingleJointedArmSim(
        pivotGearbox,
        IntakeConstants.PIVOT_GEARING_SENSOR_TO_MECH,
        IntakeConstants.PIVOT_MMOI,
        IntakeConstants.ARM_LENGTH.`in`(Meters),
        IntakeConstants.DEPLOY_POSITION.`in`(Radians),
        IntakeConstants.STOW_POSITION.`in`(Radians),
        true,
        IntakeConstants.STOW_POSITION.`in`(Radians),
    )

    // mech2d stuff
    val mech = Mechanism2d(3.0, 3.0)
    val mechRoot = mech.getRoot("Intake Pivot", 1.0, 1.0)
    val pivotMechanism = mechRoot.append(
        MechanismLigament2d(
            "Intake Pivot Ligament",
            1.0,
            IntakeConstants.STOW_POSITION.`in`(Radians),
            4.0,
            Color8Bit(Color.kRed)
        )
    )

    init {
        val pivotMotorSim = pivotMotor.simState
        val followerRollerMotorSim = followerRollerMotor.simState
        val leaderRollerMotorSim = leaderRollerMotor.simState
        pivotMotorSim.Orientation = IntakeConstants.PIVOT_SIM_ORIENTATION
        followerRollerMotorSim.Orientation = IntakeConstants.FOLLOWER_SIM_ORIENTATON
        leaderRollerMotorSim.Orientation = IntakeConstants.LEADER_SIM_ORIENTATION
        pivotMotorSim.setMotorType(TalonFXSimState.MotorType.KrakenX44)
        followerRollerMotorSim.setMotorType(TalonFXSimState.MotorType.KrakenX60)
        leaderRollerMotorSim.setMotorType(TalonFXSimState.MotorType.KrakenX60)

        SmartDashboard.putData("Intake", mech)
    }

    override fun simulationPeriodic() {
        val pivotMotorSim = pivotMotor.simState
        val followerRollerMotorSim = followerRollerMotor.simState
        val leaderRollerMotorSim = leaderRollerMotor.simState

        pivotMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage())
        followerRollerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage())
        leaderRollerMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage())

        // update arm
        pivotSim.setInputVoltage(pivotMotorSim.motorVoltageMeasure.`in`(Volts))
        pivotSim.update(0.02)
        pivotMotorSim.setRawRotorPosition(Radians.of(pivotSim.angleRads))
        pivotMotorSim.setRotorVelocity(RadiansPerSecond.of(pivotSim.velocityRadPerSec))

        pivotMechanism.angle = Units.radiansToDegrees(pivotSim.angleRads)
    }


//    override fun isNoteInsideIntake(): Boolean {
//        return intakeSimulation.gamePiecesAmount != 0
//    }

//    override fun launchNote() {
//        if (intakeSimulation.obtainGamePieceFromIntake()) {
//            ShooterIOSim.launchNote()
//        }
//    }
}
