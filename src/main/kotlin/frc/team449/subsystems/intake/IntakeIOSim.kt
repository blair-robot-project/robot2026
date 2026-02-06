package frc.team449.subsystems.intake
import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.sim.TalonFXSimState
import edu.wpi.first.math.system.plant.DCMotor
import org.ironmaple.simulation.IntakeSimulation
import edu.wpi.first.units.Units.*
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim
import frc.team449.Constants.IntakeConstants
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import frc.team449.subsystems.shooter.ShooterIOSim
import frc.team449.util.PhoenixUtil.tryUntilOk

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
    }

    fun simulationPeriodic() {
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
