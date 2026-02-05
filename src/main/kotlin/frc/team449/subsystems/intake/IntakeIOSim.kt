package frc.team449.subsystems.intake
import edu.wpi.first.math.system.plant.DCMotor
import org.ironmaple.simulation.IntakeSimulation
import edu.wpi.first.units.Units.Meters
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation
import frc.team449.subsystems.shooter.ShooterIOSim

class IntakeIOSim(driveTrainSimulation: SwerveDriveSimulation) : IntakeIO {
    // instant set to target
    private val pivotMotor = DCMotor.getKrakenX44(1)
    private val leftRollerMotor = DCMotor.getKrakenX60(1)
    private val rightRollerMotor = DCMotor.getKrakenX60(1)

    private var requestedPivotVoltage: Double = 0.0
    private var requestedLeftVoltage: Double = 0.0
    private var requestedRightVoltage: Double = 0.0

    private var pivotAngleRad = 0.0
    private var pivotSpeed = 0.0
    private val intakeSimulation: IntakeSimulation =
        IntakeSimulation.InTheFrameIntake(
            "Note",                //game peic ename
            driveTrainSimulation,
            Meters.of(0.7),
            IntakeSimulation.IntakeSide.BACK,
            1                             // max notes
        )

    override fun updateInputs(inputs: IntakeIO.IntakeIOInputs) {
        inputs.currentPivotVoltage = requestedPivotVoltage
        inputs.currentLeftRollerVoltage = requestedLeftVoltage
        inputs.currentRightRollerVoltage = requestedRightVoltage

        inputs.pivotSupplyCurrent = pivotMotor.getCurrent(0.0, requestedPivotVoltage)
        inputs.leftRollerSupplyCurrent = leftRollerMotor.getCurrent(0.0, requestedLeftVoltage)
        inputs.rightRollerSupplyCurrent = rightRollerMotor.getCurrent(0.0, requestedRightVoltage)
        pivotSpeed = requestedPivotVoltage * 0.8
        pivotAngleRad += pivotSpeed * 0.02

        inputs.pivotAngleRad = pivotAngleRad
        inputs.pivotSpeed = pivotSpeed
    }

    override fun setVoltagePivot(
        pivotVoltage: Double,
    ) {
        requestedPivotVoltage = pivotVoltage
    }

    override fun setVoltageRoller(rightRollerVoltage: Double) {
        requestedRightVoltage = rightRollerVoltage
    }

    override fun setRunning(runIntake: Boolean) {
        if (runIntake) {
            intakeSimulation.startIntake()
        } else {
            intakeSimulation.stopIntake()
        }
    }

    override fun isNoteInsideIntake(): Boolean {
        return intakeSimulation.gamePiecesAmount != 0
    }

    override fun launchNote() {
        if (intakeSimulation.obtainGamePieceFromIntake()) {
            ShooterIOSim.launchNote()
        }
    }
}
