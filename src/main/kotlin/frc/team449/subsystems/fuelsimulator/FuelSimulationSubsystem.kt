package frc.team449.subsystems.fuelsimulator

import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.FuelSim
import frc.team449.Constants
import frc.team449.RobotContainer
import org.littletonrobotics.junction.Logger
import kotlin.math.PI
import kotlin.math.round

class FuelSimulationSubsystem(
    private val robotContainer: RobotContainer
) : SubsystemBase() {

    // take these out of Constants so that they're not wasting memory on real robot
    val flywheelSimulatedBPS = 11
    val flywheelBPSRateLimit = round((1 / Constants.LOOP_TIME) / flywheelSimulatedBPS)
    val simulatedIndexingMissChance = .6

    val intakeSimulatedBPS = 20
    val intakeBPSRateLimit = round((1 / Constants.LOOP_TIME) / intakeSimulatedBPS)
    val simulatedIntakingMissChance = .6
    val simulatedHopperLimit = 50

    val fuelSim = FuelSim()
    var ballCount = 0
    var simIntaking = false

    private var simBallThrottle = 0
    private var simIntakeThrottle = 0

    init {
        fuelSim.enableAirResistance()
        fuelSim.spawnStartingFuel()
        fuelSim.registerRobot(
            Units.inchesToMeters(Constants.ROBOT_WIDTH_INCHES),
            Units.inchesToMeters(Constants.ROBOT_LENGTH_INCHES),
            Units.inchesToMeters(5.0),
            robotContainer.drive::pose,
            robotContainer.drive::getFieldRelativeSpeeds
        )

        fuelSim.registerIntake(
            Units.inchesToMeters(26.5) / 2,
            Units.inchesToMeters(26.5) / 2 + Units.inchesToMeters(12.0),
            -Units.inchesToMeters(26.5) / 2,
            Units.inchesToMeters(26.5) / 2,
            this::pollIntakeAcceptBall
        ) { ballCount -= 1 }
    }

    override fun simulationPeriodic() {
        pollFuelLaunch()
        simIntaking = pollIntakeAcceptBall()
        fuelSim.updateSim()

        Logger.recordOutput("Fuels", *fuelSim.fuels)
    }

    fun pollFuelLaunch() {
        if (simBallThrottle < flywheelSimulatedBPS) {
            if (Math.random() > simulatedIndexingMissChance) {
                simBallThrottle++
            }
            return
        }

        val hasBall = ballCount > 0
        val isSpunUp = robotContainer.shooter.isFlywheelAtTolerance()
                && robotContainer.shooter.isHoodAtTolerance()
                && robotContainer.shooter.flywheelTargetVelocityRadPerSec >= 10.0

        val isFeeding = robotContainer.indexer.indexerTargetVelocityRadPerSec >= 10.0

        if (!hasBall || !isSpunUp || !isFeeding) return

        val rightVel = robotContainer.shooter.inputs.rightLeaderVelocityRadPerSec
        val flywheelSurfaceSpeed = rightVel * Constants.ShooterConstants.FLYWHEEL_RADIUS
        val hoodRollerSurfaceSpeed = rightVel * (1.0 / Constants.ShooterConstants.HOOD_ROLLER_GEARING) * Constants.ShooterConstants.HOOD_ROLLER_RADIUS.`in`(Meters)
        val effectiveShotSpeed = (flywheelSurfaceSpeed + hoodRollerSurfaceSpeed) / 2.0 * Constants.ShooterConstants.EFFICIENCY

        fuelSim.launchFuel(
            MetersPerSecond.of(effectiveShotSpeed),
            Radians.of((PI / 2) - robotContainer.shooter.hoodSimAngle),
            Radians.of(0.0),
            Constants.ShooterConstants.SHOOTER_HEIGHT
        )

        ballCount--
        simBallThrottle = 0
    }

    fun pollIntakeAcceptBall(): Boolean {
        val intakeDeployed = robotContainer.intake.pivotDeployedState
        val hasCapacity = ballCount < simulatedHopperLimit

        if (!intakeDeployed || !hasCapacity) {
            return false
        }

        if (simIntakeThrottle < intakeBPSRateLimit) {
            if (Math.random() > simulatedIntakingMissChance) {
                simIntakeThrottle++
            }
            return false
        }

        simIntakeThrottle = 0
        return true
    }
}
