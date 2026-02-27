package frc.team449.subsystems.fuelsimulator

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj2.command.Command
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
    val simulatedIntakingMissChance = .85
    val simulatedHopperLimit = 50

    val fuelSim = FuelSim()

    var ballCount: Int = 0
    var simIntaking: Boolean = false
    var effectiveShotSpeed: Double = 0.0

    private var simBallThrottle = 0
    private var simBall2Throttle = 0
    private var simIntakeThrottle = 0

    private val shooterTransform =
        Transform2d(
            Translation2d(-Units.inchesToMeters(26.5) / 2 * 0.75, 0.0),
            Rotation2d(),
        )

    init {
        fuelSim.enableAirResistance()
        fuelSim.spawnStartingFuel()
        fuelSim.registerRobot(
            Units.inchesToMeters(Constants.ROBOT_WIDTH_INCHES),
            Units.inchesToMeters(Constants.ROBOT_LENGTH_INCHES),
            Units.inchesToMeters(5.0),
            { robotContainer.drive.pose.plus(shooterTransform) },
            robotContainer.drive::getFieldRelativeSpeeds,
        )

        fuelSim.registerIntake(
            Units.inchesToMeters(26.5) / 2,
            Units.inchesToMeters(26.5) / 2 + Units.inchesToMeters(12.0),
            -Units.inchesToMeters(26.5) / 2,
            Units.inchesToMeters(26.5) / 2,
            this::pollIntakeAcceptBall,
        ) { ballCount += 1 }
    }

    override fun simulationPeriodic() {
        pollFuelLaunch()
        pollIntakeAcceptBall()
        fuelSim.updateSim()

        Logger.recordOutput("FuelSim/Fuels", *fuelSim.fuels)
        Logger.recordOutput("FuelSim/BallCount", ballCount)
        Logger.recordOutput("FuelSim/ShotSpeedMetersPerSec", effectiveShotSpeed)
        Logger.recordOutput("FuelSim/SimIntaking", simIntaking)
    }

    fun pollFuelLaunch() {
        val hasBall = ballCount > 0
        val isSpunUp =
            robotContainer.shooter.isFlywheelAtTolerance() &&
                robotContainer.shooter.isHoodAtTolerance() &&
                robotContainer.shooter.flywheelTargetVelocityRadPerSec >= 10.0

        val isFeeding = robotContainer.indexer.indexerTargetVelocityRadPerSec >= 10.0
        val isShooting = hasBall && isSpunUp && isFeeding

        val rightVel = robotContainer.shooter.inputs.rightLeaderVelocityRadPerSec
        val flywheelSurfaceSpeed = rightVel * Constants.ShooterConstants.FLYWHEEL_RADIUS
        val hoodRollerSurfaceSpeed =
            rightVel * (1.0 / Constants.ShooterConstants.HOOD_ROLLER_GEARING) * Constants.ShooterConstants.HOOD_ROLLER_RADIUS.`in`(Meters)
        effectiveShotSpeed = (flywheelSurfaceSpeed + hoodRollerSurfaceSpeed) / 2.0 * Constants.ShooterConstants.EFFICIENCY

        if (isShooting) {
            if (simBallThrottle < flywheelBPSRateLimit) {
                if (Math.random() > simulatedIndexingMissChance) {
                    simBallThrottle++
                }
            } else {
                fuelSim.launchFuel(
                    MetersPerSecond.of(effectiveShotSpeed),
                    Radians.of((PI / 2) - robotContainer.shooter.hoodSimAngle),
                    Radians.of(0.0),
                    Constants.ShooterConstants.SHOOTER_HEIGHT,
                    true
                )
                ballCount--
                simBallThrottle = 0
            }

            if (simBall2Throttle < flywheelBPSRateLimit) {
                if (Math.random() > simulatedIndexingMissChance) {
                    simBall2Throttle++
                }
            } else {
                fuelSim.launchFuel(
                    MetersPerSecond.of(effectiveShotSpeed),
                    Radians.of((PI / 2) - robotContainer.shooter.hoodSimAngle),
                    Radians.of(0.0),
                    Constants.ShooterConstants.SHOOTER_HEIGHT,
                    false
                )
                ballCount--
                simBall2Throttle = 0
            }
        }
    }

    fun pollIntakeAcceptBall(): Boolean {
        val intakeDeployed = robotContainer.intake.pivotDeployedState
        // this should be actual vel not target
        val intakeMoving = robotContainer.intake.rollerTargetVelocityRadPerSec >= Constants.IntakeConstants.INTAKE_VELOCITY.`in`(RadiansPerSecond)
        val hasCapacity = ballCount < simulatedHopperLimit
        simIntaking = intakeDeployed && hasCapacity && intakeMoving

        if (simIntaking) {
            if (simIntakeThrottle < intakeBPSRateLimit) {
                if (Math.random() > simulatedIntakingMissChance) {
                    simIntakeThrottle++
                }
            } else {
                simIntakeThrottle = 0
                return true
            }
        }
        return false
    }

    fun resetFuel(): Command =
        runOnce {
            fuelSim.clearFuel()
            fuelSim.spawnStartingFuel()
        }
}
