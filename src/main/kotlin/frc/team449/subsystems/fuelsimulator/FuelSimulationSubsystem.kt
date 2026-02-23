package frc.team449.subsystems.fuelsimulator

import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants
import frc.team449.Constants.FuelSimulationConstants
import frc.team449.Constants.IntakeConstants
import frc.team449.Constants.ShooterConstants
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.intake.IntakeSubsystem
import frc.team449.subsystems.shooter.ShooterSubsystem
import kotlin.math.PI
import kotlin.math.abs

class FuelSimulationSubsystem(
    private val io: FuelIO?,
    driveSubsystem: DriveSubsystem,
    intakeSubsystem: IntakeSubsystem,
    shooterSubsystem: ShooterSubsystem
) : SubsystemBase() {
    private val inputs: FuelIOInputsAutoLogged = FuelIOInputsAutoLogged()
    private var simBallThrottle = 0
    private var simIntakeThrottle = 0

    private val intake = intakeSubsystem
    private val shooter = shooterSubsystem

    init {
        if (io != null) {
            io.fuelSim.enableAirResistance()
            io.fuelSim.spawnStartingFuel()
            io.fuelSim.registerRobot(
                Constants.Dimensions.FULL_WIDTH.`in`(Meters),
                Constants.Dimensions.FULL_LENGTH.`in`(Meters),
                Constants.Dimensions.BUMPER_HEIGHT.`in`(Meters),
                driveSubsystem::pose,
                driveSubsystem::getFieldRelativeSpeeds,
            )

            io.fuelSim.registerIntake(
                (Constants.Dimensions.FULL_LENGTH + Inches.of(12.0))
                    .div(2.0)
                    .`in`(Meters),
                (Constants.Dimensions.FULL_LENGTH + Inches.of(12.0))
                    .div(2.0)
                    .plus(Inches.of(3.0))
                    .`in`(Meters),
                -Constants.Dimensions.FULL_WIDTH
                    .div(2.0)
                    .minus(Inches.of(2.0))
                    .`in`(Meters),
                Constants.Dimensions.FULL_WIDTH
                    .div(2.0)
                    .minus(Inches.of(5.0))
                    .`in`(Meters),
                this::pollIntakeAcceptBall,
                { io.ballCount + -1 }
            )
        }
    }

    override fun simulationPeriodic() {
        if (io != null) {
            pollFuelLaunch()
            io.simIntaking = pollIntakeAcceptBall()
            io.fuelSim.updateSim()
            io.updateInputs(inputs)
        }
    }

    fun pollFuelLaunch() {
        if (simBallThrottle >= FuelSimulationConstants.FLYWHEEL_BPS_RATE_LIMIT && io!!.ballCount > 0 && shooter.isFlywheelAtTolerance() && shooter.flywheelTargetVelocityRadPerSec >= 10) {
            val flywheelSurfaceSpeed = shooter.inputs.rightLeaderVelocityRadPerSec * ShooterConstants.FLYWHEEL_RADIUS
            val hoodRollerSurfaceSpeed = shooter.inputs.rightLeaderVelocityRadPerSec * 1 / ShooterConstants.HOOD_ROLLER_GEARING * ShooterConstants.HOOD_ROLLER_RADIUS.`in`(Meters)
            val effectiveShootSpeed = (flywheelSurfaceSpeed + hoodRollerSurfaceSpeed) / 2 * ShooterConstants.EFFICIENCY

            io.fuelSim.launchFuel(
                MetersPerSecond.of(effectiveShootSpeed),
                Radians.of((PI / 2) - shooter.inputs.hoodPositionRad),
                Radians.of(0.0),
                ShooterConstants.SHOOTER_HEIGHT,
            )
            io.ballCount -= 1
            simBallThrottle = 0
        } else {
            if (simBallThrottle < FuelSimulationConstants.FLYWHEEL_BPS_RATE_LIMIT && Math.random() > FuelSimulationConstants.SIMULATED_BALL_INDEXING_MISS_CHANCE) { // randomness to modulate ball timing
                simBallThrottle += 1
            }
        }
    }

    fun pollIntakeAcceptBall(): Boolean {
        val intakeDeployed = abs(IntakeConstants.DEPLOY_POS_RADS - intake.inputs.leftPivotLeaderPositionRad) <= 0.2
        if (simIntakeThrottle < FuelSimulationConstants.INTAKE_BPS_RATE_LIMIT && Math.random() > FuelSimulationConstants.SIMULATED_BALL_INTAKING_MISS_CHANCE) {
            simIntakeThrottle += 1
        }
        return simIntakeThrottle >= FuelSimulationConstants.INTAKE_BPS_RATE_LIMIT && intakeDeployed && io!!.ballCount < FuelSimulationConstants.SIMULATED_BALL_INTAKE_LIMIT
        //                && abs(IntakeConstants.INTAKE_VELOCITY.`in`(RotationsPerSecond) - inputs.leftRollerLeaderVelocityRadPerSec) <= 25.0
    }
}
