package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.subsystems.drive.DriveSubsystem
import kotlin.math.abs

/** Measures the robot's wheel radius by spinning in a circle. */
class WheelRadiusCharacterizationCommand(
    private val drive: DriveSubsystem
) : Command() {
    private var accumWheelDistance = 0.0
    private var gyroDelta = 0.0

    private var lastAngle = Rotation2d()
    private var lastPositions = DoubleArray(4)
    private val timer = Timer()

    private val request = SwerveRequest.RobotCentric()
        .withVelocityX(0.0)
        .withVelocityY(0.0)
        .withRotationalRate(2.0)

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        timer.restart()
        accumWheelDistance = 0.0
        gyroDelta = 0.0
        lastAngle = drive.pose.rotation

        val currentPositions = drive.modulePositions
        for (i in 0..3) {
            lastPositions[i] = currentPositions[i].distanceMeters
        }
    }

    override fun execute() {
        drive.setControl(request)

        val currentPositions = drive.modulePositions
        val currentAngle = drive.pose.rotation

        // allow 1 second to secure wheel alignment
        if (timer.hasElapsed(1.0)) {
            for (i in 0..3) {
                accumWheelDistance += abs(currentPositions[i].distanceMeters - lastPositions[i])
            }
            gyroDelta += abs(currentAngle.minus(lastAngle).radians)
        }

        for (i in 0..3) {
            lastPositions[i] = currentPositions[i].distanceMeters
        }
        lastAngle = currentAngle
    }

    override fun isFinished(): Boolean {
        return gyroDelta >= Math.PI * 10
    }

    override fun end(interrupted: Boolean) {
        if (timer.hasElapsed(1.0)) {
            val averageWheelDistance = accumWheelDistance / 4.0

            val drivebaseRadius = 0.3906411413 // meters
            val currentWheelRadius = 0.0508 // meters

            val effectiveRadius = currentWheelRadius * (gyroDelta * drivebaseRadius) / averageWheelDistance

            println("********** Wheel Radius Characterization Results **********")
            println("\tWheel Delta (Meters): $averageWheelDistance")
            println("\tGyro Delta: $gyroDelta Radians")
            println("\tCalculated Effective Wheel Radius: $effectiveRadius")
        } else {
            println("Characterization interrupted or didn't run long enough to measure!")
        }
    }
}
