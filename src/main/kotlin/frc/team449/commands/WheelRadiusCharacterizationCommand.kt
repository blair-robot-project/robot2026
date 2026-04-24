package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Constants.DriveConstants
import frc.team449.generated.TunerConstants
import frc.team449.subsystems.drive.DriveSubsystem
import org.littletonrobotics.junction.Logger
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
        return gyroDelta >= Math.PI * 6
    }

    override fun end(interrupted: Boolean) {
        val averageWheelDistance = accumWheelDistance / 4.0

        val wheelbaseRadius = Units.inchesToMeters(DriveConstants.WHEELBASE_INCHES) / 2 // meters
        val currentWheelRadius = TunerConstants.FrontLeft.WheelRadius // meters

        val effectiveRadius = currentWheelRadius * (gyroDelta * wheelbaseRadius) / averageWheelDistance

        Logger.recordOutput("WheelRadius/WheelDeltaMeters", averageWheelDistance)
        Logger.recordOutput("WheelRadius/GyroDeltaRadians", gyroDelta)
        Logger.recordOutput("WheelRadius/EffectiveRadius", effectiveRadius)
    }
}
