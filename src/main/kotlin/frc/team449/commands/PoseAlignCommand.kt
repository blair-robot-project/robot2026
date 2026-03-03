package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Constants
import frc.team449.subsystems.drive.DriveSubsystem
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

class PoseAlignCommand(
    private val drive: DriveSubsystem,
    private val targetPoseSupplier: Supplier<Pose2d>
) : Command() {
    private val xController = PIDController(2.0, 0.0, 0.0)
    private val yController = PIDController(2.0, 0.0, 0.0)
    private val thetaController = ProfiledPIDController(
        4.0,
        0.0,
        0.0,
        TrapezoidProfile.Constraints(
            Constants.AutoConstants.AUTO_ANGULAR_SPEED_RADIANS_PER_SECOND,
            Constants.AutoConstants.AUTO_ANGULAR_ACCEL_RADIANS_PER_SECOND_PER_SECOND
        )
    )

    private val driveController = HolonomicDriveController(
        xController,
        yController,
        thetaController
    )

    private val applyChassisSpeeds = SwerveRequest.ApplyRobotSpeeds()

    private val maxSpeed = Constants.DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND

    init {
        thetaController.enableContinuousInput(-PI, PI)
        driveController.setTolerance(Pose2d(0.07, 0.07, Rotation2d.fromDegrees(1.0)))
        addRequirements(drive)
    }

    override fun initialize() {
        thetaController.reset(drive.pose.rotation.radians)
    }

    override fun execute() {
        val currentPose = drive.pose
        val targetPose = targetPoseSupplier.get()

        val robotSpeeds: ChassisSpeeds = driveController.calculate(
            currentPose,
            targetPose,
            0.0,
            targetPose.rotation
        )

        val translationSpeed = sqrt(robotSpeeds.vxMetersPerSecond * robotSpeeds.vxMetersPerSecond + robotSpeeds.vyMetersPerSecond * robotSpeeds.vyMetersPerSecond)
        robotSpeeds.vxMetersPerSecond *= min(translationSpeed, maxSpeed) / translationSpeed
        robotSpeeds.vyMetersPerSecond *= min(translationSpeed, maxSpeed) / translationSpeed

        drive.setControl(applyChassisSpeeds.withSpeeds(robotSpeeds))
    }

    override fun isFinished(): Boolean {
        return driveController.atReference()
    }

    override fun end(interrupted: Boolean) {}
}
