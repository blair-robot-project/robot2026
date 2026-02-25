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
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.abs

class PoseAlignCommand(
    private val drive: DriveSubsystem,
    private val targetPoseSupplier: Supplier<Pose2d>,
    private val throttleSupplier: DoubleSupplier,
    private val strafeSupplier: DoubleSupplier,
    private val turnSupplier: DoubleSupplier
) : Command() {
    private val xLockDeadband = Constants.DriveConstants.X_LOCK_DEADBAND

    private val xController = PIDController(5.0, 0.0, 0.15)
    private val yController = PIDController(5.0, 0.0, 0.15)
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

    init {
        thetaController.enableContinuousInput(-PI, PI)
        driveController.setTolerance(Pose2d(0.05, 0.05, Rotation2d.fromDegrees(2.0)))
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

        drive.setControl(applyChassisSpeeds.withSpeeds(robotSpeeds))
    }

    override fun isFinished(): Boolean {
        return driveController.atReference() ||
            abs(throttleSupplier.asDouble) > xLockDeadband ||
            abs(strafeSupplier.asDouble) > xLockDeadband ||
            abs(turnSupplier.asDouble) > xLockDeadband
    }

    override fun end(interrupted: Boolean) {}
}
