package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.controller.HolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.Constants
import frc.team449.subsystems.drive.DriveSubsystem
import java.util.function.Supplier
import kotlin.math.PI

class PoseAlignCommand(
    private val drive: DriveSubsystem,
    private val targetPoseSupplier: Supplier<Pose2d>,
    private val override: Trigger
) : Command() {
    private val xController = PIDController(4.0, 0.0, 0.05)
    private val yController = PIDController(4.0, 0.0, 0.05)
    private val thetaController = ProfiledPIDController(
        4.0,
        0.0,
        0.0,
        TrapezoidProfile.Constraints(
            Constants.AutoConstants.AUTO_ANGULAR_SPEED_RADIANS_PER_SECOND,
            Constants.AutoConstants.AUTO_ANGULAR_ACCEL_RADIANS_PER_SECOND_PER_SECOND
        )
    )

    private val driveController = HolonomicDriveController(xController, yController, thetaController)

    private val applyChassisSpeeds = SwerveRequest.RobotCentric()
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)

    init {
        thetaController.enableContinuousInput(-PI, PI)
        driveController.setTolerance(Pose2d(0.05, 0.05, Rotation2d.fromDegrees(2.0)))
        addRequirements(drive)
    }

    override fun initialize() {
        thetaController.reset(drive.pose.rotation.radians)
    }

    override fun execute() {
        val robotSpeeds: ChassisSpeeds = driveController.calculate(
            drive.pose,
            targetPoseSupplier.get(),
            0.0,
            targetPoseSupplier.get().rotation
        )

        val xVel = robotSpeeds.vxMetersPerSecond
            .coerceIn(-MAX_LINEAR_SPEED_METERS_PER_SECOND, MAX_LINEAR_SPEED_METERS_PER_SECOND)
        val yVel = robotSpeeds.vyMetersPerSecond
            .coerceIn(-MAX_LINEAR_SPEED_METERS_PER_SECOND, MAX_LINEAR_SPEED_METERS_PER_SECOND)

        drive.setControl(
            applyChassisSpeeds
                .withVelocityX(xVel)
                .withVelocityY(yVel)
                .withRotationalRate(robotSpeeds.omegaRadiansPerSecond)
        )
    }

    override fun isFinished(): Boolean {
        return driveController.atReference() || override.asBoolean
    }

    override fun end(interrupted: Boolean) {
        drive.setControl(SwerveRequest.Idle())
    }

    companion object {
        private const val MAX_LINEAR_SPEED_METERS_PER_SECOND = 4.0
    }
}
