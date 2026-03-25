package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Constants
import frc.team449.Constants.AimbotConstants
import frc.team449.Constants.ShooterConstants
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.shooter.ShooterSubsystem
import frc.team449.util.FieldUtil
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

class AimAtTargetCommand(
    private val drive: DriveSubsystem,
    private val shooter: ShooterSubsystem,
    private val throttleSupplier: DoubleSupplier,
    private val strafeSupplier: DoubleSupplier,
    private val maxLinearSpeedMetersPerSecond: Double = Constants.DriveConstants.SLOW_LINEAR_SPEED_METERS_PER_SEC,
    private val maxAngularSpeedRadiansPerSecond: Double = Constants.DriveConstants.SLOW_ANGULAR_SPEED_RADS_PER_SEC,
    private val targetSupplier: Supplier<Translation2d>
) : Command() {
    private val driveWithHeading =
        SwerveRequest
            .FieldCentricFacingAngle()
            .withHeadingPID(AimbotConstants.AIMBOT_KP, AimbotConstants.AIMBOT_KI, AimbotConstants.AIMBOT_KD)
            .withDeadband(maxLinearSpeedMetersPerSecond * Constants.DriveConstants.TRANSLATION_DEADBAND)
            .withRotationalDeadband(maxAngularSpeedRadiansPerSecond * Constants.DriveConstants.ANGULAR_DEADBAND)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)

    private var throttle: Double = 0.0
    private var strafe: Double = 0.0

    var alliance: DriverStation.Alliance? = null

    init {
        addRequirements(drive)
        driveWithHeading.HeadingController.setTolerance(AimbotConstants.POSITION_TOLERANCE_RAD, AimbotConstants.VELOCITY_TOLERANCE_RADS_PER_SEC)
    }

    override fun initialize() {
        println("Initializing AimAtTargetCommand!")
        alliance = DriverStation.getAlliance().get()
    }

    override fun execute() {
        val currentPose = drive.pose
        val targetTranslation = targetSupplier.get()
        val translationToTarget = if (alliance == DriverStation.Alliance.Blue) targetTranslation.minus(currentPose.translation) else currentPose.translation.minus(targetTranslation)
        val targetRotation = translationToTarget.angle

        throttle =
            abs(throttleSupplier.asDouble).pow(2) * sign(throttleSupplier.asDouble) *
            maxLinearSpeedMetersPerSecond
        strafe =
            abs(strafeSupplier.asDouble).pow(2) * sign(strafeSupplier.asDouble) *
            maxLinearSpeedMetersPerSecond

        drive.setControl(
            driveWithHeading
                .withVelocityX(throttle)
                .withVelocityY(strafe)
                .withTargetDirection(targetRotation)
        )

        shooter.setFlywheelVelocityInternal(
            RadiansPerSecond.of(ShooterConstants.FLYWHEEL_VELOCITY_MAP.get(FieldUtil.getDistanceToHub(drive.pose)))
        )

        shooter.setHoodAngleInternal(
            Radians.of(ShooterConstants.HOOD_ANGLE_MAP.get(FieldUtil.getDistanceToHub(drive.pose)))
        )

        Logger.recordOutput("Aimbot/HeadingErrorRads", driveWithHeading.HeadingController.positionError)
        Logger.recordOutput("Aimbot/DistanceToHubMeters", FieldUtil.getDistanceToHub(drive.pose))
    }

    private fun isStationary(driveSpeeds: ChassisSpeeds): Boolean = abs(driveSpeeds.vxMetersPerSecond) < 0.01 && abs(driveSpeeds.vyMetersPerSecond) < 0.1

    override fun isFinished(): Boolean = driveWithHeading.HeadingController.atSetpoint() && isStationary(drive.getRobotRelativeSpeeds())

    override fun end(interrupted: Boolean) {
        drive.setControl(SwerveRequest.Idle())
        println("AimAtTargetCommand Complete!")
    }
}
