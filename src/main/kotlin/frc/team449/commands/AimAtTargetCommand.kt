package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Constants.AlignConstants
import frc.team449.Constants.DriveConstants
import frc.team449.Constants.ShooterConstants
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.shooter.ShooterSubsystem
import frc.team449.util.FieldUtil
import org.littletonrobotics.junction.Logger
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import ShootOnTheMove.*
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

class AimAtTargetCommand(
    private val drive: DriveSubsystem,
    private val shooter: ShooterSubsystem,
    private val shotCalculator: ShotCalculator,
    private val throttleSupplier: DoubleSupplier,
    private val strafeSupplier: DoubleSupplier,
    private val maxLinearSpeedMetersPerSecond: Double = DriveConstants.SLOW_LINEAR_SPEED_METERS_PER_SEC,
    private val targetSupplier: Supplier<Translation2d>
) : Command() {
    private val driveWithHeading =
        SwerveRequest
            .FieldCentricFacingAngle()
            .withHeadingPID(AlignConstants.ALIGN_KP, 0.0, AlignConstants.ALIGN_KD)
            .withDeadband(maxLinearSpeedMetersPerSecond * DriveConstants.TRANSLATION_DEADBAND)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)

    private var throttle: Double = 0.0
    private var strafe: Double = 0.0

    var isRed: Boolean = false

    init {
        addRequirements(drive, shooter)
    }

    override fun initialize() {
        isRed = FieldUtil.isRed
        driveWithHeading.HeadingController.setTolerance(AlignConstants.POSITION_TOLERANCE_RADS, AlignConstants.VELOCITY_TOLERANCE_RADS_PER_SEC)
    }

    override fun execute() {
        val currentPose = drive.pose
        val targetTranslation = targetSupplier.get()
        val translationToTarget = if (isRed) currentPose.translation.minus(targetTranslation) else targetTranslation.minus(currentPose.translation)
        val targetRotation = translationToTarget.angle
        val distance = FieldUtil.getDistanceToPose(currentPose, targetTranslation)

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

        //sotm calcs
        val shotInputs = ShotCalculator.ShotInputs(
            currentPose,
            drive.getFieldRelativeSpeeds(),
            drive.getRobotRelativeSpeeds(),
            FieldUtil.HUB,
            FieldUtil.HUB,
            0.9,
            0.0,
            0.0
        )

        shooter.setFlywheelVelocityInternal(
            RadiansPerSecond.of(ShooterConstants.FLYWHEEL_VELOCITY_MAP.get(distance))
        )
        shooter.setHoodAngleInternal(
            Radians.of(ShooterConstants.HOOD_ANGLE_MAP.get(distance))
        )

        Logger.recordOutput("Align/HeadingErrorRads", driveWithHeading.HeadingController.positionError)
        Logger.recordOutput("Align/DistanceToHubMeters", distance)
    }

    fun atHeadingSetpoint(): Boolean = driveWithHeading.HeadingController.atSetpoint()

    override fun isFinished(): Boolean = false

    override fun end(interrupted: Boolean) {
        drive.setControl(SwerveRequest.Idle())
    }
}
