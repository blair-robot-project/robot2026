package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Constants.AlignConstants
import frc.team449.Constants.DriveConstants
import frc.team449.Constants.ShooterConstants
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.indexer.IndexerSubsystem
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
    private val indexer: IndexerSubsystem,
    private val shooter: ShooterSubsystem,
    private val throttleSupplier: DoubleSupplier,
    private val strafeSupplier: DoubleSupplier,
    private val targetSupplier: Supplier<Translation2d>,
    private val maxLinearSpeedMetersPerSecond: Double = DriveConstants.SLOW_LINEAR_SPEED_METERS_PER_SEC,
    private val isScoring: Boolean = true,
    private val flywheelVelocityMap: InterpolatingDoubleTreeMap = ShooterConstants.SCORING_FLYWHEEL_VELOCITY_MAP,
    private val hoodAngleMap: InterpolatingDoubleTreeMap = ShooterConstants.SCORING_HOOD_ANGLE_MAP
) : Command() {
    private val driveWithHeading =
        SwerveRequest
            .FieldCentricFacingAngle()
            .withHeadingPID(AlignConstants.ALIGN_KP, 0.0, AlignConstants.ALIGN_KD)
            .withDeadband(maxLinearSpeedMetersPerSecond * DriveConstants.TRANSLATION_DEADBAND)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
    private val brakeRequest = SwerveRequest.SwerveDriveBrake()

    private var throttle: Double = 0.0
    private var strafe: Double = 0.0
    private var isDriverStationary: Boolean = false

    var isRed: Boolean = false
    val angleSetpointDebounce = Debouncer(0.04)

    init {
        addRequirements(drive, shooter, indexer)
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
        val distanceToTarget = FieldUtil.getDistanceToTranslation(currentPose.translation, targetTranslation)

        throttle =
            abs(throttleSupplier.asDouble).pow(2) * sign(throttleSupplier.asDouble) *
            maxLinearSpeedMetersPerSecond
        strafe =
            abs(strafeSupplier.asDouble).pow(2) * sign(strafeSupplier.asDouble) *
            maxLinearSpeedMetersPerSecond

        isDriverStationary = abs(throttle) < 0.1 && abs(strafe) < 0.1
        val shouldXLock = isScoring && isDriverStationary && atHeadingSetpoint()
        val driveRequest = if (shouldXLock) {
            brakeRequest
        } else {
            driveWithHeading
                .withVelocityX(throttle)
                .withVelocityY(strafe)
                .withTargetDirection(targetRotation)
        }

        drive.setControl(driveRequest)

        shooter.setFlywheelVelocityInternal(RadiansPerSecond.of(flywheelVelocityMap.get(distanceToTarget)))
        shooter.setHoodAngleInternal(Radians.of(hoodAngleMap.get(distanceToTarget)))

        if (readyToShoot()) {
            indexer.setIndexerVoltageInternal(12.0, 12.0)
        } else {
            indexer.setIndexerVoltageInternal(0.0, 0.0)
        }

        Logger.recordOutput("Align/HeadingErrorRads", driveWithHeading.HeadingController.positionError)
        Logger.recordOutput("Align/HeadingAtTarget", atHeadingSetpoint())
        Logger.recordOutput("Align/DistanceToTargetMeters", distanceToTarget)
    }

    fun atHeadingSetpoint(): Boolean =
        angleSetpointDebounce.calculate(driveWithHeading.HeadingController.atSetpoint())

    fun readyToShoot(): Boolean =
        if (isScoring) {
            atHeadingSetpoint() && isDriverStationary && shooter.isFlywheelAtTolerance() && shooter.isHoodAtTolerance()
        } else {
            atHeadingSetpoint() && shooter.isFlywheelAtTolerance() && shooter.isHoodAtTolerance()
        }

    override fun isFinished(): Boolean = false

    override fun getInterruptionBehavior(): InterruptionBehavior {
        return InterruptionBehavior.kCancelIncoming
    }
}
