package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Constants
import frc.team449.subsystems.drive.DriveSubsystem
import org.littletonrobotics.junction.Logger
import java.util.Optional
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

class SwerveRequestCommand(
    private val drive: DriveSubsystem,
    private val throttleSupplier: DoubleSupplier,
    private val strafeSupplier: DoubleSupplier,
    private val turnSupplier: DoubleSupplier,
    private val maxLinearSpeedMetersPerSecond: Double = Constants.DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SEC,
    private val maxAngularSpeedRadiansPerSecond: Double = Constants.DriveConstants.MAX_ANGULAR_SPEED_RADS_PER_SEC
) : Command() {
    private val driveNoHeading: SwerveRequest.FieldCentric =
        SwerveRequest
            .FieldCentric()
            .withDeadband(maxLinearSpeedMetersPerSecond * Constants.DriveConstants.TRANSLATION_DEADBAND)
            .withRotationalDeadband(maxAngularSpeedRadiansPerSecond * Constants.DriveConstants.ANGULAR_DEADBAND)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)

    private val driveWithHeading: SwerveRequest.FieldCentricFacingAngle =
        SwerveRequest
            .FieldCentricFacingAngle()
            .withDeadband(maxLinearSpeedMetersPerSecond * Constants.DriveConstants.TRANSLATION_DEADBAND)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)

    private var lastTurnTime = 0.0
    private var headingSetpoint: Optional<Rotation2d> = Optional.empty()

    private var throttle: Double = 0.0
    private var strafe: Double = 0.0
    private var rawTurn: Double = 0.0
    private var turn: Double = 0.0

    init {
        driveWithHeading.HeadingController.setPID(
            Constants.DriveConstants.HEADING_P,
            Constants.DriveConstants.HEADING_I,
            Constants.DriveConstants.HEADING_D,
        )
        driveWithHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI)
        addRequirements(drive)
        this.name = "Swerve"
    }

    override fun initialize() {
        headingSetpoint = Optional.empty()
    }

    override fun execute() {
        throttle =
            abs(throttleSupplier.asDouble).pow(2) * sign(throttleSupplier.asDouble) *
            maxLinearSpeedMetersPerSecond
        strafe =
            abs(strafeSupplier.asDouble).pow(2) * sign(strafeSupplier.asDouble) *
            maxLinearSpeedMetersPerSecond
        rawTurn = turnSupplier.asDouble
        turn = abs(rawTurn).pow(2) * sign(rawTurn) * maxAngularSpeedRadiansPerSecond

        val activelyTurning = abs(rawTurn) > Constants.DriveConstants.ANGULAR_DEADBAND
        if (activelyTurning) {
            lastTurnTime = Timer.getFPGATimestamp()
        }

        val recentlyTurning =
            Timer.getFPGATimestamp() - lastTurnTime < 0.25 &&
                abs(drive.robotRelativeSpeeds.omegaRadiansPerSecond) > Math.toRadians(10.0)

        if (activelyTurning || recentlyTurning) {
            drive.setControl(
                driveNoHeading
                    .withVelocityX(throttle)
                    .withVelocityY(strafe)
                    .withRotationalRate(turn),
            )

            headingSetpoint = Optional.empty()
            Logger.recordOutput("DriveHeading/Mode", "NoHeading")
        } else {
            if (headingSetpoint.isEmpty) {
                headingSetpoint = Optional.of(drive.pose.rotation)
            }
            drive.setControl(
                driveWithHeading
                    .withVelocityX(throttle)
                    .withVelocityY(strafe)
                    .withTargetDirection(headingSetpoint.get()),
            )
            Logger.recordOutput("DriveHeading/Mode", "HeadingLocked")
        }

        if (headingSetpoint.isPresent) {
            Logger.recordOutput("DriveHeading/HeadingSetpoint", headingSetpoint.get())
        }
    }

    override fun isFinished(): Boolean = false
}
