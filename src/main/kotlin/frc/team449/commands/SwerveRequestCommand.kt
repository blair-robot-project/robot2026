package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.filter.SlewRateLimiter
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team449.Constants
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.power.PowerProfile
import frc.team449.subsystems.power.PowerSubsystem
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sin

class SwerveRequestCommand(
    private val drive: DriveSubsystem,
    private val throttleSupplier: DoubleSupplier,
    private val strafeSupplier: DoubleSupplier,
    private val turnSupplier: DoubleSupplier,
    private val maxLinearSpeedMetersPerSecond: Double = Constants.DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND,
    private val maxAngularSpeedRadiansPerSecond: Double = Constants.DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND
) : Command() {
    private val driveNoHeading: SwerveRequest.FieldCentric =
        SwerveRequest
            .FieldCentric()
            .withDeadband(maxLinearSpeedMetersPerSecond * Constants.DriveConstants.TRANSLATION_DEADBAND)
            .withRotationalDeadband(maxAngularSpeedRadiansPerSecond * Constants.DriveConstants.ANGULAR_DEADBAND)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)

    val translationLimiter = SlewRateLimiter(8.0, -12.0, 0.0)

    private var throttle: Double = 0.0
    private var strafe: Double = 0.0
    private var turn: Double = 0.0

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        println("Initializing SwerveRequestCommand")
        CommandScheduler.getInstance().schedule(PowerSubsystem.requestProfile(PowerProfile.DRIVING))
        translationLimiter.reset(0.0)
    }

    override fun execute() {
        val rawThrottle = abs(throttleSupplier.asDouble).pow(2) * sign(throttleSupplier.asDouble) * maxLinearSpeedMetersPerSecond
        val rawStrafe = abs(strafeSupplier.asDouble).pow(2) * sign(strafeSupplier.asDouble)  * maxLinearSpeedMetersPerSecond

        val rawMagnitude = hypot(rawThrottle, rawStrafe)
        val magnitude = translationLimiter.calculate(rawMagnitude)
        val angle = atan2(rawStrafe, rawThrottle)


        println("raw magnitude : ${rawMagnitude}, magnitude : $magnitude")

        throttle = cos(angle) * magnitude
        strafe = sin(angle) * magnitude
        turn = abs(turnSupplier.asDouble).pow(2) * sign(turnSupplier.asDouble) * maxAngularSpeedRadiansPerSecond

        drive.setControl(
            driveNoHeading
                .withVelocityX(throttle)
                .withVelocityY(strafe)
                .withRotationalRate(turn),
        )
    }

    override fun isFinished(): Boolean = false
}
