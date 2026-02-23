package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Velocity
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Constants
import frc.team449.Constants.AimbotConstants.AIMBOT_KD
import frc.team449.Constants.AimbotConstants.AIMBOT_KI
import frc.team449.Constants.AimbotConstants.AIMBOT_KP
import frc.team449.Constants.DriveConstants
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.shooter.ShooterSubsystem
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sign

class AimAtTargetCommand(
    private val drive: DriveSubsystem,
    private val shooter: ShooterSubsystem,
    private val throttleSupplier: DoubleSupplier,
    private val strafeSupplier: DoubleSupplier,
    private val targetSupplier: Supplier<Translation2d>,
    private val distanceSupplier: Supplier<Double>
) : Command() {

    private val request = SwerveRequest
        .FieldCentric()
        .withDeadband(
            DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND
                * DriveConstants.TRANSLATION_DEADBAND,
        ).withRotationalDeadband(
            DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND
                * DriveConstants.ANGULAR_DEADBAND,
        ).withDriveRequestType(SwerveModule.DriveRequestType.Velocity)

    init {
        addRequirements(drive, shooter)
    }

    override fun initialize() {
        println("Initializing AimbotCommand")

        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            request.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        }

        // not doing this in execute because this is just for shooting from anywhere
        shooter.setHoodAngleFromDistance(distanceSupplier)
        shooter.setFlywheelVelocityFromDistance(distanceSupplier)
    }

    private var throttle: Double = 0.0
    private var strafe: Double = 0.0
    private var turn: Double = 0.0

    private val turnController = PIDController(
        AIMBOT_KP,
        AIMBOT_KI,
        AIMBOT_KD,
    )

    fun throttleAtPower(power: Double): Double {
        return abs(throttleSupplier.asDouble).pow(power) * sign(throttleSupplier.asDouble) *
            DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND
    }

    // i know this seems super redundant, i made this just in case it might be used in sotf later
    fun strafeAtPower(power: Double): Double {
        return abs(strafeSupplier.asDouble).pow(power) * sign(strafeSupplier.asDouble) *
            DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND
    }

    override fun execute() {
        val target = targetSupplier.get()
        throttle = throttleAtPower(2.0)
        strafe = strafeAtPower(2.0)

        val targetHeading = Rotation2d(atan2(target.y - drive.pose.y, target.x - drive.pose.x))
        val delta = targetHeading.minus(drive.pose.rotation).radians
        turn = turnController.calculate(
            0.0,
            delta
        )

        drive.setControl(
            request
                .withVelocityX(throttle)
                .withVelocityY(strafe)
                .withRotationalRate(turn)
        )

        val headingErrorDegrees = targetHeading.minus(drive.pose.rotation).degrees
        SmartDashboard.putNumber("aimbot degree error", headingErrorDegrees)

        shooter.setHoodAngleFromDistance(distanceSupplier)
        shooter.setFlywheelVelocityFromDistance(distanceSupplier)
    }
}
