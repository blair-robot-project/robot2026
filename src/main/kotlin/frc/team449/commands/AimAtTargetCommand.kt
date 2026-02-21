package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Velocity
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Constants
import frc.team449.Constants.AimbotConstants.AIMBOT_KD
import frc.team449.Constants.AimbotConstants.AIMBOT_KI
import frc.team449.Constants.AimbotConstants.AIMBOT_KP
import frc.team449.Constants.AimbotConstants.JOYSTICK_POWER
import frc.team449.Constants.AimbotConstants.PERIODIC_TIME
import frc.team449.Constants.AimbotConstants.VELOCITY_COEFFICIENT
import frc.team449.Constants.DriveConstants
import frc.team449.subsystems.drive.DriveSubsystem
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.pow
import kotlin.math.sign

class AimAtTargetCommand(
    private val drive: DriveSubsystem,
    private val throttleSupplier: DoubleSupplier,
    private val strafeSupplier: DoubleSupplier,
    private val targetSupplier: Supplier<Pose2d>
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
        addRequirements(drive)
    }

    override fun initialize() {
        println("Initializing AimbotCommand")

        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            request.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        }
    }

    private var throttle: Double = 0.0
    private var strafe: Double = 0.0
    private var turn: Double = 0.0

    private val turnController = PIDController(
        AIMBOT_KP,
        AIMBOT_KI,
        AIMBOT_KD,
    )

    fun throttleAtPower(power: Double) : Double {
        return abs(throttleSupplier.asDouble).pow(power) * sign(throttleSupplier.asDouble) *
                DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND
    }

    //i know this seems super redundant, i made this just in case it might be used in sotf later
    fun strafeAtPower(power: Double) : Double {
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
    }
}