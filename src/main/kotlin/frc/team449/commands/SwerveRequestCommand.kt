package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.wpilibj2.command.Command
import frc.team449.Constants
import frc.team449.subsystems.drive.DriveSubsystem
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sign

class SwerveRequestCommand(
    private val drive: DriveSubsystem,
    private val throttleSupplier: DoubleSupplier,
    private val strafeSupplier: DoubleSupplier,
    private val turnSupplier: DoubleSupplier,
    private val maxLinearSpeedMetersPerSecond: Double,
    private val maxAngularSpeedRadiansPerSecond: Double
) : Command() {
    private val driveNoHeading: SwerveRequest.FieldCentric =
        SwerveRequest
            .FieldCentric()
            .withDeadband(maxLinearSpeedMetersPerSecond * Constants.DriveConstants.TRANSLATION_DEADBAND)
            .withRotationalDeadband(maxAngularSpeedRadiansPerSecond * Constants.DriveConstants.ANGULAR_DEADBAND)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)

    private var throttle: Double = 0.0
    private var strafe: Double = 0.0
    private var turn: Double = 0.0

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        println("Initializing SwerveRequestCommand")

        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            driveNoHeading.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        }
    }

    override fun execute() {
        throttle =
            abs(throttleSupplier.asDouble).pow(2) * sign(throttleSupplier.asDouble) *
            maxLinearSpeedMetersPerSecond
        strafe =
            abs(strafeSupplier.asDouble).pow(2) * sign(strafeSupplier.asDouble) *
            maxLinearSpeedMetersPerSecond
        turn =
            abs(turnSupplier.asDouble).pow(2) * sign(turnSupplier.asDouble) * maxAngularSpeedRadiansPerSecond

        drive.setControl(
            driveNoHeading
                .withVelocityX(throttle)
                .withVelocityY(strafe)
                .withRotationalRate(turn),
        )
    }

    override fun isFinished(): Boolean = false
}
