package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.Constants
import frc.team449.Constants.AimbotConstants.AIMBOT_HEADING_TOLERANCE
import frc.team449.Constants.AimbotConstants.AIMBOT_KD
import frc.team449.Constants.AimbotConstants.AIMBOT_KI
import frc.team449.Constants.AimbotConstants.AIMBOT_KP
import frc.team449.Constants.DriveConstants
import frc.team449.subsystems.RobotActions
import frc.team449.subsystems.drive.DriveSubsystem
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier

class ShootAtTargetCommand(
    private val drive: DriveSubsystem,
    private val actions: RobotActions,
    private val exitTrigger: Trigger,
    targetSupplier: Supplier<Translation2d>
) : Command() {

    private val distance: Double
    private val angleSetpoint: Angle
    private val request = SwerveRequest
        .FieldCentricFacingAngle()
        .withDeadband(
            DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND
                * DriveConstants.TRANSLATION_DEADBAND,
        ).withRotationalDeadband(
            DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND
                * DriveConstants.ANGULAR_DEADBAND,
        ).withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
        .withHeadingPID(
            AIMBOT_KP,
            AIMBOT_KI,
            AIMBOT_KD,
        )

    init {
        addRequirements(drive)

        val target = targetSupplier.get()
        distance = drive.pose.translation.getDistance(target)
        angleSetpoint = (
            if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                drive.pose.translation.minus(target).angle
            } else {
                target.minus(drive.pose.translation).angle
            }
            ).measure
    }

    enum class AimbotState {
        ChangingHeading,
        SpinningFlywheel
    }
    private var state = AimbotState.ChangingHeading

    override fun initialize() {
        println("Initializing AimbotCommand")

        if (Constants.CURRENT_MODE == Constants.Mode.SIM) {
            request.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        }
    }

    override fun execute() {
        if (state == AimbotState.ChangingHeading) {
            drive.setControl(
                request
                    .withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withTargetDirection(Rotation2d(angleSetpoint))
            )
            if ((drive.pose.rotation.measure - angleSetpoint).`in`(Radians) < AIMBOT_HEADING_TOLERANCE) {
                Commands.sequence(
                    actions.prepShotFromAnywhere { distance },
                    actions.checkAndFeed()
                ).execute()
                state = AimbotState.SpinningFlywheel
            }
        }

        val headingErrorDegrees = angleSetpoint.minus(drive.pose.rotation.measure).`in`(Degrees)
        Logger.recordOutput("aimbot degree error", headingErrorDegrees)
        Logger.recordOutput("Distance To Hub", distance)
        Logger.recordOutput("aimbot state", state.name)
    }

    override fun isFinished(): Boolean {
        return exitTrigger.asBoolean
    }
}
