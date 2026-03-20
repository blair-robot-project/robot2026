package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.team449.Constants.AimbotConstants
import frc.team449.subsystems.RobotActions
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.power.PowerProfile
import frc.team449.subsystems.power.PowerSubsystem
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier
import kotlin.math.PI

class AimAtTargetCommand(
    private val drive: DriveSubsystem,
    private val actions: RobotActions,
    private val targetSupplier: Supplier<Translation2d>
) : Command() {
    private val request = SwerveRequest.FieldCentric()
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)

    private val headingController = ProfiledPIDController(
        AimbotConstants.AIMBOT_KP,
        AimbotConstants.AIMBOT_KI,
        AimbotConstants.AIMBOT_KD,
        TrapezoidProfile.Constraints(
            4.0,
            8.0
        )
    ).apply {
        enableContinuousInput(-PI, PI)
        setTolerance(AimbotConstants.AIMBOT_HEADING_TOLERANCE_RADIANS)
    }

    var currentPose: Pose2d = drive.pose
    var translationToTarget: Translation2d = targetSupplier.get().minus(currentPose.translation)
    var distance: Double = translationToTarget.norm

    var targetHeading: Rotation2d = translationToTarget.angle
    var omegaRadPerSec: Double = headingController.calculate(currentPose.rotation.radians, targetHeading.radians)

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        headingController.reset(drive.pose.rotation.radians)
        CommandScheduler.getInstance().schedule(PowerSubsystem.requestProfile(PowerProfile.SHOOTING))
        println("Initializing AimAtTargetCommand.")
    }

    override fun execute() {
        currentPose = drive.pose
        translationToTarget = targetSupplier.get().minus(currentPose.translation)
        distance = translationToTarget.norm

        targetHeading = translationToTarget.angle
        omegaRadPerSec = headingController.calculate(currentPose.rotation.radians, targetHeading.radians)

        drive.setControl(
            request
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(omegaRadPerSec)
        )

        CommandScheduler.getInstance().schedule(
            actions.prepShotFromAnywhere(distance)
        )

        Logger.recordOutput("Aimbot/HeadingErrorRads", headingController.positionError)
        Logger.recordOutput("Aimbot/ProfiledTargetVelocity", headingController.setpoint.velocity)
        Logger.recordOutput("Aimbot/DistanceToHubMeters", distance)
    }

    override fun isFinished(): Boolean = headingController.atGoal()

    override fun end(interrupted: Boolean) = println("AimAtTargetCommand Interrupted: $interrupted")
}
