package frc.team449.commands

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.team449.Constants.AimbotConstants
import frc.team449.Constants.DriveConstants
import frc.team449.subsystems.RobotActions
import frc.team449.subsystems.drive.DriveSubsystem
import frc.team449.subsystems.power.PowerProfile
import frc.team449.subsystems.power.PowerSubsystem
import org.littletonrobotics.junction.Logger
import kotlin.math.abs

class ShootAtTargetCommand(
    private val drive: DriveSubsystem,
    private val power: PowerSubsystem,
    private val actions: RobotActions,
    private val target: Translation2d
) : Command() {
    private val request = SwerveRequest.FieldCentricFacingAngle()
        .withDeadband(DriveConstants.MAX_LINEAR_SPEED_METERS_PER_SECOND * DriveConstants.TRANSLATION_DEADBAND,)
        .withRotationalDeadband(DriveConstants.MAX_ANGULAR_SPEED_RADIANS_PER_SECOND * DriveConstants.ANGULAR_DEADBAND,)
        .withDriveRequestType(SwerveModule.DriveRequestType.Velocity,)
        .withHeadingPID(AimbotConstants.AIMBOT_KP, AimbotConstants.AIMBOT_KI, AimbotConstants.AIMBOT_KD,)

    private var shootingSequenceStarted = false

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        power.requestProfile(PowerProfile.SHOOTING)
    }

    override fun execute() {
        val currentPose = drive.pose
        val translationToTarget = target.minus(currentPose.translation)
        val distance = translationToTarget.norm

        val isRed = DriverStation.getAlliance().get() == DriverStation.Alliance.Red
        val targetHeading = if (isRed) translationToTarget.angle.plus(Rotation2d.fromDegrees(180.0)) else translationToTarget.angle

        val headingErrorRad = abs(currentPose.rotation.minus(targetHeading).radians)
        if (headingErrorRad < AimbotConstants.AIMBOT_HEADING_TOLERANCE_RADIANS && !shootingSequenceStarted) {
            CommandScheduler.getInstance().schedule(
                SequentialCommandGroup(
                    drive.xLock(),
                    actions.prepShotFromAnywhere(distance),
                    actions.checkAndFeed(),
                    actions.shuffleIntakePivot()
                )
            )

            shootingSequenceStarted = true
        } else {
            drive.setControl(
                request
                    .withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withTargetDirection(targetHeading)
            )
        }

        Logger.recordOutput("Aimbot/HeadingErrorRads", headingErrorRad)
        Logger.recordOutput("Aimbot/DistanceToHubMeters", distance)
    }
}
