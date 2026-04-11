package frc.team449.auto

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.Pair
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.lib.BLine.FollowPath
import frc.robot.lib.BLine.Path
import frc.team449.Constants.AutoConstants
import frc.team449.subsystems.RobotActions
import frc.team449.subsystems.drive.DriveSubsystem
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import java.util.function.Consumer

class BLineRoutines(
    private val drive: DriveSubsystem,
    private val actions: RobotActions
) {
    fun logBLineAuto() {
        // might be too much logging unless we trynna debug stuff, comment it out later
        FollowPath.setPoseLoggingConsumer { pair ->
            Logger.recordOutput(pair.first, pair.second)
        }

        FollowPath.setTranslationListLoggingConsumer(
            Consumer { pair: Pair<String, Array<Translation2d>> ->
                Logger.recordOutput(pair.first, *pair.second)
            },
        )

        FollowPath.setDoubleLoggingConsumer { pair ->
            Logger.recordOutput(pair.first, pair.second)
        }

        FollowPath.setBooleanLoggingConsumer(
            Consumer { pair: Pair<String, Boolean> ->
                Logger.recordOutput(pair.first, pair.second)
            },
        )
    }

    private val translationController = PIDController(
        AutoConstants.TRANSLATION_P,
        AutoConstants.TRANSLATION_I,
        AutoConstants.TRANSLATION_D
    )
    private val rotationController = PIDController(
        AutoConstants.ROTATION_P,
        AutoConstants.ROTATION_I,
        AutoConstants.ROTATION_D
    )
    private val crossTrackController = PIDController(
        AutoConstants.CTE_P,
        AutoConstants.CTE_I,
        AutoConstants.CTE_D
    )

    private val applyRobotSpeedsRequest = SwerveRequest.ApplyRobotSpeeds()

    private fun pathBuilder(mirror: Boolean): FollowPath.Builder =
        FollowPath
            .Builder(
                drive,
                drive::pose,
                drive::getRobotRelativeSpeeds,
                { speeds: ChassisSpeeds ->
                    drive.setControl(applyRobotSpeedsRequest.withSpeeds(speeds))
                },
                translationController,
                rotationController,
                crossTrackController,
            ).withDefaultShouldFlip()
            .withShouldMirror { mirror }

    private fun pathBuilderWithReset(mirror: Boolean): FollowPath.Builder =
        FollowPath
            .Builder(
                drive,
                drive::pose,
                drive::getRobotRelativeSpeeds,
                { speeds: ChassisSpeeds ->
                    drive.setControl(applyRobotSpeedsRequest.withSpeeds(speeds))
                },
                translationController,
                rotationController,
                crossTrackController,
            ).withDefaultShouldFlip()
            .withPoseReset(drive::resetOdometry)
            .withShouldMirror { mirror }

    private fun eventTriggerCommands() {
        FollowPath.registerEventTrigger("start_intake", actions.deployAndIntake())
        FollowPath.registerEventTrigger("trench_shot", actions.autoTrenchShot())
        FollowPath.registerEventTrigger("hub_shot", actions.autoHubShot())
        FollowPath.registerEventTrigger("bump_shot", actions.autoBumpShot())
        FollowPath.registerEventTrigger("stop_shot", actions.stopAll())
    }

    private fun nothing(): Command = Commands.none()

    /**------------TRENCH AUTOS-----------**/

    private fun regularAndTrenchSweep(mirror: Boolean): Command {
        val path1 = Path("R_half_reg_pt1")
        val path2 = Path("R_half_reg_pt2")
        val path3 = Path("trench_sweep")
        val path4 = Path("r_end")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            pathBuilderWithReset(mirror).build(path1),
            pathBuilder(mirror).build(path2),
            WaitCommand(AutoConstants.AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path3),
            WaitCommand(AutoConstants.AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path4),
        )
    }

    private fun farAndTrenchSweep(mirror: Boolean): Command {
        val path1 = Path("R_half_far_pt1")
        val path2 = Path("R_half_far_pt2")
        val path3 = Path("trench_sweep")
        val path4 = Path("r_end")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            pathBuilderWithReset(mirror).build(path1),
            pathBuilder(mirror).build(path2),
            WaitCommand(AutoConstants.AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path3),
            WaitCommand(AutoConstants.AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path4),
        )
    }

    /**------------BUMP AUTOS------------**/

    private fun regularAndBumpSweep(mirror: Boolean): Command {
        val path1 = Path("R_half_reg_pt1")
        val path2 = Path("R_half_reg_pt2")
        val path3 = Path("bump_sweep_pt1")
        val path4 = Path("bump_end")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            pathBuilderWithReset(mirror).build(path1),
            pathBuilder(mirror).build(path2),
            WaitCommand(AutoConstants.AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path3),
            WaitCommand(AutoConstants.AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path4),
        )
    }

    private fun doubleBumpSweep(mirror: Boolean): Command {
        val path1 = Path("bump_sweep_pt1")
        val path2 = Path("bump_sweep_pt2")
        val path3 = Path("bump_end")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            pathBuilderWithReset(mirror).build(path1),
            WaitCommand(AutoConstants.AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path2),
            WaitCommand(AutoConstants.AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path3)
        )
    }

    fun addAutoOptions(autoChooser: LoggedDashboardChooser<Command>) {
        autoChooser.addDefaultOption("Do Nothing", nothing())

        autoChooser.addOption("R Regular & Trench Sweep", regularAndTrenchSweep(false))
        autoChooser.addOption("L Far & Trench Sweep", farAndTrenchSweep(false))
        autoChooser.addOption("R Regular & Bump Sweep", regularAndBumpSweep(false))
        autoChooser.addOption("R Double Bump Sweep", doubleBumpSweep(false))

        autoChooser.addOption("L Regular & Trench Sweep", regularAndTrenchSweep(true))
        autoChooser.addOption("L Far & Trench Sweep", farAndTrenchSweep(true))
        autoChooser.addOption("L Regular & Bump Sweep", regularAndBumpSweep(true))
        autoChooser.addOption("L Double Bump Sweep", doubleBumpSweep(true))
    }
}
