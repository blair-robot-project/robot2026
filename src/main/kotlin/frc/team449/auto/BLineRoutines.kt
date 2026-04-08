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
import frc.team449.Constants.AutoConstants.AUTO_SHOOTING_TIME_SEC
import frc.team449.Constants.AutoConstants.CTE_D
import frc.team449.Constants.AutoConstants.CTE_I
import frc.team449.Constants.AutoConstants.CTE_P
import frc.team449.Constants.AutoConstants.ROTATION_D
import frc.team449.Constants.AutoConstants.ROTATION_I
import frc.team449.Constants.AutoConstants.ROTATION_P
import frc.team449.Constants.AutoConstants.TRANSLATION_D
import frc.team449.Constants.AutoConstants.TRANSLATION_I
import frc.team449.Constants.AutoConstants.TRANSLATION_P
import frc.team449.subsystems.RobotActions
import frc.team449.subsystems.drive.DriveSubsystem
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import java.util.function.Consumer

class BLineRoutines(
    private val drive: DriveSubsystem,
    private val actions: RobotActions
) {
    private val translationP = LoggedNetworkNumber("Auto/Translation/P", TRANSLATION_P)
    private val translationI = LoggedNetworkNumber("Auto/Translation/I", TRANSLATION_I)
    private val translationD = LoggedNetworkNumber("Auto/Translation/D", TRANSLATION_D)

    private val rotationP = LoggedNetworkNumber("Auto/Rotation/P", ROTATION_P)
    private val rotationI = LoggedNetworkNumber("Auto/Rotation/I", ROTATION_I)
    private val rotationD = LoggedNetworkNumber("Auto/Rotation/D", ROTATION_D)

    private val crossTrackP = LoggedNetworkNumber("Auto/CrossTrackError/P", CTE_P)
    private val crossTrackI = LoggedNetworkNumber("Auto/CrossTrackError/I", CTE_I)
    private val crossTrackD = LoggedNetworkNumber("Auto/CrossTrackError/D", CTE_D)

    fun logBLineAuto() {
        translationController.setPID(translationP.get(), translationI.get(), translationD.get())
        rotationController.setPID(rotationP.get(), rotationI.get(), rotationD.get())
        crossTrackController.setPID(crossTrackP.get(), crossTrackI.get(), crossTrackD.get())

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

    private val translationController = PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D)
    private val rotationController = PIDController(ROTATION_P, ROTATION_I, ROTATION_D)
    private val crossTrackController = PIDController(CTE_P, CTE_I, CTE_D)

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
        FollowPath.registerEventTrigger("start_shooting", actions.autoTrenchShot())
        FollowPath.registerEventTrigger("lemon_shoot", actions.autoLemonShot())
        FollowPath.registerEventTrigger("stop_shooting", actions.stopAll())
        FollowPath.registerEventTrigger("start_shooting_hub", actions.autoHubShot())
    }

    private fun nothing(): Command = Commands.none()

    private fun preloadHubShot(): Command {
        val path1 = Path("hub_start")
        val path2 = Path("hub_end")

        eventTriggerCommands()

        return Commands.sequence(
            pathBuilderWithReset(false).build(path1),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilder(false).build(path2),
        )
    }

    private fun halfClose(mirror: Boolean): Command {
        val path1 = Path("R_half_reg_pt1")
        val path2 = Path("R_half_reg_pt2")
        val path3 = Path("R_half_closer_pt1")
        val path4 = Path("R_half_closer_pt2")
        val path5 = Path("r_end")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            pathBuilderWithReset(mirror).build(path1),
            pathBuilder(mirror).build(path2),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path3),
            pathBuilder(mirror).build(path4),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path5),
        )
    }

    private fun regAndCleanUp(mirror: Boolean): Command {
        val path1 = Path("R_half_reg_pt1")
        val path2 = Path("R_half_reg_pt2")
        val path3 = Path("close_cleanup")
        val path4 = Path("r_end")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            pathBuilderWithReset(mirror).build(path1),
            pathBuilder(mirror).build(path2),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path3),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path4),
        )
    }

    private fun doubleCleanUp(mirror: Boolean): Command {
        val path1 = Path("far_cleanup")
        val path2 = Path("close_cleanup")
        val path3 = Path("r_end")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            pathBuilderWithReset(mirror).build(path1),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path2),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path3),
        )
    }

    private fun halfFar(mirror: Boolean): Command {
        val path1 = Path("R_half_far_pt1")
        val path2 = Path("R_half_far_pt2")
        val path3 = Path("R_half_close_pt1")
        val path4 = Path("R_half_close_pt2")

        val path5 = Path("r_end")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            pathBuilderWithReset(mirror).build(path3),
            pathBuilder(mirror).build(path4),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path1),
            pathBuilder(mirror).build(path2),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilder(false).build(path5),
        )
    }

    /**------------bump autos------------**/
    private fun lemonAuto(mirror: Boolean): Command {
        val path1 = Path("lemon_pt1")
        val path2 = Path("lemon_pt2")
        val path3 = Path("lemon_end")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            pathBuilderWithReset(mirror).build(path1),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path2),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilder(mirror).build(path3),
        )
    }

    private fun bumpAuto(mirror: Boolean): Command {
        val path1 = Path("bump_pt1")
        val path2 = Path("bump_pt2")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            WaitCommand(2.0), // wait until partners get out of NZ
            pathBuilderWithReset(mirror).build(path1),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilderWithReset(mirror).build(path2),
        )
    }

    fun addAutoOptions(autoChooser: LoggedDashboardChooser<Command>) {
        autoChooser.addDefaultOption("Do Nothing", nothing())
        autoChooser.addOption("Preload Hub", preloadHubShot())

        autoChooser.addOption("R Half Close", halfClose(false))
        autoChooser.addOption("R Regular & Clean Up", regAndCleanUp(false))
        autoChooser.addOption("R Trench Double Clean Up", doubleCleanUp(false))
        autoChooser.addOption("R Bump Double Clean Up", lemonAuto(false))

        autoChooser.addOption("L Half Close", halfClose(true))
        autoChooser.addOption("L regular & Clean Up", regAndCleanUp(true))
        autoChooser.addOption("L Trench Double Clean Up", doubleCleanUp(true))
        autoChooser.addOption("L Bump Double Clean Up", lemonAuto(true))

        // autoChooser.addOption("R Bump", bumpAuto(false))
        // autoChooser.addOption("L Bump", bumpAuto(true))
        // autoChooser.addOption("R Half Far", halfFar(false))
        // autoChooser.addOption("L Half Far", halfFar(true))
    }
}
