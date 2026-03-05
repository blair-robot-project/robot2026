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

    val translationController = PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D)
    val rotationController = PIDController(ROTATION_P, ROTATION_I, ROTATION_D)
    val crossTrackController = PIDController(CTE_P, CTE_I, CTE_D)

    private val applyRobotSpeedsRequest = SwerveRequest.ApplyRobotSpeeds()
    var pathBuilder: FollowPath.Builder =
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

    var pathBuilderWithReset: FollowPath.Builder =
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

    fun eventTriggerCommands() {
        FollowPath.registerEventTrigger("start_intake", actions.deployAndToggleIntake())
        FollowPath.registerEventTrigger("end_intake", actions.stopIntake())
        FollowPath.registerEventTrigger("start_shooting", actions.autoTrenchShot())
        FollowPath.registerEventTrigger("start_shooting_hub", actions.autoHubShot())
        FollowPath.registerEventTrigger("stop_shooting", actions.stopFeedAndShooter())
    }

    fun rHalfClose(): Command {
        val path1 = Path("R_half_reg_pt1")
        val path2 = Path("R_half_reg_pt2")
        val path3 = Path("R_half_closer_pt1")
        val path4 = Path("R_half_closer_pt2")
        val path5 = Path("r_end")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            pathBuilderWithReset.build(path1),
            pathBuilderWithReset.build(path2),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilderWithReset.build(path3),
            pathBuilderWithReset.build(path4),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilderWithReset.build(path5),
        )
    }

    fun rHalfFar(): Command {
        val path1 = Path("R_half_close_pt1")
        val path2 = Path("R_half_close_pt2")
        val path3 = Path("R_half_far_pt1")
        val path4 = Path("R_half_far_pt2")
        val path5 = Path("r_end")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            pathBuilderWithReset.build(path3),
            pathBuilderWithReset.build(path4),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilderWithReset.build(path1),
            pathBuilderWithReset.build(path2),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilderWithReset.build(path5),
        )
    }

    fun rHalfAndLoop(): Command {
        val path1 = Path("R_half_reg_pt1")
        val path2 = Path("R_half_reg_pt2")
        val path3 = Path("R_loop_reg")
        val path4 = Path("l_end")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            pathBuilderWithReset.build(path1),
            pathBuilderWithReset.build(path2),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilderWithReset.build(path3),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilderWithReset.build(path4),
        )
    }

    fun lHalfClose(): Command {
        val path1 = Path("L_half_reg_pt1")
        val path2 = Path("L_half_reg_pt2")
        val path3 = Path("L_half_closer_pt1")
        val path4 = Path("L_half_closer_pt2")
        val path5 = Path("l_end")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            pathBuilderWithReset.build(path1),
            pathBuilderWithReset.build(path2),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilderWithReset.build(path3),
            pathBuilderWithReset.build(path4),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilderWithReset.build(path5),
        )
    }

    fun lHalfFar(): Command {
        val path1 = Path("L_half_close_pt1")
        val path2 = Path("L_half_close_pt2")
        val path3 = Path("L_half_far_pt1")
        val path4 = Path("L_half_far_pt2")
        val path5 = Path("l_end")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            pathBuilderWithReset.build(path3),
            pathBuilderWithReset.build(path4),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilderWithReset.build(path1),
            pathBuilderWithReset.build(path2),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilderWithReset.build(path5),
        )
    }

    fun lHalfAndLoop(): Command {
        val path1 = Path("L_half_reg_pt1")
        val path2 = Path("L_half_reg_pt2")
        val path3 = Path("L_loop_reg")
        val path4 = Path("r_end")

        eventTriggerCommands()

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            pathBuilderWithReset.build(path1),
            pathBuilderWithReset.build(path2),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilderWithReset.build(path3),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilderWithReset.build(path4),
        )
    }

    fun nothing(): Command = Commands.none()

    private fun preloadHubShot(): Command {
        val path1 = Path("hub_start")
        val path2 = Path("hub_end")

        eventTriggerCommands()

        return Commands.sequence(
            pathBuilderWithReset.build(path1),
            WaitCommand(AUTO_SHOOTING_TIME_SEC),
            pathBuilderWithReset.build(path2),
        )
    }

    fun addAutoOptions(autoChooser: LoggedDashboardChooser<Command>) {
        autoChooser.addDefaultOption("Do Nothing", nothing())
        autoChooser.addOption("Preload Hub", preloadHubShot())
        autoChooser.addOption("R Half Close", rHalfClose())
        autoChooser.addOption("R Half Far", rHalfFar())
        autoChooser.addOption("R Half Loop", rHalfAndLoop())
        autoChooser.addOption("L Half Close", lHalfClose())
        autoChooser.addOption("L Half Far", lHalfFar())
        autoChooser.addOption("L Half Loop", lHalfAndLoop())
    }
}
