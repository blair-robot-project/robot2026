package frc.team449.auto

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.Pair
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.lib.BLine.FollowPath
import frc.robot.lib.BLine.Path
import frc.team449.Constants.AutoConstants.CTC_D
import frc.team449.Constants.AutoConstants.CTC_I
import frc.team449.Constants.AutoConstants.CTC_P
import frc.team449.Constants.AutoConstants.ROTATION_D
import frc.team449.Constants.AutoConstants.ROTATION_I
import frc.team449.Constants.AutoConstants.ROTATION_P
import frc.team449.Constants.AutoConstants.TRANSLATION_D
import frc.team449.Constants.AutoConstants.TRANSLATION_I
import frc.team449.Constants.AutoConstants.TRANSLATION_P
import frc.team449.Robot
import frc.team449.RobotContainer.actions
import frc.team449.RobotContainer.drive
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import java.util.function.Consumer

class BLineRoutines(
    robot: Robot
) {
    private val transP = LoggedNetworkNumber("Auto/Translation/P", TRANSLATION_P)
    private val transI = LoggedNetworkNumber("Auto/Translation/I", TRANSLATION_I)
    private val transD = LoggedNetworkNumber("Auto/Translation/D", TRANSLATION_D)

    private val rotP = LoggedNetworkNumber("Auto/Rotation/P", ROTATION_P)
    private val rotI = LoggedNetworkNumber("Auto/Rotation/I", ROTATION_I)
    private val rotD = LoggedNetworkNumber("Auto/Rotation/D", ROTATION_D)

    private val ctcP = LoggedNetworkNumber("Auto/CrossTrack/P", CTC_P)
    private val ctcI = LoggedNetworkNumber("Auto/CrossTrack/I", CTC_I)
    private val ctcD = LoggedNetworkNumber("Auto/CrossTrack/D", CTC_D)

    fun logBLineAuto() {
        translationController.setPID(transP.get(), transI.get(), transD.get())
        rotationController.setPID(rotP.get(), rotI.get(), rotD.get())
        crossTrackController.setPID(ctcP.get(), ctcI.get(), ctcD.get())

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
    }

    val translationController = PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D)
    val rotationController = PIDController(ROTATION_P, ROTATION_I, ROTATION_D)
    val crossTrackController = PIDController(CTC_P, CTC_I, CTC_D)

    var pathBuilderWithReset: FollowPath.Builder =
        FollowPath
            .Builder(
                drive,
                drive::pose,
                drive::getRobotRelativeSpeeds,
                { speeds: ChassisSpeeds ->
                    drive.setControl(
                        SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds),
                    )
                },
                translationController,
                rotationController,
                crossTrackController,
            ).withDefaultShouldFlip()
            .withPoseReset(drive::resetOdometry)

    var pathBuilder: FollowPath.Builder =
        FollowPath
            .Builder(
                drive,
                drive::pose,
                drive::getRobotRelativeSpeeds,
                { speeds: ChassisSpeeds ->
                    drive.setControl(
                        SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds),
                    )
                },
                translationController,
                rotationController,
                crossTrackController,
            ).withDefaultShouldFlip()

    fun eventTriggerCommands() {
        FollowPath.registerEventTrigger(
            "start_intake",
            Commands.parallel(
                actions.deployAndIntake(),
                actions.stopShooter()
            )
        )
        FollowPath.registerEventTrigger("end_intake", actions.stopIntake())
        FollowPath.registerEventTrigger(
            "start_shooting",
            actions.prepTrenchShot().andThen(actions.feed()),
        )
        FollowPath.registerEventTrigger("stop_shooting", actions.stopShooter())
    }

    fun rHalfClose(): Command {
        val path1 = Path("R_half_reg_pt1")
        val path2 = Path("R_half_reg_pt2")
        val path3 = Path("R_half_closer_pt1")
        val path4 = Path("R_half_closer_pt2")
        val path5 = Path("r_nothing")

        eventTriggerCommands()

        return Commands.sequence(
            pathBuilderWithReset.build(path1),
            pathBuilderWithReset.build(path2),
            WaitCommand(6.0),
            pathBuilderWithReset.build(path3),
            pathBuilderWithReset.build(path4),
            WaitCommand(6.0),
        )
    }

    fun rHalfFar(): Command {
        val path1 = Path("R_half_close_pt1")
        val path2 = Path("R_half_close_pt2")
        val path3 = Path("R_half_far_pt1")
        val path4 = Path("R_half_far_pt2")

        eventTriggerCommands()

        return Commands.sequence(
            pathBuilderWithReset.build(path3),
            pathBuilderWithReset.build(path4),
            WaitCommand(6.0),
            pathBuilderWithReset.build(path1),
            pathBuilderWithReset.build(path2),
            WaitCommand(6.0),
        )
    }

    fun rHalfAndLoop(): Command {
        val path1 = Path("R_half_reg_pt1")
        val path2 = Path("R_half_reg_pt2")
        val path3 = Path("R_loop_reg")

        eventTriggerCommands()

        return Commands.sequence(
            pathBuilderWithReset.build(path1),
            pathBuilderWithReset.build(path2),
            WaitCommand(6.0),
            pathBuilderWithReset.build(path3),
            WaitCommand(6.0),
        )
    }

    fun lHalfClose(): Command {
        val path1 = Path("L_half_reg_pt1")
        val path2 = Path("L_half_reg_pt2")
        val path3 = Path("L_half_closer_pt1")
        val path4 = Path("L_half_closer_pt2")

        eventTriggerCommands()

        return Commands.sequence(
            pathBuilderWithReset.build(path1),
            pathBuilderWithReset.build(path2),
            WaitCommand(6.0),
            pathBuilderWithReset.build(path3),
            pathBuilderWithReset.build(path4),
            WaitCommand(6.0),
        )
    }

    fun lHalfFar(): Command {
        val path1 = Path("L_half_close_pt1")
        val path2 = Path("L_half_close_pt2")
        val path3 = Path("L_half_far_pt1")
        val path4 = Path("L_half_far_pt2")

        eventTriggerCommands()

        return Commands.sequence(
            pathBuilderWithReset.build(path3),
            pathBuilderWithReset.build(path4),
            WaitCommand(6.0),
            pathBuilderWithReset.build(path1),
            pathBuilderWithReset.build(path2),
            WaitCommand(6.0),
        )
    }

    fun lHalfAndLoop(): Command {
        val path1 = Path("L_half_reg_pt1")
        val path2 = Path("L_half_reg_pt2")
        val path3 = Path("L_loop_reg")

        eventTriggerCommands()

        return Commands.sequence(
            pathBuilderWithReset.build(path1),
            pathBuilderWithReset.build(path2),
            WaitCommand(6.0),
            pathBuilderWithReset.build(path3),
            WaitCommand(6.0),
        )
    }

    fun nothing(): Command = Commands.none()

    fun addAutoOptions(autoChooser: SendableChooser<Command>) {
        autoChooser.setDefaultOption("Do Nothing", nothing())
        autoChooser.addOption("R Half Close", rHalfClose())
        autoChooser.addOption("R Half Far", rHalfFar())
        autoChooser.addOption("R Half Loop", rHalfAndLoop())
        autoChooser.addOption("L Half Close", lHalfClose())
        autoChooser.addOption("L Half Far", lHalfFar())
        autoChooser.addOption("L Half Loop", lHalfAndLoop())
    }
}
