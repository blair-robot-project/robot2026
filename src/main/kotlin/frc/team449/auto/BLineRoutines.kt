package frc.team449.auto
import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.trajectory.SwerveSample
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.Pair
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
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
import frc.team449.RobotContainer.drive
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import java.util.function.Consumer

class BLineRoutines(
    robot: Robot
) {
    val autoFactory =
        AutoFactory(
            drive::pose,
            drive::resetOdometry,
            { sample: SwerveSample -> drive.followTrajectory(robot, sample) },
            true,
            drive,
        )

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
                Logger.recordOutput<Translation2d>(pair.getFirst(), *pair.getSecond())
            },
        )

        FollowPath.setDoubleLoggingConsumer { pair ->
            Logger.recordOutput(pair.first, pair.second)
        }
    }

    val translationController: PIDController = PIDController(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D)
    val rotationController: PIDController = PIDController(ROTATION_P, ROTATION_I, ROTATION_D)
    val crossTrackController: PIDController = PIDController(CTC_P, CTC_I, CTC_D)

    var pathBuilder: FollowPath.Builder =
        FollowPath
            .Builder(
                drive,
                drive::pose,
                drive::getRobotRelativeSpeeds,
                { speeds: ChassisSpeeds ->
                    Logger.recordOutput("Bline speed", speeds)
                    drive.setControl(
                        SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds),
                    )
                },
                translationController,
                rotationController,
                crossTrackController,
            ).withDefaultShouldFlip()
            .withPoseReset(drive::resetOdometry)

    fun eventTriggerCommands() {
        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shooting", PrintCommand("Start shooting"))
    }

    fun R_half_close(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("R_half_close")
        val path1 = Path("R_half_reg_pt1")
        val path2 = Path("R_half_reg_pt2")
        val path3 = Path("R_half_close_pt1")
        val path4 = Path("R_half_close_pt2")

        eventTriggerCommands()

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                WaitCommand(6.0),
                PrintCommand("Stop shooting"),
                pathBuilder.build(path3),
                pathBuilder.build(path4),
                WaitCommand(6.0),
                PrintCommand("Stop shooting"),
            ),
        )

        return routine
    }

    fun R_half_far(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("R_half_far")
        val path1 = Path("R_half_reg_pt1")
        val path2 = Path("R_half_reg_pt2")
        val path3 = Path("R_half_far_pt1")
        val path4 = Path("R_half_far_pt2")

        eventTriggerCommands()

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                WaitCommand(6.0),
                PrintCommand("Stop Shooting"),
                pathBuilder.build(path3),
                pathBuilder.build(path4),
                WaitCommand(6.0),
                PrintCommand("Stop Shooting")
            )
        )

        return routine
    }

    fun R_half_thenregloop(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("R_half_thenregloop")
        val path1 = Path("R_half_reg_pt1")
        val path2 = Path("R_half_reg_pt2")
        val path3 = Path("R_loop_reg")

        eventTriggerCommands()

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                WaitCommand(6.0),
                PrintCommand("Stop Shooting"),
                pathBuilder.build(path3),
                WaitCommand(6.0),
                PrintCommand("Stop Shooting")
            )
        )


        return routine
    }

    fun L_half_close(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("L_half_close")
        val path1 = Path("L_half_reg_pt1")
        val path2 = Path("L_half_reg_pt2")
        val path3 = Path("L_half_close_pt1")
        val path4 = Path("L_half_close_pt2")

        eventTriggerCommands()

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                WaitCommand(6.0),
                PrintCommand("Stop shooting"),
                pathBuilder.build(path3),
                pathBuilder.build(path4),
                WaitCommand(6.0),
                PrintCommand("Stop shooting")
            )
        )

        return routine
    }

    fun L_half_far(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("L_half_far")
        val path1 = Path("L_half_reg_pt1")
        val path2 = Path("L_half_reg_pt2")
        val path3 = Path("L_half_far_pt1")
        val path4 = Path("L_half_far_pt2")

        eventTriggerCommands()

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path3),
                pathBuilder.build(path4),
                WaitCommand(6.0),
                PrintCommand("Stop shooting"),
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                WaitCommand(6.0),
                PrintCommand("Stop shooting")
            )
        )

        return routine
    }

    fun L_half_thenregloop(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("L_half_thenregloop")
        val path1 = Path("L_half_reg_pt1")
        val path2 = Path("L_half_reg_pt2")
        val path3 = Path("L_loop_reg")

        eventTriggerCommands()

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                WaitCommand(6.0),
                PrintCommand("Stop Shooting"),
                pathBuilder.build(path3),
                WaitCommand(6.0),
                PrintCommand("Stop Shooting")
            )
        )

        return routine
    }

    fun trenchSameR(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val path1 = Path("R_trench_start")
        val path2 = Path("R_trench_beyond")

        eventTriggerCommands()

        routine.active().onTrue(
            Commands.sequence(pathBuilder.build(path1), pathBuilder.build(path2))
        )

        return routine
    }

    fun trenchSameL(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val path1 = Path("L_trench_start")
        val path2 = Path("L_trench_beyond")

        eventTriggerCommands()

        routine.active().onTrue(
            Commands.sequence(pathBuilder.build(path1), pathBuilder.build(path2))
        )

        return routine
    }

    fun trenchDifferentR(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val path1 = Path("R_trench_start")
        val path2 = Path("R_trench_across")

       eventTriggerCommands()

        routine.active().onTrue(
            Commands.sequence(pathBuilder.build(path1), pathBuilder.build(path2))
        )

        return routine
    }

    fun trenchDifferentL(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val path1 = Path("L_trench_start")
        val path2 = Path("L_trench_across")

        eventTriggerCommands()

        routine.active().onTrue(
            Commands.sequence(pathBuilder.build(path1), pathBuilder.build(path2))
        )

        return routine
    }

    fun addOptions(autoChooser: AutoChooser) {
        autoChooser.addRoutine("B-Line R half close", this::R_half_close)
        autoChooser.addRoutine("B-Line R half far", this::R_half_far)
        autoChooser.addRoutine("B-Line R half then reg loop", this::R_half_thenregloop)
        autoChooser.addRoutine("B-Line L half close", this::L_half_close)
        autoChooser.addRoutine("B-Line L half far", this::L_half_far)
        autoChooser.addRoutine("B-Line L half then reg loop", this::L_half_thenregloop)

        autoChooser.addRoutine("B-Line 2 cycle same trench auto (R)", this::trenchSameR)
        autoChooser.addRoutine("B-Line 2 cycle same trench auto (L)", this::trenchSameL)
        autoChooser.addRoutine("B-Line 2 cycle different trench auto (R)", this::trenchDifferentR)
        autoChooser.addRoutine("B-Line 2 cycle different trench auto (L)", this::trenchDifferentL)
    }
}
