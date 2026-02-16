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
            drive::getPose,
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
                drive::getPose,
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

    fun bump2cycle(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(Path("zigzag")), // 9.5
                // use the event triggers on the path
            ),
        )
        return routine
    }

    fun trench2cycleRight(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val path = Path("trench_1")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shoot", PrintCommand("Starting shoot"))
        FollowPath.registerEventTrigger("end_shoot", PrintCommand("Ending shoot"))

        routine.active().onTrue(
            pathBuilder.build(path),
        )

        return routine
    }

    fun trench2cycleLeft(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val path = Path("trench_3")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shoot", PrintCommand("Starting shoot"))
        FollowPath.registerEventTrigger("end_shoot", PrintCommand("Ending shoot"))

        routine.active().onTrue(
            pathBuilder.build(path),
        )

        return routine
    }

    fun trenchChuteRight(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val path = Path("trench_2")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("shoot", PrintCommand("Shooting"))
        FollowPath.registerEventTrigger("chute_intake", PrintCommand("Intaking from chute"))

        routine.active().onTrue(
            pathBuilder.build(path),
        )

        return routine
    }

    fun R_half_close(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("R_half_close")
        val path1 = Path("R_half_reg_pt1")
        val path2 = Path("R_half_reg_pt2")
        val path3 = Path("R_half_close_pt1")
        val path4 = Path("R_half_close_pt2")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shooting", PrintCommand("Shooting"))

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                // WaitCommand(3.5),
                PrintCommand("Stop shooting"),
                pathBuilder.build(path3),
                pathBuilder.build(path4),
                // WaitCommand(3.5),
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

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shooting", PrintCommand("Start Shooting"))

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                //WaitCommand(3.5),
                PrintCommand("Stop Shooting"),
                pathBuilder.build(path3),
                pathBuilder.build(path4),
                //WaitCommand(3.5),
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

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shooting", PrintCommand("Start Shooting"))

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                //WaitCommand(3.5),
                PrintCommand("Stop Shooting"),
                pathBuilder.build(path3),
                //WaitCommand(3.5),
                PrintCommand("Stop Shooting")
            )
        )


        return routine
    }

    fun R_half_thencloseloop(): AutoRoutine {
        val routine : AutoRoutine = autoFactory.newRoutine("R_half_thencloseloop")
        val path1 = Path("R_half_reg_pt1")
        val path2 = Path("R_half_reg_pt2")
        val path3 = Path("R_loop_close")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shooting", PrintCommand("Start Shooting"))

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                //WaitCommand(3.5),
                PrintCommand("Stop Shooting"),
                pathBuilder.build(path3),
                //WaitCommand(3.5),
                PrintCommand("Stop Shooting")
            )
        )

        return routine
    }

    fun R_half_thenfarloop(): AutoRoutine {
        val routine : AutoRoutine = autoFactory.newRoutine("R_half_thenfarloop")
        val path1 = Path("R_half_reg_pt1")
        val path2 = Path("R_half_reg_pt2")
        val path3 = Path("R_loop_far")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shooting", PrintCommand("Start Shooting"))

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                //WaitCommand(3.5),
                PrintCommand("Stop Shooting"),
                pathBuilder.build(path3),
                //WaitCommand(3.5),
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

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shooting", PrintCommand("Start Shooting"))

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                //WaitCommnad(3.5),
                PrintCommand("Stop shooting"),
                pathBuilder.build(path3),
                pathBuilder.build(path4),
                //WaitCommand(3.5)
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

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shooting", PrintCommand("Start Shooting"))

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                //WaitCommnad(3.5),
                PrintCommand("Stop shooting"),
                pathBuilder.build(path3),
                pathBuilder.build(path4),
                //WaitCommand(3.5)
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

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shooting", PrintCommand("Start shooting"))

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                //WaitCommand(3.5),
                PrintCommand("Stop Shooting"),
                pathBuilder.build(path3),
                //WaitCommand(3.5),
                PrintCommand("Stop Shooting")
            )
        )

        return routine
    }

    fun L_half_thencloseloop(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("L_half_thencloseloop")
        val path1 = Path("L_half_reg_pt1")
        val path2 = Path("L_half_reg_pt2")
        val path3 = Path("L_half_close")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shooting", PrintCommand("Start shooting"))

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                //WaitCommand(3.5),
                PrintCommand("Stop shooting"),
                pathBuilder.build(path3),
                //WaitCommand(3.5),
                PrintCommand("Stop Shooting")
            )
        )


        return routine
    }

    fun L_half_thenfarloop(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("L_half_thenfarloop")
        val path1: Path = Path("L_half_reg_pt1")
        val path2: Path = Path("L_half_reg_pt2")
        val path3 = Path("L_half_far")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shooting", PrintCommand("Start Shooting"))

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                pathBuilder.build(path2),
                //WaitCommand(3.5),
                PrintCommand("Stop shooting"),
                pathBuilder.build(path3),
                //WaitCommand(3.5),
                PrintCommand("Stop shooting")
            )
        )

        return routine
    }


    fun R_regloop_thencloseloop(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("R_regloop_thencloseloop")
        val path1: Path = Path("R_loop_reg")
        val path2: Path = Path("L_loop_close")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shooting", PrintCommand("Start shooting"))

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                //WaitCommand(3.5),
                PrintCommand("Start shooting"),
                pathBuilder.build(path2),
                //WaitCommand(3.0),
                PrintCommand("Stop shooting")
            )
        )

        return routine
    }

    fun R_regloop_thenfarloop(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("R_regloop_thenfarloop")
        val path1: Path = Path("R_loop_reg")
        val path2: Path = Path("L_loop_far")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shooting", PrintCommand("Start Shooting"))

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                //WaitCommand(3.5),
                PrintCommand("Stop shooting"),
                pathBuilder.build(path2),
                //WaitCommand(3.5),
                PrintCommand("Stop shooting")
            )
        )

        return routine
    }

    fun L_regloop_thencloseloop(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("L_regloop_thencloseloop")
        val path1: Path = Path("L_loop_reg")
        val path2: Path = Path("R_loop_close")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shooting", PrintCommand("Start Shooting"))

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                //WaitCommand(3.5),
                PrintCommand("Stop shooting"),
                pathBuilder.build(path2),
                //WaitCommand(3.5),
                PrintCommand("Stop shooting")
            )
        )

        return routine
    }


    fun L_regloop_thenfarloop(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("L_regloop_thenfarloop")
        val path1: Path = Path("L_loop_reg")
        val path2: Path = Path("R_loop_far")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("start_shooting", PrintCommand("Start shooting"))

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path1),
                //WaitCommand(3.5),
                PrintCommand("Stop Shooting"),
                pathBuilder.build(path2),
                //WaitCommand(3.5),
                PrintCommand("Stop Shooting")
            )
        )

        return routine
    }



    fun just_forward(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("just_forward")
        val path = Path("just_forward")

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(path),
            ),
        )
        return routine
    }

    fun test(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val a = Path("1")
        val b = Path("2")
        val c = Path("3")

        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(a),
                pathBuilder.build(b),
                pathBuilder.build(c),
            ),
        )

        return routine
    }

    fun test1(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto test1")
        val path = Path("one_samecycle_test_pt1")

        routine.active().onTrue(
            pathBuilder.build(path),
        )
        return routine
    }

    fun bumpsL(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        FollowPath.registerEventTrigger("intake", PrintCommand("Start intake"))
        FollowPath.registerEventTrigger("stop_intake", PrintCommand("Stop intake"))
        FollowPath.registerEventTrigger("fire", PrintCommand("Start fire"))
        FollowPath.registerEventTrigger("stop_fire", PrintCommand("Stop fire"))
        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(Path("bumptwoleft")),
            ),
        )
        return routine
    }

    fun bumpsR(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        FollowPath.registerEventTrigger("intake", PrintCommand("Start intake"))
        FollowPath.registerEventTrigger("stop_intake", PrintCommand("Stop intake"))
        FollowPath.registerEventTrigger("fire", PrintCommand("Start fire"))
        FollowPath.registerEventTrigger("stop_fire", PrintCommand("Stop fire"))
        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(Path("bumptworight")),
            ),
        )
        return routine
    }

    fun trenchBumpL(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        FollowPath.registerEventTrigger("intake", PrintCommand("Start intake"))
        FollowPath.registerEventTrigger("stop_intake", PrintCommand("Stop intake"))
        FollowPath.registerEventTrigger("fire", PrintCommand("Start fire"))
        FollowPath.registerEventTrigger("stop_fire", PrintCommand("Stop fire"))
        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(Path("trenchbumpleft")),
            ),
        )
        return routine
    }

    fun trenchBumpR(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        FollowPath.registerEventTrigger("intake", PrintCommand("Start intake"))
        FollowPath.registerEventTrigger("stop_intake", PrintCommand("Stop intake"))
        FollowPath.registerEventTrigger("fire", PrintCommand("Start fire"))
        FollowPath.registerEventTrigger("stop_fire", PrintCommand("Stop fire"))
        routine.active().onTrue(
            Commands.sequence(
                pathBuilder.build(Path("trenchbumpright")),
            ),
        )
        return routine
    }

    fun trenchSameR(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val path = Path("trench_5")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("shoot", PrintCommand("Shooting"))

        routine.active().onTrue(
            pathBuilder.build(path),
        )

        return routine
    }

    fun trenchSameL(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val path = Path("trench_7")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("shoot", PrintCommand("Shooting"))

        routine.active().onTrue(
            pathBuilder.build(path),
        )

        return routine
    }

    fun trenchDifferentR(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val path = Path("trench_4")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("shoot", PrintCommand("Shooting"))

        routine.active().onTrue(
            pathBuilder.build(path),
        )

        return routine
    }

    fun trenchDifferentL(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("auto")
        val path = Path("trench_6")

        FollowPath.registerEventTrigger("start_intake", PrintCommand("Starting intake"))
        FollowPath.registerEventTrigger("end_intake", PrintCommand("Ending intake"))
        FollowPath.registerEventTrigger("shoot", PrintCommand("Shooting"))

        routine.active().onTrue(
            pathBuilder.build(path),
        )

        return routine
    }

    fun addOptions(autoChooser: AutoChooser) {
        autoChooser.addRoutine("B-Line 2 cycle bump (R)", this::bump2cycle)
        // autoChooser.addRoutine("B-Line 2 cycle trench (R)", this::trench2cycle)
        // autoChooser.addRoutine("B-Line 1 cycle trench + chute cycle (R)", this::trench_chute)
        autoChooser.addRoutine("B-Line R half close", this::R_half_close)
        autoChooser.addRoutine("B-Line R half far", this::R_half_far)
        autoChooser.addRoutine("B-Line R half then reg loop", this::R_half_thenregloop)
        autoChooser.addRoutine("B-Line R half then close loop", this::R_half_thencloseloop)
        autoChooser.addRoutine("B-Line R half then far loop", this::R_half_thenfarloop)
        autoChooser.addRoutine("B-Line L half close", this::L_half_close)
        autoChooser.addRoutine("B-Line L half far", this::L_half_far)
        autoChooser.addRoutine("B-Line L half then reg loop", this::L_half_thenregloop)
        autoChooser.addRoutine("B-Line L half then close loop", this::L_half_thencloseloop)
        autoChooser.addRoutine("B-Line L half then far loop", this::L_half_thenfarloop)
        autoChooser.addRoutine("B-Line R reg loop then close loop", this::R_regloop_thencloseloop)
        autoChooser.addRoutine("B-Line R reg loop then far loop", this::R_regloop_thenfarloop)
        autoChooser.addRoutine("B-Line L reg loop then close loop", this::L_regloop_thencloseloop)
        autoChooser.addRoutine("B-Line L reg loop then far loop", this::L_regloop_thenfarloop)
        autoChooser.addRoutine("B-Line 2 cycle trench (R)", this::trench2cycleRight)
        autoChooser.addRoutine("B-Line 2 cycle trench (L)", this::trench2cycleLeft)
        autoChooser.addRoutine("B-Line 1 cycle trench + chute cycle (R)", this::trenchChuteRight)
        autoChooser.addRoutine("TEST", this::test)
        autoChooser.addRoutine("test 1", this::test1)
        autoChooser.addRoutine("just forward", this::just_forward)
        autoChooser.addRoutine("B-Line 2 cycle with trench 2bump then trench (R)", this::bumpsR)
        autoChooser.addRoutine("B-Line 2 cycle with trench 2bump then trench (L)", this::bumpsL)
        autoChooser.addRoutine("B-Line 2 cycle with trench bump  other side trench bump (L)", this::trenchBumpL)
        autoChooser.addRoutine("B-Line 2 cycle with trench bump  other side trench bump (R)", this::trenchBumpR)
        autoChooser.addRoutine("B-Line 2 cycle same trench auto (R)", this::trenchSameR)
        autoChooser.addRoutine("B-Line 2 cycle same trench auto (L)", this::trenchSameL)
        autoChooser.addRoutine("B-Line 2 cycle different trench auto (R)", this::trenchDifferentR)
        autoChooser.addRoutine("B-Line 2 cycle different trench auto (L)", this::trenchDifferentL)
    }
}
