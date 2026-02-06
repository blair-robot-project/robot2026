package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.team449.Robot
import frc.team449.RobotContainer.drive

open class ChoreoRoutines(
    robot: Robot,
) {
    val autoFactory =
        AutoFactory(
            drive::getPose,
            drive::resetOdometry,
            { sample: SwerveSample -> drive.followTrajectory(robot, sample) },
            true,
            drive,
        )

    fun doNothing(): AutoRoutine {
        val nothing: AutoRoutine = autoFactory.newRoutine("Nothing")
        return nothing
    }

    fun bl_trench_same(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("bl_trench_same")
        val path1: AutoTrajectory = routine.trajectory("bl_trench_pl1_pt1")
        val path2: AutoTrajectory = routine.trajectory("bl_pl1_trench_pt2")

        path1.atTime("start_shooting").onTrue(PrintCommand("Start Shooting"))
        path1.atTime("start_intake").onTrue(PrintCommand("Start Intake"))
        path1.atTime("stop_intake").onTrue(PrintCommand("Stop Intake"))
        path2.atTime("start_shooting").onTrue(PrintCommand("Start Shooting"))
        path2.atTime("stop_shooting").onTrue(PrintCommand("Stop Shooting"))
        path2.atTime("start_intake").onTrue(PrintCommand("Start Intake"))
        path2.atTime("stop_intake").onTrue(PrintCommand("Stop Intake"))

        routine.active().onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                path1.cmd(),
                // wait about 3-4 seconds for shooting
                path2.cmd(),
                // wait about 3-4 seconds for shooting
                PrintCommand("Stop Shooting"),
            ),
        )
        return routine
    }

    fun br_trench_same(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine("br_trench_same")
        val path1: AutoTrajectory = routine.trajectory("br_trench_pr1_pt1")
        val path2: AutoTrajectory = routine.trajectory("br_pr1_trench_pt2")

        path1.atTime("start_shooting").onTrue(PrintCommand("Start Shooting"))
        path1.atTime("start_intake").onTrue(PrintCommand("Start Intake"))
        path1.atTime("stop_intake").onTrue(PrintCommand("Stop Intake"))
        path2.atTime("start_shooting").onTrue(PrintCommand("Start Shooting"))
        path2.atTime("stop_shooting").onTrue(PrintCommand("Stop Shooting"))
        path2.atTime("start_intake").onTrue(PrintCommand("Start Intake"))
        path2.atTime("stop_intake").onTrue(PrintCommand("Stop Intake"))

        routine.active().onTrue(
            Commands.sequence(
                path1.resetOdometry(),
                path1.cmd(),
                // wait about 3-4 seconds for shooting
                path2.cmd(),
                // wait about 3-4 seconds for shooting
                PrintCommand("Stop Shooting"),
            ),
        )
        return routine
    }

    fun addOptions(autoChooser: AutoChooser) {
        autoChooser.addRoutine("bl_trench_same", this::bl_trench_same)
    }
}
