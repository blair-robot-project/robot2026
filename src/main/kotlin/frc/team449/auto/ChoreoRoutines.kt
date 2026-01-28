package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj2.command.Commands
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

    // do nothing
    fun doNothing(): AutoRoutine {
        val nothing: AutoRoutine = autoFactory.newRoutine("Nothing")
        return nothing
    }

    fun forward(): AutoRoutine {
        val taxi: AutoRoutine = autoFactory.newRoutine(" Taxi")
        val path: AutoTrajectory = taxi.trajectory("forward")
        taxi.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
                path.cmd(),
            ),
        )
        return taxi
    }

    fun twoCycle(): AutoRoutine {
        val taxi: AutoRoutine = autoFactory.newRoutine(" Taxi")
        val a: AutoTrajectory = taxi.trajectory("leftintake")
        val b: AutoTrajectory = taxi.trajectory("leftback")
        val c: AutoTrajectory = taxi.trajectory("leftintake2")
        val d: AutoTrajectory = taxi.trajectory("leftback2")
        taxi.active().onTrue(
            Commands.sequence(
                a.resetOdometry(),
                a.cmd(),
                b.cmd(),
                c.cmd(),
                d.cmd(),
            ),
        )
        return taxi
    }

    fun round(): AutoRoutine {
        val routine: AutoRoutine = autoFactory.newRoutine(" auto")
        val path: AutoTrajectory = routine.trajectory("spiny")
        routine.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
                path.cmd(),
            ),
        )
        return routine
    }

    fun addOptions(autoChooser: AutoChooser) {
        autoChooser.addRoutine("taxi", this::forward)
        autoChooser.addRoutine("!spin fuel test!", this::round)
        autoChooser.addRoutine("choreo 2 cycle (R)", this::twoCycle)
    }
}
