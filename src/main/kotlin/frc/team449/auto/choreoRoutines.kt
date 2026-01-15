package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.team449.Robot
import frc.team449.RobotContainer.drive

open class choreoRoutines(
    robot: Robot) {
    val autoFactory = AutoFactory(
        drive::getPose,
        drive::resetOdometry,
        { sample: SwerveSample -> drive.followTrajectory(robot, sample) },
        true,
        drive
    )

    // do nothing
    fun doNothing(): AutoRoutine {
        val nothing: AutoRoutine = autoFactory.newRoutine("Nothing")
        return nothing
    }


    fun testing(): AutoRoutine {
        val test: AutoRoutine = autoFactory.newRoutine(" test")
        val path: AutoTrajectory = test.trajectory("test")
        test.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
                path.cmd()
            )
        )
        return test
    }


    fun forward(): AutoRoutine {
        val taxi: AutoRoutine = autoFactory.newRoutine(" Taxi")
        val path: AutoTrajectory = taxi.trajectory("forward")
        taxi.active().onTrue(
            Commands.sequence(
                path.resetOdometry(),
                path.cmd()
            )
        )
        return taxi
    }




    fun addOptions(autoChooser: AutoChooser) {
        autoChooser.addRoutine("test", this::testing)
        autoChooser.addRoutine("taxi", this::forward)
    }

}