package frc.team449.auto

import choreo.auto.AutoChooser
import choreo.auto.AutoFactory
import choreo.auto.AutoRoutine
import choreo.auto.AutoTrajectory
import choreo.trajectory.SwerveSample
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.team449.RobotContainer

class Routines(
    val robotContainer: RobotContainer,
    val characterizationMode: Boolean
) {
    private val autoFactory = AutoFactory(
        robotContainer.drive::getPose,
        robotContainer.drive::resetOdometry,
        { sample: SwerveSample -> robotContainer.drive.followTrajectory(sample) },
        true,
        robotContainer.drive
    )

    fun doNothing(): AutoRoutine {
        val nothing: AutoRoutine = autoFactory.newRoutine("Nothing")
        return nothing
    }

    fun testing(): AutoRoutine {
        val test: AutoRoutine = autoFactory.newRoutine("test")
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
        val taxi: AutoRoutine = autoFactory.newRoutine("Taxi")
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

        if (characterizationMode) {
            addCharacterizationOptions(autoChooser)
        }
    }

    fun addCharacterizationOptions(autoChooser: AutoChooser) {
        // drive characterization
        robotContainer.autoChooser.addCmd("Quasistatic Forward") { robotContainer.drive.sysIdRoutineTranslation.quasistatic(SysIdRoutine.Direction.kForward) }
        robotContainer.autoChooser.addCmd("Quasistatic Reverse") { robotContainer.drive.sysIdRoutineTranslation.quasistatic(SysIdRoutine.Direction.kReverse) }
        robotContainer.autoChooser.addCmd("Dynamic Forward") { robotContainer.drive.sysIdRoutineTranslation.dynamic(SysIdRoutine.Direction.kForward) }
        robotContainer.autoChooser.addCmd("Dynamic Reverse") { robotContainer.drive.sysIdRoutineTranslation.dynamic(SysIdRoutine.Direction.kReverse) }
    }
}
