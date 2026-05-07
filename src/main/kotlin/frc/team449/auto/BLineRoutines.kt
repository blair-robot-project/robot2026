package frc.team449.auto

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Rotation2d
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

class BLineRoutines(
    private val drive: DriveSubsystem,
    private val actions: RobotActions
) {
    private val translationController = PIDController(AutoConstants.TRANSLATION_P, 0.0, AutoConstants.TRANSLATION_D)
    private val rotationController = PIDController(AutoConstants.ROTATION_P, 0.0, AutoConstants.ROTATION_D)
    private val crossTrackController = PIDController(AutoConstants.CTE_P, 0.0, AutoConstants.CTE_D)

    private val applyRobotSpeedsRequest = SwerveRequest.ApplyRobotSpeeds()

    private val baseBuilder =
        FollowPath.Builder(
            drive,
            drive::pose,
            drive::robotRelativeSpeeds,
            { speeds: ChassisSpeeds ->
                drive.setControl(applyRobotSpeedsRequest.withSpeeds(speeds))
            },
            translationController,
            rotationController,
            crossTrackController,
        )
            .withDefaultShouldFlip()

    private fun registerEventTriggers() {
        FollowPath.registerEventTrigger("start_intake", actions.deployAndIntake())
        FollowPath.registerEventTrigger("stop_intake", actions.stopIntake())
        FollowPath.registerEventTrigger("trench_shot", actions.autoShot(3.43))
        FollowPath.registerEventTrigger("hub_shot", actions.autoShot(1.3))
        FollowPath.registerEventTrigger("bump_shot", actions.autoShot(2.22))
        FollowPath.registerEventTrigger("depot_shot", actions.autoShot(3.7))
        FollowPath.registerEventTrigger("stop_shot", actions.stopAll())
    }

    fun configurePathFollowerLogging() {
        FollowPath.setPoseLoggingConsumer { pair ->
            Logger.recordOutput(pair.first, pair.second)
        }

        FollowPath.setTranslationListLoggingConsumer { pair ->
            Logger.recordOutput(pair.first, *pair.second)
        }

        FollowPath.setDoubleLoggingConsumer { pair ->
            Logger.recordOutput(pair.first, pair.second)
        }

        FollowPath.setBooleanLoggingConsumer { pair ->
            Logger.recordOutput(pair.first, pair.second)
        }
    }

    init {
        registerEventTriggers()
        configurePathFollowerLogging()
    }

    private fun nothing() = Commands.none()

    private fun doubleTrench(mirror: Boolean): Command {
        val resetBuilder = baseBuilder.withPoseReset(drive::resetOdometry).withShouldMirror { mirror }
        val standardBuilder = baseBuilder.withShouldMirror { mirror }

        return Commands
            .sequence(
                drive.alignModules(Rotation2d.kCW_90deg),
                resetBuilder.build(Path("trench_pt1")),
                standardBuilder.build(Path("trench_pt2")),
                WaitCommand(AutoConstants.AUTO_SHOOTING_TIME_SEC),
                standardBuilder.build(Path("trench_pt3")),
                standardBuilder.build(Path("trench_pt4")),
                WaitCommand(AutoConstants.AUTO_SHOOTING_TIME_SEC),
                standardBuilder.build(Path("trench_to_nz")),
            )
            .withName("DoubleTrench")
    }

    private fun doubleBumpSweep(mirror: Boolean): Command {
        val resetBuilder = baseBuilder.withPoseReset(drive::resetOdometry).withShouldMirror { mirror }
        val standardBuilder = baseBuilder.withShouldMirror { mirror }

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            resetBuilder.build(Path("trench_bump_pt1")),
            WaitCommand(AutoConstants.AUTO_SHOOTING_TIME_SEC),
            standardBuilder.build(Path("trench_bump_pt2")),
            WaitCommand(AutoConstants.AUTO_SHOOTING_TIME_SEC),
            standardBuilder.build(Path("bump_to_nz")),
        )
            .withName("DoubleBumpSweep")
    }

    private fun delayAuto(mirror: Boolean): Command {
        val resetBuilder = baseBuilder.withPoseReset(drive::resetOdometry).withShouldMirror { mirror }
        val standardBuilder = baseBuilder.withShouldMirror { mirror }

        return Commands.sequence(
            drive.alignModules(Rotation2d.kCW_90deg),
            resetBuilder.build(Path("to_trench")),
            WaitCommand(AutoConstants.AUTO_PRELOAD_SHOOTING_TIME_SEC),
            standardBuilder.build(Path("delay_trench_bump")),
            WaitCommand(AutoConstants.AUTO_SHOOTING_TIME_SEC),
            WaitCommand(2.0), // check with alliance partners
            standardBuilder.build(Path("trench_to_nz")),
        )
            .withName("DoubleTrench")
    }

    private fun hubDepot(mirror: Boolean): Command {
        val resetBuilder = baseBuilder.withPoseReset(drive::resetOdometry).withShouldMirror { mirror }
        val standardBuilder = baseBuilder.withShouldMirror { mirror }

        return Commands.sequence(
            resetBuilder.build(Path("hub_start")),
            WaitCommand(AutoConstants.AUTO_PRELOAD_SHOOTING_TIME_SEC),
            standardBuilder.build(Path("hub_end")),
            standardBuilder.build(Path("hub_depot")),
            WaitCommand(AutoConstants.AUTO_SHOOTING_TIME_SEC),
            standardBuilder.build(Path("depot_to_nz"))
        )
            .withName("HubDepot")
    }

    fun addOptionsToChooser(autoChooser: LoggedDashboardChooser<Command>) {
        autoChooser.addDefaultOption("Do Nothing", Commands.none())

        fun addMirroredOptions(name: String, buildRoutine: (mirror: Boolean) -> Command) {
            autoChooser.addOption("R $name", buildRoutine(false))
            autoChooser.addOption("L $name", buildRoutine(true))
        }

        addMirroredOptions("Double Bump Sweep", ::doubleBumpSweep)
        addMirroredOptions("Double Trench", ::doubleTrench)
        addMirroredOptions("Delayed Auto", ::delayAuto)

        // one-side autos
        autoChooser.addOption("Hub Depot NZ", hubDepot(false))
    }
}
