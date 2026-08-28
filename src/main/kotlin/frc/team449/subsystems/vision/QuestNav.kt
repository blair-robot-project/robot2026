package frc.team449.subsystems.vision

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants
import gg.questnav.questnav.QuestNav
import org.littletonrobotics.junction.Logger

class QuestNav(
    val offset: Transform3d,
    private val consumeVisionMeasurement: (visionRobotPoseMeters: Pose2d, timestampSeconds: Double, visionMeasurementStdDevs: Matrix<N3, N1>) -> Unit
) : SubsystemBase() {
    val questNav = QuestNav()

    override fun periodic() {
        questNav.commandPeriodic()

        Logger.recordOutput("QuestNav/isConnected", questNav.isConnected)
        Logger.recordOutput("QuestNav/isTracking", questNav.isTracking)
        Logger.recordOutput("QuestNav/latency", questNav.latency)
        questNav.batteryPercent.ifPresent { b ->
            Logger.recordOutput("QuestNav/batteryPercentage", b)
        }
        questNav.trackingLostCounter.ifPresent { c ->
            Logger.recordOutput("QuestNav/trackingLostCounter", c)
        }
        val acceptedPoses = mutableListOf<Pose3d>()
        val rejectedPoses = mutableListOf<Pose3d>()

        for (frame in questNav.getAllUnreadPoseFrames()) {
            if (frame.isTracking) {
                val robotPose =
                    frame
                        .questPose3d()
                        .transformBy(offset.inverse())

                if (
                    robotPose.x > Constants.FieldConstants.FIELD_WIDTH_METERS &&
                    robotPose.x < 0 &&
                    robotPose.y > Constants.FieldConstants.FIELD_LENGTH_METERS &&
                    robotPose.y < 0 &&
                    robotPose.z > Constants.VisionConstants.MAX_Z_ERROR_METERS
                ) {
                    rejectedPoses.add(robotPose)
                    continue
                }
                acceptedPoses.add(robotPose)
                consumeVisionMeasurement(
                    robotPose.toPose2d(),
                    frame.dataTimestamp(),
                    Constants.VisionConstants.QUESTNAV_STD_DEVS,
                )
            }
        }
        Logger.recordOutput("QuestNav/RejectedPoses", *rejectedPoses.toTypedArray())
        Logger.recordOutput("QuestNav/AcceptedPoses", *acceptedPoses.toTypedArray())
    }
}
