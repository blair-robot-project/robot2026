package frc.team449.subsystems.vision

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger
import java.util.*
import kotlin.math.abs
import kotlin.math.pow

class Vision(
    visionConsumer: VisionConsumer,
    vararg visionIO: VisionIO
) : SubsystemBase() {
    private val consumer = visionConsumer
    private val io = visionIO
    val inputs = mutableListOf<VisionIOInputsAutoLogged>()
    private val disconnectedAlerts = mutableListOf<Alert>()

    init {
        for (i in 1..io.size) {
            inputs.add(VisionIOInputsAutoLogged())
        }

        for (i in 1..io.size) {
            disconnectedAlerts.add(Alert("Vision camera ${inputs[i - 1]} is disconnected.", AlertType.kWarning))
        }
    }

//    fun getTargetX(cameraIndex: Int): Rotation2d {
//        return inputs[cameraIndex]!!.latestTargetObservation.tx
//    }

    fun interface VisionConsumer {
        fun accept(
            visionRobotPoseMeters: Pose2d,
            timestampSeconds: Double,
            visionMeasurementStdDevs: Matrix<N3, N1>
        )
    }

    override fun periodic() {
        for (i in io.indices) {
            io[i].updateInputs(inputs[i])
            Logger.processInputs("Vision/Camera$i", inputs[i])
        }

        // Initialize logging values
        val allTagPoses: MutableList<Pose3d> = LinkedList()
        val allRobotPoses: MutableList<Pose3d> = LinkedList()
        val allRobotPosesAccepted: MutableList<Pose3d> = LinkedList()
        val allRobotPosesRejected: MutableList<Pose3d> = LinkedList()

        // Loop over cameras
        for (cameraIndex in io.indices) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected)

            // Initialize logging values
            val tagPoses: MutableList<Pose3d> = LinkedList()
            val robotPoses: MutableList<Pose3d> = LinkedList()
            val robotPosesAccepted: MutableList<Pose3d> = LinkedList()
            val robotPosesRejected: MutableList<Pose3d> = LinkedList()

            // Add tag poses
            for (tagId in inputs[cameraIndex].tagIds) {
                val tagPose = VisionConstants.aprilTagLayout.getTagPose(tagId)
                if (tagPose.isPresent) {
                    tagPoses.add(tagPose.get())
                }
            }

            // Loop over pose observations
            for (observation in inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                val rejectPose =
                    observation!!.tagCount == 0 || // Must have at least one tag
                        (
                            observation.tagCount == 1 &&
                                observation.ambiguity > VisionConstants.maxAmbiguity
                            ) || // Cannot be high ambiguity
                        (
                            abs(observation.pose.z)
                            > VisionConstants.maxZError
                            ) || // Must have realistic Z coordinate
                        // Must be within the field boundaries
                        observation.pose.x < 0.0 || observation.pose.x > VisionConstants.aprilTagLayout.fieldLength ||
                        observation.pose.y < 0.0 || observation.pose.y > VisionConstants.aprilTagLayout.fieldWidth

                robotPoses.add(observation.pose)
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose)
                } else {
                    robotPosesAccepted.add(observation.pose)
                }

                if (rejectPose) continue

                // Calculate standard deviations
                val stdDevFactor: Double =
                    observation.averageTagDistance.pow(2.0) / observation.tagCount
                var linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor
                var angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor
                if (observation.type == VisionIO.PoseObservationType.MEGATAG_2) {
                    linearStdDev *= VisionConstants.linearStdDevMegatag2Factor
                    angularStdDev *= VisionConstants.angularStdDevMegatag2Factor
                }
                if (cameraIndex < VisionConstants.cameraStdDevFactors.size) {
                    linearStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex]
                    angularStdDev *= VisionConstants.cameraStdDevFactors[cameraIndex]
                }

                // Send vision observation
                consumer.accept(
                    observation.pose.toPose2d(),
                    observation.timestamp,
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                )
            }

            // Log camera metadata
            Logger.recordOutput(
                "Vision/Camera$cameraIndex/TagPoses",
                *tagPoses.toTypedArray<Pose3d>()
            )
            Logger.recordOutput(
                "Vision/Camera$cameraIndex/RobotPoses",
                *robotPoses.toTypedArray<Pose3d>()
            )
            Logger.recordOutput(
                "Vision/Camera$cameraIndex/RobotPosesAccepted",
                *robotPosesAccepted.toTypedArray<Pose3d>()
            )
            Logger.recordOutput(
                "Vision/Camera$cameraIndex/RobotPosesRejected",
                *robotPosesRejected.toTypedArray<Pose3d>()
            )
            allTagPoses.addAll(tagPoses)
            allRobotPoses.addAll(robotPoses)
            allRobotPosesAccepted.addAll(robotPosesAccepted)
            allRobotPosesRejected.addAll(robotPosesRejected)
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", *allTagPoses.toTypedArray<Pose3d>())
        Logger.recordOutput("Vision/Summary/RobotPoses", *allRobotPoses.toTypedArray<Pose3d>())
        Logger.recordOutput(
            "Vision/Summary/RobotPosesAccepted",
            *allRobotPosesAccepted.toTypedArray<Pose3d>()
        )
        Logger.recordOutput(
            "Vision/Summary/RobotPosesRejected",
            *allRobotPosesRejected.toTypedArray<Pose3d>()
        )
    }
}
