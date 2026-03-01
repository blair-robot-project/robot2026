package frc.team449.subsystems.vision

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants
import org.littletonrobotics.junction.Logger
import kotlin.math.abs
import kotlin.math.pow

class VisionSubsystem(
    private val consumeVisionMeasurement: (visionRobotPoseMeters: Pose2d, timestampSeconds: Double, visionMeasurementStdDevs: Matrix<N3, N1>) -> Unit,
    private vararg val io: VisionIO
) : SubsystemBase() {

    init {
        val robottotag = Transform3d(0.419, 0.305, 1.13284, Rotation3d(0.0, 0.0, 0.0))
        val tagtocam = Transform3d(-0.32, 0.24, -0.81, Rotation3d(0.59, -0.459, 0.328))
        println("robot to cam: ${Transform3d(robottotag.toMatrix() * tagtocam.toMatrix())}")
        println("rotation ${Transform3d(robottotag.toMatrix() * tagtocam.toMatrix()).rotation.x}, y: ${Transform3d(robottotag.toMatrix() * tagtocam.toMatrix()).rotation.y}, z: ${Transform3d(robottotag.toMatrix() * tagtocam.toMatrix()).rotation.z}")

    }

    private val inputs = Array(io.size) { VisionIOInputsAutoLogged() }
    private val disconnectedAlerts = Array(io.size) { i ->
        Alert("Vision Camera $i Disconnected.", AlertType.kWarning)
    }

    fun getLatestTargetX(cameraIndex: Int): Rotation2d {
        val input = inputs[cameraIndex]
        if (input.tagIds.isEmpty()) return Rotation2d.kZero
        return input.latestTargetObservation.tx
    }

    fun getLatestTargetY(cameraIndex: Int): Rotation2d {
        val input = inputs[cameraIndex]
        if (input.tagIds.isEmpty()) return Rotation2d.kZero
        return input.latestTargetObservation.ty
    }

    override fun periodic() {
        val allRobotPosesAccepted = mutableListOf<Pose3d>()
        val allRobotPosesRejected = mutableListOf<Pose3d>()

        for (cameraIndex in io.indices) {
            io[cameraIndex].updateInputs(inputs[cameraIndex])
            Logger.processInputs("Vision/Camera$cameraIndex", inputs[cameraIndex])

            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected)

            val robotPosesAccepted = mutableListOf<Pose3d>()
            val robotPosesRejected = mutableListOf<Pose3d>()
            var observationID = 0

            for (observation in inputs[cameraIndex].poseObservations) {
                val rejectPose =
                    observation.tagCount == 0 ||
                        (observation.tagCount == 1 && observation.ambiguity > Constants.VisionConstants.maxAmbiguity) ||
                        abs(observation.pose.z) > Constants.VisionConstants.maxZError ||
                        observation.pose.x < 0.0 || observation.pose.x > Constants.VisionConstants.aprilTagLayout.fieldLength ||
                        observation.pose.y < 0.0 || observation.pose.y > Constants.VisionConstants.aprilTagLayout.fieldWidth

                if (rejectPose) {
                    robotPosesRejected.add(observation.pose)
                    continue
                }

                robotPosesAccepted.add(observation.pose)

                val stdDevFactor = observation.averageTagDistance.pow(2.0) / observation.tagCount
                var linearStdDev = Constants.VisionConstants.linearStdDevBaseline * stdDevFactor
                var angularStdDev = Constants.VisionConstants.angularStdDevBaseline * stdDevFactor

                if (observation.type == VisionIO.PoseObservationType.MEGATAG_2) {
                    linearStdDev *= Constants.VisionConstants.linearStdDevMegatag2Factor
                    angularStdDev *= Constants.VisionConstants.angularStdDevMegatag2Factor
                }
                if (cameraIndex < Constants.VisionConstants.cameraStdDevFactors.size) {
                    linearStdDev *= Constants.VisionConstants.cameraStdDevFactors[cameraIndex]
                    angularStdDev *= Constants.VisionConstants.cameraStdDevFactors[cameraIndex]
                }

                consumeVisionMeasurement(
                    observation.pose.toPose2d(),
                    observation.timestamp,
                    VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                )

                Logger.recordOutput("Vision/Camera$cameraIndex/Observation$observationID/LinearStandardDeviation", linearStdDev)
                Logger.recordOutput("Vision/Camera$cameraIndex/Observation$observationID/AngularStandardDeviation", angularStdDev)

                observationID++
            }

            Logger.recordOutput("Vision/Camera$cameraIndex/Yaw", inputs[cameraIndex].latestTargetObservation.tx)
            Logger.recordOutput("Vision/Camera$cameraIndex/RobotPosesAccepted", *robotPosesAccepted.toTypedArray())
            Logger.recordOutput("Vision/Camera$cameraIndex/RobotPosesRejected", *robotPosesRejected.toTypedArray())

            allRobotPosesAccepted.addAll(robotPosesAccepted)
            allRobotPosesRejected.addAll(robotPosesRejected)
        }

        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", *allRobotPosesAccepted.toTypedArray())
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", *allRobotPosesRejected.toTypedArray())
    }
}
