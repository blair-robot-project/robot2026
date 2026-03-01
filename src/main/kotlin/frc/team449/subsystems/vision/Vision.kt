package frc.team449.subsystems.vision

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants
import org.littletonrobotics.junction.Logger
import kotlin.math.abs
import kotlin.math.pow

class Vision(
    private val consumer: VisionConsumer,
    private vararg val io: VisionIO
) : SubsystemBase() {
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

    fun interface VisionConsumer {
        fun accept(
            visionRobotPoseMeters: Pose2d,
            timestampSeconds: Double,
            visionMeasurementStdDevs: Matrix<N3, N1>
        )
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

                consumer.accept(
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
