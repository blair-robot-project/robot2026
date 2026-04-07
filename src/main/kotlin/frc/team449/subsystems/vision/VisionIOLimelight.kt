package frc.team449.subsystems.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.wpilibj.Timer
import frc.team449.subsystems.vision.VisionIO.VisionIOInputs
import limelight.Limelight
import limelight.networktables.AngularVelocity3d
import limelight.networktables.LimelightPoseEstimator.EstimationMode
import limelight.networktables.LimelightResults
import limelight.networktables.LimelightSettings
import limelight.networktables.Orientation3d
import limelight.networktables.PoseEstimate
import java.util.Optional
import java.util.function.Supplier

class VisionIOLimelight(
    name: String,
    private val poseSupplier: () -> Pose2d,
    private val angularVelocitySupplier: Supplier<AngularVelocity3d>,
    offset: Pose3d
) : VisionIO {
    private val limelight = Limelight(name)
    private var estimationMode = EstimationMode.MEGATAG2
    private var poseObservationType = PoseObservationType.MEGATAG_2
    private val poseEstimator = limelight.createPoseEstimator(estimationMode)

    init {
        limelight.settings.withLimelightLEDMode(LimelightSettings.LEDMode.PipelineControl)
            .withCameraOffset(offset)
        limelight.settings.withImuMode(LimelightSettings.ImuMode.ExternalImu)
            .save()
    }

    override fun updateInputs(inputs: VisionIOInputs) {
        limelight.settings
            .withRobotOrientation(
                Orientation3d(
                    Rotation3d(poseSupplier().rotation),
                    angularVelocitySupplier.get(),
//                    AngularVelocity3d(
//                        DegreesPerSecond.of(0.0),
//                        DegreesPerSecond.of(0.0),
//                        DegreesPerSecond.of(0.0)
//                    )
                )
            )
            .save()

        val visionEstimateOpt: Optional<PoseEstimate> = poseEstimator.poseEstimate
        val resultsOpt: Optional<LimelightResults> = limelight.data.results

        if (visionEstimateOpt.isPresent) {
            val est = visionEstimateOpt.get()

            inputs.connected = (Timer.getFPGATimestamp() - est.timestampSeconds) < 0.25

            inputs.poseObservations = arrayOf(
                PoseObservation(
                    est.timestampSeconds,
                    est.pose,
                    est.avgTagAmbiguity,
                    est.tagCount,
                    est.avgTagDist,
                    poseObservationType,
                )
            )
        } else {
            inputs.connected = false
            inputs.poseObservations = emptyArray()
        }

        if (resultsOpt.isPresent) {
            val results = resultsOpt.get()

            if (results.targets_Fiducials.isNotEmpty()) {
                val bestTarget = results.targets_Fiducials[0]
                inputs.latestTargetObservation = TargetObservation(
                    Rotation2d.fromDegrees(bestTarget.tx),
                    Rotation2d.fromDegrees(bestTarget.ty)
                )
            } else {
                inputs.latestTargetObservation = TargetObservation(Rotation2d(), Rotation2d())
            }

            val tagIdsSet = mutableSetOf<Int>()
            for (target in results.targets_Fiducials) {
                tagIdsSet.add(target.fiducialID.toInt())
            }
            inputs.tagIds = tagIdsSet.toIntArray()
        } else {
            inputs.latestTargetObservation = TargetObservation(Rotation2d(), Rotation2d())
            inputs.tagIds = IntArray(0)
        }
    }
}
