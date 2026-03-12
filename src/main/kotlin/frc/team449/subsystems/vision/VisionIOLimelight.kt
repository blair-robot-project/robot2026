package frc.team449.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.wpilibj.Timer
import frc.team449.subsystems.vision.VisionIO.PoseObservation
import frc.team449.subsystems.vision.VisionIO.PoseObservationType
import frc.team449.subsystems.vision.VisionIO.TargetObservation
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

/** IO implementation for real Limelight hardware.  */
/**
 * Creates a new VisionIOLimelight.
 *
 * @param name The configured name of the Limelight.
 * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
 */
class VisionIOLimelight(
    name: String,
    private val rotationSupplier: Supplier<Rotation2d>,
    offset: Pose3d
) : VisionIO {
    private val limelight = Limelight(name)
    private var estimationMode = EstimationMode.MEGATAG1 // can change this if wanna run both megatag1 and 2
    private var poseObservationType = PoseObservationType.MEGATAG_1
    private val poseEstimator = limelight.createPoseEstimator(estimationMode)

    private val logsEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2)

    init {
        limelight.settings.withLimelightLEDMode(LimelightSettings.LEDMode.PipelineControl)
            .withCameraOffset(offset).save()
        limelight.settings.withImuMode(LimelightSettings.ImuMode.InternalImuExternalAssist)
    }

    override fun updateInputs(inputs: VisionIOInputs) {
        limelight.settings
            .withRobotOrientation(
                Orientation3d(
                    Rotation3d(rotationSupplier.get()),
                    AngularVelocity3d(
                        DegreesPerSecond.of(0.0),
                        DegreesPerSecond.of(0.0),
                        DegreesPerSecond.of(0.0),
                    )
                )
            )
            .save()

        NetworkTableInstance.getDefault().flush()

        val visionEstimateOpt: Optional<PoseEstimate> = poseEstimator.poseEstimate

        val logsEstimateOpt: Optional<PoseEstimate> = logsEstimator.poseEstimate

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
                    PoseObservationType.MEGATAG_1
                )
            )
        } else {
            inputs.connected = false
            inputs.poseObservations = emptyArray()
        }

        if (visionEstimateOpt.isPresent) {
            val logsest = logsEstimateOpt.get()

            inputs.logsObservations = arrayOf(
                PoseObservation(
                    logsest.timestampSeconds,
                    logsest.pose,
                    logsest.avgTagAmbiguity,
                    logsest.tagCount,
                    logsest.avgTagDist,
                    PoseObservationType.MEGATAG_2
                )
            )
        } else {
            inputs.logsObservations = emptyArray()
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
