// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.team449.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.units.Units.DegreesPerSecond
import edu.wpi.first.wpilibj.RobotController
import frc.team449.Constants
import frc.team449.RobotContainer.drive
import frc.team449.subsystems.vision.VisionIO.*
import limelight.Limelight
import limelight.networktables.*
import limelight.networktables.LimelightPoseEstimator.EstimationMode
import java.util.*
import java.util.function.Supplier
import kotlin.math.pow

/** IO implementation for real Limelight hardware.  */
/**
 * Creates a new VisionIOLimelight.
 *
 * @param name The configured name of the Limelight.
 * @param rotationSupplier Supplier for the current estimated rotation, used for MegaTag 2.
 */
class VisionIOLimelight(
    name: String,
    private val rotationSupplier: Supplier<Rotation2d>
) : VisionIO {
    private val limelight = Limelight(name)
    private var estimationMode = EstimationMode.MEGATAG2 // can change this if wanna run both megatag1 and 2
    private var poseObservationType = PoseObservationType.MEGATAG_2 // TODO() this is lowk stupid

    fun configure(offset: Pose3d): VisionIOLimelight {
        limelight.settings.withLimelightLEDMode(LimelightSettings.LEDMode.PipelineControl)
            .withCameraOffset(offset)
            .save()
        return this
    }

    init {
        limelight.settings
            .withRobotOrientation(
                Orientation3d(
                    Rotation3d(Rotation2d(drive.inputs.gyroAngle)),
                    AngularVelocity3d(
                        DegreesPerSecond.of(drive.inputs.pitchVel),
                        DegreesPerSecond.of(drive.inputs.rollVel),
                        DegreesPerSecond.of(drive.inputs.yawVel),
                    )
                )
            )
            .save()
        limelight.settings.withImuMode(LimelightSettings.ImuMode.InternalImuExternalAssist)
    }

    override fun updateInputs(inputs: VisionIOInputs) {
        val poseObservations: MutableList<PoseObservation> = LinkedList()

        val visionEstimate: Optional<PoseEstimate> =
            limelight.createPoseEstimator(estimationMode).poseEstimate

        visionEstimate.ifPresent { poseEstimate: PoseEstimate ->
            poseObservations.add(
                PoseObservation(
                    poseEstimate.timestampSeconds,
                    poseEstimate.pose,
                    poseEstimate.avgTagAmbiguity,
                    poseEstimate.tagCount,
                    poseEstimate.avgTagDist,
                    poseObservationType
                )
            )
        }

        // Save pose observations to inputs object
        inputs.poseObservations = poseObservations.toTypedArray()

        inputs.latestLatency = visionEstimate.get().latency
        inputs.latestTimestamp = visionEstimate.get().timestampSeconds
        inputs.latestPose = visionEstimate.get().pose
        inputs.latestAverageTagAmbiguity = visionEstimate.get().avgTagAmbiguity
        inputs.latestMinTagAmbiguity = visionEstimate.get().minTagAmbiguity
        inputs.latestMaxTagAmbiguity = visionEstimate.get().maxTagAmbiguity
        inputs.latestTagCount = visionEstimate.get().tagCount
        inputs.latestAverageTagDist = visionEstimate.get().avgTagDist
        inputs.orientation = limelight.data.results.get().imuResults.yaw

        inputs.stdDevFactor =
            inputs.latestAverageTagDist.pow(2.0) / inputs.latestTagCount
        inputs.linearStdDev = Constants.VisionConstants.linearStdDevBaseline * inputs.stdDevFactor
        inputs.angularStdDev = Constants.VisionConstants.angularStdDevBaseline * inputs.stdDevFactor
        inputs.linearStdDev *= Constants.VisionConstants.linearStdDevMegatag2Factor
        inputs.angularStdDev *= Constants.VisionConstants.angularStdDevMegatag2Factor

        // Update connection status based on whether an update has been seen in the last 250ms
        inputs.connected = ((RobotController.getFPGATime() - inputs.latestLatency) / 1000) < 250

        inputs.numFiducials = limelight.data.results.get().targets_Fiducials.size

        inputs.tagIds = DoubleArray(inputs.numFiducials)
        inputs.tx = DoubleArray(inputs.numFiducials)
        inputs.ty = DoubleArray(inputs.numFiducials)
        inputs.targetObservations = Array<Pose3d>(inputs.numFiducials) { Pose3d() }

        for (i in 1..inputs.numFiducials) {
            inputs.tagIds[i - 1] = limelight.data.results.get().targets_Fiducials[i - 1].fiducialID
            inputs.tx[i - 1] = limelight.data.results.get().targets_Fiducials[i - 1].tx
            inputs.ty[i - 1] = limelight.data.results.get().targets_Fiducials[i - 1].ty // limelight.latestResults.get().targets_Fiducials
            inputs.targetObservations[i - 1] = (limelight.data.results.get().targets_Fiducials[i - 1].targetPose_RobotSpace)
        }

        limelight.settings
            .withRobotOrientation(
                Orientation3d(
                    Rotation3d(Rotation2d(drive.inputs.gyroAngle)),
                    AngularVelocity3d(
                        DegreesPerSecond.of(drive.inputs.pitchVel),
                        DegreesPerSecond.of(drive.inputs.rollVel),
                        DegreesPerSecond.of(drive.inputs.yawVel),
                    )
                )
            )
            .save()
    }
}
