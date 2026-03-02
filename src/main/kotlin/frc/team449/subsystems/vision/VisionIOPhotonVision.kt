// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.team449.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import frc.team449.Constants.VisionConstants.aprilTagLayout
import frc.team449.subsystems.vision.VisionIO.*
import org.photonvision.PhotonCamera
import java.util.*

/** IO implementation for real PhotonVision hardware.  */
/**
 * Creates a new VisionIOPhotonVision.
 *
 * @param name The configured name of the camera.
 * @param robotToCamera The 3D position of the camera relative to the robot.
 */
open class VisionIOPhotonVision(
    name: String,
    private val robotToCamera: Transform3d
) : VisionIO {
    protected val camera: PhotonCamera = PhotonCamera(name)

    override fun updateInputs(inputs: VisionIOInputs) {
        inputs.connected = camera.isConnected

        val tagIds = mutableSetOf<Int>()

        val poseObservations = mutableListOf<PoseObservation>()

        for (result in camera.allUnreadResults) {
            // update latest target observation
            if (result.hasTargets()) {
                inputs.latestTargetObservation = TargetObservation(
                    Rotation2d.fromDegrees(result.bestTarget.yaw),
                    Rotation2d.fromDegrees(result.bestTarget.pitch)
                )
            } else {
                inputs.latestTargetObservation = TargetObservation(Rotation2d(), Rotation2d())
            }

            // add pose observation
            if (result.multitagResult.isPresent) {
                val multitagResult = result.multitagResult.get()

                // calculate robot pose
                val fieldToCamera: Transform3d = multitagResult.estimatedPose.best
                val fieldToRobot = fieldToCamera.plus(robotToCamera.inverse())
                val robotPose = Pose3d(fieldToRobot.translation, fieldToRobot.rotation)

                // calculate average tag distance and collect IDs safely
                var totalTagDistance = 0.0
                for (target in result.targets) {
                    totalTagDistance += target.bestCameraToTarget.translation.norm
                    tagIds.add(target.fiducialId)
                }

                poseObservations.add(
                    PoseObservation(
                        result.timestampSeconds,
                        robotPose,
                        multitagResult.estimatedPose.ambiguity,
                        multitagResult.fiducialIDsUsed.size,
                        totalTagDistance / result.targets.size,
                        PoseObservationType.PHOTONVISION
                    )
                )
            } else if (result.targets.isNotEmpty()) { // single tag result
                val target = result.targets[0]

                // calculate robot pose
                val tagPose = aprilTagLayout.getTagPose(target.fiducialId)
                if (tagPose.isPresent) {
                    val fieldToTarget = Transform3d(tagPose.get().translation, tagPose.get().rotation)
                    val cameraToTarget: Transform3d = target.bestCameraToTarget
                    val fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse())
                    val fieldToRobot = fieldToCamera.plus(robotToCamera.inverse())
                    val robotPose = Pose3d(fieldToRobot.translation, fieldToRobot.rotation)

                    // add tag ID
                    tagIds.add(target.fiducialId)

                    // add observation
                    poseObservations.add(
                        PoseObservation(
                            result.timestampSeconds,
                            robotPose,
                            target.poseAmbiguity,
                            1,
                            cameraToTarget.translation.norm,
                            PoseObservationType.PHOTONVISION
                        )
                    )
                }
            }

            // save collections to the inputs arrays
            inputs.poseObservations = poseObservations.toTypedArray()
            inputs.tagIds = tagIds.toIntArray()
        }
    }
}
