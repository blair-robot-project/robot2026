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
import frc.team449.subsystems.vision.VisionConstants.aprilTagLayout
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
open class VisionIOPhotonVision(name: String?, protected val robotToCamera: Transform3d) : VisionIO {
    protected val camera: PhotonCamera = PhotonCamera(name)

    override fun updateInputs(inputs: VisionIOInputs?) {
        if (inputs != null) {
            inputs.connected = camera.isConnected

            // Read new camera observations
            val tagIds: MutableList<Short?> = mutableListOf()
            val poseObservations: MutableList<PoseObservation> = LinkedList()
            for (result in camera.allUnreadResults) {
                // Update latest target observation
                if (result.hasTargets()) {
                    inputs.latestTargetObservation =
                        TargetObservation(
                            Rotation2d.fromDegrees(result.bestTarget.getYaw()),
                            Rotation2d.fromDegrees(result.bestTarget.getPitch())
                        )
                } else {
                    inputs.latestTargetObservation = TargetObservation(Rotation2d.kZero, Rotation2d.kZero)
                }

                // Add pose observation
                if (result.multitagResult.isPresent) { // Multitag result
                    val multitagResult = result.multitagResult.get()

                    // Calculate robot pose
                    val fieldToCamera: Transform3d = multitagResult.estimatedPose.best
                    val fieldToRobot = fieldToCamera.plus(robotToCamera.inverse())
                    val robotPose = Pose3d(fieldToRobot.translation, fieldToRobot.rotation)

                    // Calculate average tag distance
                    var totalTagDistance = 0.0
                    for (target in result.targets) {
                        totalTagDistance += target.bestCameraToTarget.translation.norm
                    }

                    // Add tag IDs
                    tagIds.addAll(multitagResult.fiducialIDsUsed)

                    // Add observation
                    poseObservations.add(
                        PoseObservation(
                            result.timestampSeconds, // Timestamp
                            robotPose, // 3D pose estimate
                            multitagResult.estimatedPose.ambiguity, // Ambiguity
                            multitagResult.fiducialIDsUsed.size, // Tag count
                            totalTagDistance / result.targets.size, // Average tag distance
                            PoseObservationType.PHOTONVISION
                        )
                    ) // Observation type
                } else if (result.targets.isNotEmpty()) { // Single tag result
                    val target = result.targets[0]

                    // Calculate robot pose
                    val tagPose = aprilTagLayout.getTagPose(target.fiducialId)
                    if (tagPose.isPresent) {
                        val fieldToTarget = Transform3d(tagPose.get().translation, tagPose.get().rotation)
                        val cameraToTarget: Transform3d = target.bestCameraToTarget
                        val fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse())
                        val fieldToRobot = fieldToCamera.plus(robotToCamera.inverse())
                        val robotPose = Pose3d(fieldToRobot.translation, fieldToRobot.rotation)

                        // Add tag ID
                        tagIds.add(target.fiducialId.toShort())

                        // Add observation
                        poseObservations.add(
                            PoseObservation(
                                result.timestampSeconds, // Timestamp
                                robotPose, // 3D pose estimate
                                target.poseAmbiguity, // Ambiguity
                                1, // Tag count
                                cameraToTarget.translation.norm, // Average tag distance
                                PoseObservationType.PHOTONVISION
                            )
                        ) // Observation type
                    }
                }
            }

            // Save pose observations to inputs object
            inputs.poseObservations = arrayOfNulls(poseObservations.size)
            for (i in poseObservations.indices) {
                inputs.poseObservations[i] = poseObservations[i]
            }

            // Save tag IDs to inputs objects
            inputs.tagIds = IntArray(tagIds.size)
            var i = 0
            for (id in tagIds) {
                if (id != null) {
                    inputs.tagIds[i++] = id.toInt()
                }
            }
        }
    }
}
