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
open class VisionIOPhotonVision(name: String?, private val robotToCamera: Transform3d) : VisionIO {
    protected val camera: PhotonCamera = PhotonCamera(name)

    override fun updateInputs(inputs: VisionIOInputs) {
        inputs.connected = camera.isConnected

        // Read new camera observations
        val tagIds: MutableList<Short> = mutableListOf()
        val poseObservations: MutableList<PoseObservation> = LinkedList()
        for (result in camera.allUnreadResults) {
            // Update latest target observation
            if (result.hasTargets()) {
                inputs.latestTargetObservationPhoton =
                    TargetObservation(
                        Rotation2d.fromDegrees(result.bestTarget.getYaw()),
                        Rotation2d.fromDegrees(result.bestTarget.getPitch())
                    )

            } else {
                inputs.latestTargetObservationPhoton = TargetObservation(Rotation2d.kZero, Rotation2d.kZero)
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
                inputs.latestTagCount = multitagResult.fiducialIDsUsed.size

                inputs.latestTimestamp = result.timestampSeconds
                var runningMinAmbiguity = 100.0
                var runningMaxAmbiguity = 0.0
                var sumAmbiguity = 0.0
                for (i in 1..result.targets.size) {
                    if (result.targets[i - 1].poseAmbiguity < runningMinAmbiguity) runningMinAmbiguity = result.targets[i - 1].poseAmbiguity
                    if (result.targets[i - 1].poseAmbiguity > runningMaxAmbiguity) runningMaxAmbiguity = result.targets[i - 1].poseAmbiguity
                    sumAmbiguity += result.targets[i - 1].poseAmbiguity
                    inputs.orientation = result.targets[i - 1].yaw
                    inputs.latestAverageTagDist = result.targets[i - 1].bestCameraToTarget.translation.norm
                }
                inputs.latestMinTagAmbiguity = runningMinAmbiguity
                inputs.latestMaxTagAmbiguity = runningMaxAmbiguity
                inputs.latestAverageTagAmbiguity = sumAmbiguity / result.targets.size

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

                inputs.latestPose = robotPose
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
                    inputs.latestTagCount = 1

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
                    inputs.latestPose = robotPose
                }
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = poseObservations.toTypedArray()

        // Save tag IDs to inputs objects
        inputs.tagIds = DoubleArray(tagIds.size)
        var i = 0
        for (id in tagIds) {
            inputs.tagIds[i++] = id.toDouble()
        }
    }
}
