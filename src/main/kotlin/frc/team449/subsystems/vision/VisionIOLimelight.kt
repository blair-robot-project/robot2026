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
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.networktables.*
import edu.wpi.first.wpilibj.RobotController
import frc.team449.subsystems.vision.VisionIO.*
import java.util.*
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
    offset: Transform3d,
    private val rotationSupplier: Supplier<Rotation2d>
) : VisionIO {
    private val orientationPublisher: DoubleArrayPublisher
    private val latencySubscriber: DoubleSubscriber
    private val txSubscriber: DoubleSubscriber
    private val tySubscriber: DoubleSubscriber
    private val megatag1Subscriber: DoubleArraySubscriber
    private val megatag2Subscriber: DoubleArraySubscriber

    init {
        val table = NetworkTableInstance.getDefault().getTable(name)
        orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish()
        latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0)
        txSubscriber = table.getDoubleTopic("tx").subscribe(0.0)
        tySubscriber = table.getDoubleTopic("ty").subscribe(0.0)
        megatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(doubleArrayOf())
        megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(doubleArrayOf())

        // TODO: check this please
        table.getDoubleArrayTopic("camerapose_robotspace_set").publish().accept(doubleArrayOf(offset.x, offset.y, offset.z, offset.rotation.x, offset.rotation.y, offset.rotation.z))
    }

    override fun updateInputs(inputs: VisionIOInputs) {
        // Update connection status based on whether an update has been seen in the last 250ms
        inputs.connected = ((RobotController.getFPGATime() - latencySubscriber.lastChange) / 1000) < 250

        // Update target observation
        inputs.latestTargetObservation = TargetObservation(
            Rotation2d.fromDegrees(txSubscriber.get()),
            Rotation2d.fromDegrees(tySubscriber.get())
        )

        // update orientation for MT2
        // TODO: orientationPublisher: DEG and DEG/S [yaw, yawrate, pitch, pitchrate, roll, rollrate]
        orientationPublisher.accept(doubleArrayOf(rotationSupplier.get().degrees, 0.0, 0.0, 0.0, 0.0, 0.0))
        NetworkTableInstance.getDefault().flush() // increases network traffic but recommended by Limelight

        val mt1Queue = megatag1Subscriber.readQueue()
        val mt2Queue = megatag2Subscriber.readQueue()

        // read new pose observations from NT
        val tagIds: MutableSet<Int> = HashSet()
        val poseObservations: ArrayList<PoseObservation> = ArrayList<PoseObservation>(mt1Queue.size + mt2Queue.size)

        for (rawSample in megatag1Subscriber.readQueue()) {
            if (rawSample.value.isEmpty()) continue
            var i = 11
            while (i < rawSample.value.size) {
                tagIds.add(rawSample.value[i].toInt())
                i += 7
            }
            poseObservations.add(
                PoseObservation(
                    rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3, // timestamp
                    parsePose(rawSample.value), // 3d pose estimate
                    if (rawSample.value.size >= 18) rawSample.value[17] else 0.0, // ambiguity, using only the first tag because ambiguity isn't applicable for multitag
                    rawSample.value[7].toInt(), // tag count
                    rawSample.value[9], // avg tag distance
                    PoseObservationType.MEGATAG_1
                )
            )
        }

        for (rawSample in megatag2Subscriber.readQueue()) {
            if (rawSample.value.isEmpty()) continue
            var i = 11
            while (i < rawSample.value.size) {
                tagIds.add(rawSample.value[i].toInt())
                i += 7
            }
            poseObservations.add(
                PoseObservation(
                    rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3, // timestamp
                    parsePose(rawSample.value), // 3d pose estimate
                    0.0, // ambiguity, already disambiguated
                    rawSample.value[7].toInt(), // tag count
                    rawSample.value[9], // avg tag distance
                    PoseObservationType.MEGATAG_2
                )
            )
        }

        // save pose observations to inputs object
        inputs.poseObservations = poseObservations.toTypedArray()

        // save tag IDs to inputs
        inputs.tagIds = IntArray(tagIds.size)
        var i = 0
        for (id in tagIds) {
            inputs.tagIds[i++] = id // sahu is having a nightmare rn and doesn't know why
        }
    }

    companion object {
        /** Parses the 3D pose from a Limelight botpose array.  */
        private fun parsePose(rawLLArray: DoubleArray): Pose3d {
            return Pose3d(
                rawLLArray[0],
                rawLLArray[1],
                rawLLArray[2],
                Rotation3d(
                    Units.degreesToRadians(rawLLArray[3]),
                    Units.degreesToRadians(rawLLArray[4]),
                    Units.degreesToRadians(rawLLArray[5])
                )
            )
        }
    }
}
