// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.team449.subsystems.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import frc.team449.Constants.VisionConstants
import frc.team449.subsystems.vision.VisionIO.VisionIOInputs
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import java.util.function.Supplier

/** IO implementation for physics sim using PhotonVision simulator.  */
/**
 * Creates a new VisionIOPhotonVisionSim.
 *
 * @param name The name of the camera.
 * @param poseSupplier Supplier for the robot pose to use in simulation.
 */
class VisionIOPhotonVisionSim(
    name: String,
    robotToCamera: Transform3d,
    private val poseSupplier: Supplier<Pose2d>
) : VisionIOPhotonVision(name, robotToCamera) {
    private val cameraSim: PhotonCameraSim
    private val visionSim: VisionSystemSim = VisionSystemSim("main")

    init {
        visionSim.addAprilTags(VisionConstants.REBUILT_FIELD_LAYOUT)

        // camera properties (Limelight 4)
        val cameraProperties = SimCameraProperties()
        cameraProperties.setCalibration(1280, 800, Rotation2d(99.41))
        cameraProperties.setCalibError(0.0, 0.0)
        cameraProperties.setFPS(120.0)
        cameraProperties.setAvgLatencyMs(15.0)
        cameraProperties.setLatencyStdDevMs(5.0)

        // add sim camera
        cameraSim = PhotonCameraSim(camera, cameraProperties, VisionConstants.REBUILT_FIELD_LAYOUT)
        visionSim.addCamera(cameraSim, robotToCamera)
    }

    override fun updateInputs(inputs: VisionIOInputs) {
        visionSim.update(poseSupplier.get())
        super.updateInputs(inputs)
    }
}
