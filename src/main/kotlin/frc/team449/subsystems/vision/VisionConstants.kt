// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.team449.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d

object VisionConstants {
    var aprilTagLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark)

    var camera0Name: String = "limelight-right"
    var camera1Name: String = "limelight-left"

    // Robot to camera transforms
    // TODO: idt these are actually right, will prolly havta look at it again after it's mounted
    var robotToCameraRight: Pose3d = Pose3d(0.2, -0.2, 0.53, Rotation3d(0.0, 30.0, -25.0))
    var robotToCameraLeft: Pose3d = Pose3d(-0.2, -0.2, 0.53, Rotation3d(0.0, 30.0, -25.0))

    // Basic filtering thresholds
    var maxAmbiguity: Double = 0.3
    var maxZError: Double = 0.75

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    var linearStdDevBaseline: Double = 0.02 // Meters
    var angularStdDevBaseline: Double = 0.06 // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    var cameraStdDevFactors: DoubleArray = doubleArrayOf(
        1.0, // Camera 0
        1.0 // Camera 1
    )

    // Multipliers to apply for MegaTag 2 observations
    var linearStdDevMegatag2Factor: Double = 0.5 // More stable than full 3D solve
    var angularStdDevMegatag2Factor: Double = Double.POSITIVE_INFINITY // No rotation data available
}
