// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.team449.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d

object VisionConstants {
    var aprilTagLayout: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField)

    var camera0Name: String = "camera_0" // TODO: match to names thru configuration
    var camera1Name: String = "camera_1"

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead) <--- TODO
    var robotToCamera0: Transform3d = Transform3d(0.2, 0.0, 0.2, Rotation3d(0.0, -0.4, 0.0))
    var robotToCamera1: Transform3d = Transform3d(-0.2, 0.0, 0.2, Rotation3d(0.0, -0.4, Math.PI))

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
