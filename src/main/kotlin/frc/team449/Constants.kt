package frc.team449

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotBase
import kotlin.math.PI

object Constants {
    enum class Mode {
        REAL,
        SIM,
        REPLAY
    }

    val CURRENT_MODE: Mode = if (RobotBase.isReal()) Mode.REAL else Mode.SIM
    const val TUNING_MODE: Boolean = false

    const val PDH_CAN_ID = 1

    /** Controller Configurations */
    val ROT_RATE_LIMIT: Double = 17.27 * PI // rad /s
    val NEG_ROT_RATE_LIM: Double = -27.5 * PI // rad / s
    val SNAP_TO_ANGLE_TOLERANCE_RAD: Angle = Units.Degrees.of(3.5)

    const val USE_ACCEL_LIMIT = true

    val POSE_2D_ZERO = Pose2d()

    const val LOOP_TIME = 0.02

    const val ROBOT_MASS_KG = 54.43
    const val ROBOT_WIDTH_INCHES = 35.0 // including bumpers (front to rear)
    const val ROBOT_LENGTH_INCHES = 35.0 // including bumpers (left to right)

    object DriveConstants {
        const val SIM_LOOP_TIME = 0.005

        const val MAX_LINEAR_SPEED_METERS_PER_SECOND = 4.6767
        const val MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 1.26767 * PI

        val MAX_ACCEL = 25.0 // m/s/s

        const val TRACKWIDTH_INCHES = 30.0 // front to rear
        const val WHEELBASE_INCHES = 30.0 // left to right

        const val TRANSLATION_DEADBAND = 0.05
        const val ANGULAR_DEADBAND = 0.05

        const val WHEEL_FRICTION_COEFFICIENT = 1.2
    }

    object PowerConstants
}
