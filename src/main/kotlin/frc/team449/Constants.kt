package frc.team449

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Degrees
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

    const val LOOP_TIME = 0.02

    const val ROBOT_MASS_KG = 54.43
    const val ROBOT_WIDTH_INCHES = 32.0 // including bumpers (front to rear)
    const val ROBOT_LENGTH_INCHES = 32.0 // including bumpers (left to right)

    object DriveConstants {
        const val SIM_LOOP_TIME = 0.01 // 100 Hz

        const val MAX_LINEAR_SPEED_METERS_PER_SECOND = 5.04
        const val MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 1.26767 * PI

        const val TRACKWIDTH_INCHES = 27.0 // front to rear
        const val WHEELBASE_INCHES = 27.0 // left to right

        const val TRANSLATION_DEADBAND = 0.05
        const val ANGULAR_DEADBAND = 0.1

        const val WHEEL_COF = 1.4
    }

    object AutoConstants {
        // auto constants
    }

    object FieldConstants {
        // CHS uses AndyMark Rebuilt Field
        val REBUILT_FIELD_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark)
        val FIELD_LENGTH = REBUILT_FIELD_LAYOUT.fieldLength
        val FIELD_WIDTH = REBUILT_FIELD_LAYOUT.fieldLength
    }

    object PowerConstants {
        // current limit configs go here
    }

    object SensorConstants {
        // motor and sensor IDs
    }

    object VisionConstants {
        // vision constants
    }

    object ShooterConstants {
        const val LEFT_FLYWHEEL_LEADER_ID = 1
        const val LEFT_FLYWHEEL_FOLLOWER_ID = 2
        const val RIGHT_FLYWHEEL_LEADER_ID = 3
        const val RIGHT_FLYWHEEL_FOLLOWER_ID = 4
        const val HOOD_MOTOR_ID = 5

        const val FLYWHEEL_SUPPLY_LIM = 40.0
        const val FLYWHEEL_STATOR_LIM = 80.0

        const val HOOD_SUPPLY_LIM = 40.0
        const val HOOD_STATOR_LIM = 80.0

        const val HOOD_CRUISE_VELOCITY = 2.0
        const val HOOD_ACCELERATION = 5.0

        val HOOD_MIN_ANGLE = Degrees.of(30.0) // TODO: find
        val HOOD_MAX_ANGLE = Degrees.of(90.0) // TODO: find

        val HOOD_TOLERANCE = Degrees.of(5.0) // TODO: find

        val kSimGearbox = DCMotor.getKrakenX60Foc(4)
        val kSimGearing: Double = 30.0 / 8.0 // TODO: find real values
        const val kSimMOI: Double = 52.0 // TODO: find real values

        const val SHOOTER_VOLTAGE = 11.0

        const val FLYWHEEL_GEARING = 1.0 // TODO: find
        const val HOOD_GEARING = 1.0 // TODO: find
    }

    object LEDConstants {
        // led constants
    }

    // other subsystem constants when applicable
}
