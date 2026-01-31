package frc.team449

import com.pathplanner.lib.config.RobotConfig
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Distance
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

    object AutoConstants {
        val config: RobotConfig = RobotConfig.fromGUISettings()
    }

    object LEDConstants {
        // led constants
    }

    object FuelSimDimensions {
        val BUMPER_THICKNESS: Distance = Inches.of(3.0) // frame to edge of bumper
        val BUMPER_HEIGHT: Distance = Inches.of(7.0) // height from floor to top of bumper
        val FRAME_WIDTH: Distance = Inches.of(27.0) // left to right (y-axis)
        val FRAME_LENGTH: Distance = Inches.of(27.0) // front to back (x-axis)

        val FULL_WIDTH: Distance = FRAME_WIDTH.plus(BUMPER_THICKNESS.times(2.0))
        val FULL_LENGTH: Distance = FRAME_LENGTH.plus(BUMPER_THICKNESS.times(2.0))
    }

    object IntakeConstants {
        // config constants
        const val PIVOT_MOTOR_ID = 40

        const val PIVOT_STATOR_LIMIT = 80.0
        const val PIVOT_SUPPLY_LIMIT = 40.0

        const val LEFT_ROLLER_MOTOR_ID = 41
        const val RIGHT_ROLLER_MOTOR_ID = 42

        const val ROLLER_STATOR_LIMIT = 80.0
        const val ROLLER_SUPPLY_LIMIT = 40.0

        const val ROLLER_KP = 5.0
        const val ROLLER_KV = 0.12

        // sim constants

        // setpoint constants
    }

    // other subsystem constants when applicable
}
