package frc.team449

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.util.Units
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

    // used to remove the shooter sim bindings I was using to test
    const val RUNNING_SHOOTER_SIM = true

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

        // these are all random ahh gains but it dont matter cuz its sim
        const val HOOD_SIM_KP = 3.0
        const val HOOD_SIM_KI = 0.0
        const val HOOD_SIM_KD = 0.0

        const val HOOD_SIM_KS = 0.2
        const val HOOD_SIM_KG = 0.3
        const val HOOD_SIM_KV = 0.1
        const val HOOD_SIM_KA = 0.0

        const val FLYWHEEL_SIM_KS: Double = 0.0001 // V
        const val FLYWHEEL_SIM_KV: Double = 0.000195 // V/RPM
        const val FLYWHEEL_SIM_KA: Double = 0.0003 // V/(RPM/s)

        val HOOD_TOLERANCE = Degrees.of(5.0) // TODO: find

        const val HOOD_SIM_GRAVITY = false

        const val FLYWHEEL_GEARING = 1.0 // TODO: find
        const val HOOD_GEARING = 1.0 // TODO: find

        const val HOOD_ANGLE_ENCODER_DISTANCE_PER_PULSE = 2 * PI / 4096 // TODO: find
        const val HOOD_MOTOR_GEARING = 1.0 // TODO: find
        const val HOOD_MASS = 8.0 // kg TODO: find
        val HOOD_LENGTH = Units.inchesToMeters(30.0) // TODO: find
    }

    object LEDConstants {
        // led constants
    }

    // other subsystem constants when applicable

    object IntakeConstants {
        // config constants
        const val PIVOT_MOTOR_ID = 40

        const val PIVOT_STATOR_LIMIT = 80.0
        const val PIVOT_SUPPLY_LIMIT = 40.0

        const val ROLLER_STATOR_LIMIT = 80.0
        const val ROLLER_SUPPLY_LIMIT = 40.0

        const val LEFT_ROLLER_MOTOR_ID = 41
        const val RIGHT_ROLLER_MOTOR_ID = 42

        // setpoint constants
        const val PIVOT_INTAKE_VOLTAGE = 0.0
        const val LEFT_ROLLER_INTAKE_VOLTAGE = 8.0
        const val RIGHT_ROLLER_INTAKE_VOLTAGE = 8.0

        const val PIVOT_STOW_VOLTAGE = 5.0
        const val LEFT_ROLLER_STOW_VOLTAGE = 0.0
        const val RIGHT_ROLLER_STOW_VOLTAGE = 0.0
    }

    object IndexerConstants {
        // motor definitions that are currently placeholders

        const val LEFT_INDEXER_ID = 1
        const val RIGHT_INDEXER_ID = 2
        const val INDEXER_STATOR_LIMIT = 60.0
        const val INDEXER_SUPPLY_LIMIT = 30.0

        // add more indexers as needed
    }
}
