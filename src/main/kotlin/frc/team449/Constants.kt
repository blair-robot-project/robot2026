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
        const val LEFT_FLYWHEEL_LEADER_ID = 11
        const val LEFT_FLYWHEEL_FOLLOWER_ID = 12
        const val RIGHT_FLYWHEEL_LEADER_ID = 13
        const val RIGHT_FLYWHEEL_FOLLOWER_ID = 14
        const val HOOD_MOTOR_ID = 15

        const val FLYWHEEL_SUPPLY_LIM = 40.0
        const val FLYWHEEL_STATOR_LIM = 80.0

        const val HOOD_SUPPLY_LIM = 40.0
        const val HOOD_STATOR_LIM = 50.0

        val HOOD_MIN_ANGLE = Degrees.of(14.85072467)
        val HOOD_MAX_ANGLE = Degrees.of(46.24524767)

        //hood gains
        const val HOOD_KP = 6.7
        const val HOOD_KI = 0.67
        const val HOOD_KD = 0.0
        //feedforward
        const val HOOD_KS = 0.1
        const val HOOD_KG = 0.11
        const val HOOD_KV = 2.1

        //flywheel gains
        const val FLYWHEEL_KP = 0.5
        const val FLYWHEEL_KI = 0.0
        const val FLYWHEEL_KD = 0.0
        const val FLYWHEEL_KS = 0.0


        val HOOD_TOLERANCE = Degrees.of(5.0) // TODO: find

        const val FLYWHEEL_GEARING = 32.0/18 //
        const val HOOD_GEARING = 6.0*15 // TODO: rough estimate

        const val HOOD_MOMENT_OF_INERTIA = 0.07
        val HOOD_LENGTH = Units.inchesToMeters(5.91)
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

        const val LEFT_INDEXER_ID = 21
        const val RIGHT_INDEXER_ID = 22
        const val INDEXER_STATOR_LIMIT = 60.0
        const val INDEXER_SUPPLY_LIMIT = 30.0

        // add more indexers as needed
    }
}
