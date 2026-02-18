package frc.team449

import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import edu.wpi.first.wpilibj.RobotBase
import kotlin.math.PI
import kotlin.math.pow

object Constants {
    const val LOOP_TIME = 0.02

    enum class Mode {
        REAL,
        SIM,
        REPLAY
    }

    val CURRENT_MODE: Mode = if (RobotBase.isReal()) Mode.REAL else Mode.SIM

    const val TUNING_MODE: Boolean = false

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
        const val X_LOCK_DEADBAND = 0.25

        const val WHEEL_COF = 1.4
    }

    object AutoConstants {
        const val AUTO_ANGULAR_SPEED_RADIANS_PER_SECOND = 1.26767 * PI
        const val AUTO_ANGULAR_ACCEL_RADIANS_PER_SECOND_PER_SECOND = PI
    }

    object FieldConstants {
        // CHS uses AndyMark Rebuilt Field
        val REBUILT_FIELD_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark)

        val FIELD_LENGTH_METERS = REBUILT_FIELD_LAYOUT.fieldLength
        val FIELD_WIDTH_METERS = REBUILT_FIELD_LAYOUT.fieldLength

        val BLUE_TRENCH_POSES: List<Pose2d> = listOf(
            Pose2d(4.35, 0.45, Rotation2d(1.5)),
            Pose2d(4.35, 7.60, Rotation2d(-1.5))
        )
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

        val FLYWHEEL_RADIUS = Meters.of(3.965079 / 2)
        val SHOOTER_HEIGHT = Meters.of(1.0) // TODO: Find

        const val HOOD_SUPPLY_LIM = 40.0
        const val HOOD_STATOR_LIM = 50.0

        val MIN_HOOD_ANGLE = Degrees.of(14.85072467)
        val MAX_HOOD_ANGLE = Degrees.of(46.24524767)

        // hood gains
        const val HOOD_KP = 6.7
        const val HOOD_KI = 0.67
        const val HOOD_KD = 0.0

        // feedforward
        const val HOOD_KS = 0.1
        const val HOOD_KG = 0.11
        const val HOOD_KV = 2.1

        // flywheel gains
        const val FLYWHEEL_KP = 0.5
        const val FLYWHEEL_KI = 0.0
        const val FLYWHEEL_KD = 0.0
        const val FLYWHEEL_KS = 0.05
        const val FLYWHEEL_KV = 0.1

        // debouncer
        const val HOMING_DEBOUNCE_TIME = 0.5 // seconds
        val HOMING_DEBOUNCE_TYPE = Debouncer.DebounceType.kRising

        const val TOLERANCE_DEBOUNCE_TIME = 0.2 // seconds
        val TOLERANCE_DEBOUNCE_TYPE = Debouncer.DebounceType.kRising

        const val HOOD_TOLERANCE_RAD = 0.1 // todo: REFINE

        const val FLYWHEEL_VELOCITY_TOLERANCE_RAD_PER_SEC = 10.0

        const val FLYWHEEL_GEARING = 32.0 / 18 //
        const val HOOD_GEARING = 6.0 * 15 // TODO: rough estimate

        const val HOOD_MOMENT_OF_INERTIA = 0.0694270649
        val HOOD_LENGTH = Units.inchesToMeters(5.91)
        val FLYWHEEL_MOI = 0.5 * Units.lbsToKilograms(1.5) * Units.inchesToMeters(4.0).pow(2.0)

        const val HOMING_VOLTAGE = 2.0
        const val HOMING_STATOR_AMPS = 45.0 // amps
    }

    object LEDConstants {
        // led constants
    }

    object Dimensions {
        val BUMPER_THICKNESS: Distance = Inches.of(3.0) // frame to edge of bumper
        val BUMPER_HEIGHT: Distance = Inches.of(7.0) // height from floor to top of bumper
        val FRAME_WIDTH: Distance = Inches.of(27.0) // left to right (y-axis)
        val FRAME_LENGTH: Distance = Inches.of(27.0) // front to back (x-axis)

        val FULL_WIDTH: Distance = FRAME_WIDTH.plus(BUMPER_THICKNESS.times(2.0))
        val FULL_LENGTH: Distance = FRAME_LENGTH.plus(BUMPER_THICKNESS.times(2.0))
    }

    // other subsystem constants when applicable

    object IntakeConstants {
        // config constants
        const val PIVOT_MOTOR_ID = 40
        const val PIVOT_FOLLOWER_ID = 41
        const val ROLLER_MOTOR_ID = 42
        const val ROLLER_FOLLOWER_ID = 43

        val ROLLER_FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed
        val PIVOT_FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed

        const val STOW_POS_RADS = 0.0
        const val DEPLOY_POS_RADS = 1.61

        const val DEPLOY_VOLTS = 8.0
        const val DEPLOY_HOLD_VOLTS = 2.0
        const val STOW_VOLTS = -8.0
        const val STOW_HOLD_VOLTS = -2.0

        const val PIVOT_GEARING_SENSOR_TO_MECH = 50.0
        const val PIVOT_MOI = 0.1549510896
        const val ARM_LENGTH_METERS = 0.2996692
        const val VIZ_OFFSET_DEG = 33.873

        val INTAKE_VELOCITY: AngularVelocity = RotationsPerSecond.of(80.0)
        val OUTTAKE_VELOCITY: AngularVelocity = RotationsPerSecond.of(-40.0)

        const val ROLLER_MOI = 0.0001411489
        const val ROLLER_GEARING = 4.0 / 3

        val HOMING_CURRENT_AMPS = 20.0
        val HOMING_VELOCITY_RAD_PER_SEC = 0.5
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
