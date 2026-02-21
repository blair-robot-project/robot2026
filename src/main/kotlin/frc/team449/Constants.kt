package frc.team449

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import edu.wpi.first.units.measure.Angle
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

    const val ROBOT_MASS_KG = 67.0
    const val ROBOT_WIDTH_INCHES = 35.0 // including bumpers (front to rear)
    const val ROBOT_LENGTH_INCHES = 34.125 // including bumpers (left to right)

    object DriveConstants {
        const val SIM_LOOP_TIME = 0.01 // 100 Hz
        const val ODOMETRY_LOOP_HZ = 100.0

        const val MAX_LINEAR_SPEED_METERS_PER_SECOND = 5.04
        const val MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * PI

        const val TRACKWIDTH_INCHES = 21.75 // front to rear
        const val WHEELBASE_INCHES = 21.75 // left to right

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

        val BLUE_TRENCH_POSES: List<Pose2d> =
            listOf(
                Pose2d(4.35, 0.45, Rotation2d(1.5)),
                Pose2d(4.35, 7.60, Rotation2d(-1.5)),
            )
    }

    object VisionConstants {
        // vision constants
    }

    object ShooterConstants {
        // FLYWHEEL
        const val LEFT_FLYWHEEL_LEADER_ID = 11
        const val LEFT_FLYWHEEL_FOLLOWER_ID = 12
        const val RIGHT_FLYWHEEL_LEADER_ID = 13
        const val RIGHT_FLYWHEEL_FOLLOWER_ID = 14

        const val FLYWHEEL_SUPPLY_LIM = 40.0
        const val FLYWHEEL_STATOR_LIM = 80.0

        const val FLYWHEEL_GEARING = 16.0 / 9.0

        const val FLYWHEEL_KP = 0.01
        const val FLYWHEEL_KI = 0.0
        const val FLYWHEEL_KD = 0.0
        const val FLYWHEEL_KS = 0.0
        const val FLYWHEEL_KV = 0.213

        val TRENCH_HOOD_ANGLE: Angle = Degrees.of(30.0) // TODO:FIND
        val TRENCH_FLYWHEEL_VEL = RadiansPerSecond.of(200.0)

        val HUB_HOOD_ANGLE: Angle = Degrees.of(30.0) // todo: find
        val HUB_FLYWHEEL_VEL = RadiansPerSecond.of(100.0)

        const val EFFICIENCY = 0.95 // fake value for sim

        // HOOD
        const val HOOD_MOTOR_ID = 15

        const val HOOD_SUPPLY_LIM = 40.0
        const val HOOD_STATOR_LIM = 50.0

        const val HOOD_GEARING = 106.0
        const val HOOD_ROLLER_GEARING = 64.0 / 27.0
        val HOOD_ROLLER_RADIUS = Inches.of(0.5)


        const val HOOD_KP = 100.0
        const val HOOD_KI = 0.0
        const val HOOD_KD = 0.0
        const val HOOD_KS = 0.1
        const val HOOD_KG = 0.11
        const val HOOD_KV = 2.1

        // HOMING AND TOLERANCE
        const val HOMING_DEBOUNCE_TIME = 0.5 // seconds
        const val TOLERANCE_DEBOUNCE_TIME = 0.2 // seconds

        const val HOMING_VOLTAGE = -2.0
        const val HOMING_CURRENT_AMPS = 45.0 // amps
        const val HOMING_VELOCITY_RAD_PER_SEC = 0.5

        const val FLYWHEEL_VELOCITY_TOLERANCE_RAD_PER_SEC = 10.0
        const val HOOD_TOLERANCE_RAD = 0.1 // todo: REFINE

        val MIN_HOOD_ANGLE: Angle = Degrees.of(14.85072467) // todo: verify
        val MAX_HOOD_ANGLE: Angle = Degrees.of(46.24524767) // todo: verify

        val FLYWHEEL_MOI = 0.0033537

        const val HOOD_MOI = 0.077132
        val HOOD_LENGTH = Units.inchesToMeters(7.1)

        // fuel sim
        val FLYWHEEL_RADIUS = Units.inchesToMeters(3.965079 / 2)
        val SHOOTER_HEIGHT = Inches.of(18.0)
    }

    object Dimensions {
        val BUMPER_THICKNESS: Distance = Inches.of(3.0) // frame to edge of bumper
        val BUMPER_HEIGHT: Distance = Inches.of(7.0) // height from floor to top of bumper
        val FRAME_WIDTH: Distance = Inches.of(33.0) // i think? // left to right (y-axis)
        val FRAME_LENGTH: Distance = Inches.of(27.0) // front to back (x-axis)

        val FULL_WIDTH: Distance = FRAME_WIDTH.plus(BUMPER_THICKNESS.times(2.0))
        val FULL_LENGTH: Distance = FRAME_LENGTH.plus(BUMPER_THICKNESS.times(2.0))
    }

    // other subsystem constants when applicable

    object IntakeConstants {
        // config constants
        const val LEFT_PIVOT_MOTOR_ID = 40
        const val RIGHT_PIVOT_FOLLOWER_ID = 41
        const val LEFT_ROLLER_MOTOR_ID = 42
        const val RIGHT_ROLLER_FOLLOWER_ID = 43

        val LEFT_PIVOT_NEUTRAL_MODE = NeutralModeValue.Brake
        val LEFT_PIVOT_INVERSION = InvertedValue.Clockwise_Positive

        val LEFT_ROLLER_NEUTRAL_MODE = NeutralModeValue.Coast
        val LEFT_ROLLER_INVERSION = InvertedValue.CounterClockwise_Positive

        val RIGHT_ROLLER_FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed
        val RIGHT_PIVOT_FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed

        const val PIVOT_GEARING_SENSOR_TO_MECH = 48.75

        const val PIVOT_SUPPLY_LIMIT = 20.0
        const val PIVOT_STATOR_LIMIT = 40.0

        const val STOW_POS_RADS = 0.0
        const val DEPLOY_POS_RADS = 2.269
        const val DEPLOY_VOLTS = 8.0
        const val DEPLOY_HOLD_VOLTS = 0.0
        const val STOW_VOLTS = -8.0
        const val STOW_HOLD_VOLTS = -0.5

        const val HOMING_CURRENT_AMPS = 20.0
        const val HOMING_VELOCITY_RAD_PER_SEC = 0.5

        const val PIVOT_MOI = 0.16241
        val ARM_LENGTH_METERS = Units.inchesToMeters(8.4)

        const val VIZ_OFFSET_DEG = 33.873

        const val ROLLER_SUPPLY_LIMIT = 40.0
        const val ROLLER_STATOR_LIMIT = 80.0

        val INTAKE_VELOCITY: AngularVelocity = RotationsPerSecond.of(80.0)
        val OUTTAKE_VELOCITY: AngularVelocity = RotationsPerSecond.of(-40.0)
        const val ROLLER_MOI = 0.0001411489

        const val ROLLER_GEARING = 4.0 / 3
    }

    // INDEXER CONSTANTS STILL SLIGHTLY OFF
    object IndexerConstants {
        const val WEDGE_INDEXER_ID = 21
        const val WEDGE_STATOR_LIMIT = 30.0
        const val WEDGE_SUPPLY_LIMIT = 30.0
        const val WEDGE_GEARING = 1.5

        val WEDGE_NEUTRAL_MODE = NeutralModeValue.Coast
        val WEDGE_INVERSION = InvertedValue.Clockwise_Positive

        const val WEDGE_KP = 0.5
        const val WEDGE_KI = 0.0
        const val WEDGE_KD = 0.0
        const val WEDGE_KS = 0.05
        const val WEDGE_KV = 0.12

        const val WEDGE_MOI = .001

        const val FLOOR_INDEXER_ID = 22
        const val FLOOR_STATOR_LIMIT = 50.0
        const val FLOOR_SUPPLY_LIMIT = 30.0
        const val FLOOR_GEARING = 27.0 / 14.0

        val FLOOR_NEUTRAL_MODE = NeutralModeValue.Coast
        val FLOOR_INVERSION = InvertedValue.CounterClockwise_Positive

        const val FLOOR_KP = 0.5
        const val FLOOR_KI = 0.0
        const val FLOOR_KD = 0.0
        const val FLOOR_KS = 0.05
        const val FLOOR_KV = 0.1

        const val FLOOR_MOI = .005

        const val TOP_INDEXER_ID = 23
        const val TOP_STATOR_LIMIT = 60.0
        const val TOP_SUPPLY_LIMIT = 30.0
        const val TOP_GEARING = 31.0 / 11.0

        val TOP_NEUTRAL_MODE = NeutralModeValue.Coast
        val TOP_INVERSION = InvertedValue.Clockwise_Positive

        const val TOP_KP = 0.5
        const val TOP_KI = 0.0
        const val TOP_KD = 0.0
        const val TOP_KS = 0.05
        const val TOP_KV = 0.1

        const val TOP_MOI = .000000008 // TODO: Find

        val SHOOTING_INDEXER_SPEED: AngularVelocity = RadiansPerSecond.of(30.0)
    }

    object LEDConstants {
        // led constants
    }
}
