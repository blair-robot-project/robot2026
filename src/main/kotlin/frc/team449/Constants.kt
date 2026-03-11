package frc.team449

import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.geometry.*
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotBase
import kotlin.math.PI

object Constants {
    // --- OPERATIONAL MODES ---
    enum class Mode {
        REAL,
        SIM,
        REPLAY
    }

    val CURRENT_MODE: Mode = if (RobotBase.isReal()) Mode.REAL else Mode.SIM
    const val TUNING_MODE: Boolean = false

    // --- SYSTEM TIMING ---
    const val LOOP_TIME = 0.02

    // --- PHYSICAL SPECS ---
    const val ROBOT_MASS_KG = 59.8
    const val ROBOT_WIDTH_INCHES = 35.0 // including bumpers (front to rear)
    const val ROBOT_LENGTH_INCHES = 34.125 // including bumpers (left to right)

    object DriveConstants {
        // --- LOOP TIMING ---
        const val SIM_LOOP_TIME = 0.01 // 100 Hz
        const val ODOMETRY_LOOP_HZ = 100.0

        // --- PHYSICAL SPECS ---
        const val TRACKWIDTH_INCHES = 21.75 // front to rear
        const val WHEELBASE_INCHES = 21.75 // left to right
        const val WHEEL_COF = 1.4

        // --- SPEED LIMITS (STANDARD) ---
        const val MAX_LINEAR_SPEED_METERS_PER_SECOND = 4.7244
        const val MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = 5.804 / 4 * PI

        // --- SPEED LIMITS (SLOW) ---
        const val SLOW_LINEAR_SPEED_METERS_PER_SECOND = 1.5
        const val SLOW_ANGULAR_SPEED_RADIANS_PER_SECOND = 0.5804

        // --- DEADBANDS & TOLERANCE ---
        const val TRANSLATION_DEADBAND = 0.05
        const val ANGULAR_DEADBAND = 0.1
        const val MODULE_ALIGN_TOLERANCE = 5.0 // degrees
    }

    object AutoConstants {
        // auto constants TODO: tune
        const val TRANSLATION_P = 3.2
        const val TRANSLATION_I = 0.0
        const val TRANSLATION_D = 0.0

        const val ROTATION_P = 2.8
        const val ROTATION_I = 0.0
        const val ROTATION_D = 0.0

        const val CTE_P = 1.0
        const val CTE_I = 0.0
        const val CTE_D = 0.0

        const val AUTO_SHOOTING_TIME_SEC = 5.0

        const val AUTO_ANGULAR_SPEED_RADIANS_PER_SECOND = 2 * PI
        const val AUTO_ANGULAR_ACCEL_RADIANS_PER_SECOND_PER_SECOND = 4 * PI
    }

    object FieldConstants {
        val FIELD_LENGTH_METERS = VisionConstants.REBUILT_FIELD_LAYOUT.fieldLength
        val FIELD_WIDTH_METERS = VisionConstants.REBUILT_FIELD_LAYOUT.fieldWidth
    }

    object ShooterConstants {
        // --- HARDWARE IDs ---
        const val LEFT_FLYWHEEL_LEADER_ID = 11
        const val LEFT_FLYWHEEL_FOLLOWER_ID = 12
        const val RIGHT_FLYWHEEL_LEADER_ID = 13
        const val RIGHT_FLYWHEEL_FOLLOWER_ID = 14
        const val HOOD_MOTOR_ID = 15

        // --- HARDWARE CONFIGURATION ---
        val LEFT_FLYWHEEL_NEUTRAL_MODE = NeutralModeValue.Coast
        val LEFT_FLYWHEEL_INVERSION = InvertedValue.CounterClockwise_Positive
        val LEFT_FLYWHEEL_FOLLOWER_ALIGNMENT = MotorAlignmentValue.Aligned

        val RIGHT_FLYWHEEL_NEUTRAL_MODE = NeutralModeValue.Coast
        val RIGHT_FLYWHEEL_INVERSION = InvertedValue.Clockwise_Positive
        val RIGHT_FLYWHEEL_FOLLOWER_ALIGNMENT = MotorAlignmentValue.Aligned

        val HOOD_NEUTRAL_MODE = NeutralModeValue.Brake
        val HOOD_INVERSION = InvertedValue.CounterClockwise_Positive

        // --- PHYSICAL SPECS & GEARING ---
        const val FLYWHEEL_GEARING = 16.0 / 9.0
        const val FLYWHEEL_MOI_KG_MM = .0033537
        val FLYWHEEL_RADIUS = Units.inchesToMeters(3.965079 / 2)

        const val HOOD_GEARING = 106.0
        const val HOOD_ROLLER_GEARING = 1.0 / 3.0
        val HOOD_ROLLER_RADIUS: Distance = Inches.of(0.5)
        val HOOD_LENGTH = Units.inchesToMeters(7.1)
        const val HOOD_MOI_KG_MM = .077132
        const val EFFICIENCY = 0.97

        // --- CURRENT & OPERATIONAL LIMITS ---
        const val FLYWHEEL_SUPPLY_LIM = 30.0
        const val FLYWHEEL_STATOR_LIM = 80.0
        const val HOOD_SUPPLY_LIM = 20.0
        const val HOOD_STATOR_LIM = 20.0

        val MIN_HOOD_ANGLE: Angle = Radians.of(0.0)
        val MAX_HOOD_ANGLE: Angle = Radians.of(0.5185)
        val SHOOTER_HEIGHT: Distance = Inches.of(18.0)

        // --- FLYWHEEL GAINS ---
        const val LEFT_FLYWHEEL_KP = 0.55
        const val LEFT_FLYWHEEL_KI = 0.0
        const val LEFT_FLYWHEEL_KD = 0.0
        const val LEFT_FLYWHEEL_KS = 0.286
        const val LEFT_FLYWHEEL_KV = 0.2067
        // const val LEFT_FLYWHEEL_KA = ...

        const val RIGHT_FLYWHEEL_KP = 0.5
        const val RIGHT_FLYWHEEL_KI = 0.0
        const val RIGHT_FLYWHEEL_KD = 0.0
        const val RIGHT_FLYWHEEL_KS = 0.271
        const val RIGHT_FLYWHEEL_KV = 0.2055
        // const val RIGHT_FLYWHEEL_KA = ...

        // --- HOOD GAINS ---
        const val HOOD_KP = 200.0
        const val HOOD_KI = 0.0
        const val HOOD_KD = 0.0
        const val HOOD_KS = 0.1
        const val HOOD_KG = 0.14 // 0.14
        const val HOOD_KV = 3.5

        // --- HOMING & TOLERANCE ---
        const val HOMING_VOLTAGE = -2.0
        const val HOMING_CURRENT_AMPS = 10.0
        const val HOMING_VELOCITY_RAD_PER_SEC = 0.2
        const val HOMING_DEBOUNCE_TIME = 0.1

        const val HOOD_TOLERANCE_RAD = 0.02
        const val FLYWHEEL_VELOCITY_TOLERANCE_RAD_PER_SEC = 10.0
        const val TOLERANCE_DEBOUNCE_TIME = 0.2

        // --- STATIC SETPOINTS ---
        val HUB_HOOD_ANGLE: Angle = MIN_HOOD_ANGLE
        val HUB_FLYWHEEL_VEL: AngularVelocity = RadiansPerSecond.of(160.5)

        val TRENCH_HOOD_ANGLE: Angle = Radians.of(0.14)
        val TRENCH_FLYWHEEL_VEL: AngularVelocity = RadiansPerSecond.of(200.5)

        val TOWER_HOOD_ANGLE: Angle = Radians.of(0.09)
        val TOWER_FLYWHEEL_VEL: AngularVelocity = RadiansPerSecond.of(200.5)

        val TEST_FLYWHEEL_VEL: AngularVelocity = RadiansPerSecond.of(20.0)

        // --- INTERPOLATION MAPS ---
        val SHOT_TIME_MAP =
            InterpolatingDoubleTreeMap().apply {
                put(1.0, 0.75)
                put(2.0, 0.97)
                put(3.0, 1.10)
                put(5.0, 1.35)
            }

        val FLYWHEEL_VELOCITY_MAP =
            InterpolatingDoubleTreeMap().apply {
                put(1.041, HUB_FLYWHEEL_VEL.`in`(RadiansPerSecond))
                put(2.032, 200.5)
                put(3.59511479485, 200.5)
                put(5.0, 220.5)
            }

        val HOOD_ANGLE_MAP =
            InterpolatingDoubleTreeMap().apply {
                put(1.041, HUB_HOOD_ANGLE.`in`(Radians))
                put(2.032, 0.04)
                put(3.59511479485, TRENCH_HOOD_ANGLE.`in`(Radians))
                put(5.0, 0.2806)
            }
    }

    object IntakeConstants {
        // --- HARDWARE IDs ---
        const val LEFT_PIVOT_MOTOR_ID = 40
        const val RIGHT_PIVOT_FOLLOWER_ID = 41
        const val LEFT_ROLLER_MOTOR_ID = 42
        const val RIGHT_ROLLER_FOLLOWER_ID = 43

        // --- HARDWARE CONFIGURATION ---
        val LEFT_PIVOT_NEUTRAL_MODE = NeutralModeValue.Brake
        val LEFT_PIVOT_INVERSION = InvertedValue.Clockwise_Positive
        val RIGHT_PIVOT_FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed

        val LEFT_ROLLER_NEUTRAL_MODE = NeutralModeValue.Coast
        val LEFT_ROLLER_INVERSION = InvertedValue.Clockwise_Positive
        val RIGHT_ROLLER_FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed

        // --- PHYSICAL SPECS & GEARING ---
        const val PIVOT_GEARING_SENSOR_TO_MECH = 52.0
        const val PIVOT_MOI_KG_MM = .16241
        val ARM_LENGTH_METERS = Units.inchesToMeters(8.4)

        const val ROLLER_GEARING = 4.0 / 3
        const val ROLLER_MOI_KG_MM = .0001411489

        // --- CURRENT LIMITS ---
        const val PIVOT_SUPPLY_LIMIT = 15.0
        const val PIVOT_STATOR_LIMIT = 40.0
        const val ROLLER_SUPPLY_LIMIT = 20.0
        const val ROLLER_STATOR_LIMIT = 60.0

        // --- PIVOT STATE SETTINGS ---
        const val STOW_POS_RADS = 0.0
        const val DEPLOY_POS_RADS = 2.269

        const val DEPLOY_VOLTS = 4.0
        const val DEPLOY_HOLD_VOLTS = 0.0
        const val STOW_VOLTS = -4.0
        const val STOW_HOLD_VOLTS = 0.0

        // --- ROLLER VELOCITY SETPOINTS ---
        val INTAKE_VELOCITY: AngularVelocity = RotationsPerSecond.of(80.0)
        val OUTTAKE_VELOCITY: AngularVelocity = RotationsPerSecond.of(-40.0)

        // --- HOMING & VISUALIZATION ---
        const val HOMING_CURRENT_AMPS = 35.0
        const val HOMING_VELOCITY_RAD_PER_SEC = 0.5
        const val HOMING_DEBOUNCE_TIME = 0.3
        const val VIZ_OFFSET_DEG = 33.873
    }

    // INDEXER CONSTANTS STILL SLIGHTLY OFF
    object IndexerConstants {
        // --- HARDWARE IDs ---
        const val WEDGE_INDEXER_ID = 21
        const val FLOOR_INDEXER_ID = 22
        const val TOP_INDEXER_ID = 23

        // --- HARDWARE CONFIGURATION ---
        val WEDGE_NEUTRAL_MODE = NeutralModeValue.Coast
        val WEDGE_INVERSION = InvertedValue.Clockwise_Positive

        val FLOOR_NEUTRAL_MODE = NeutralModeValue.Coast
        val FLOOR_INVERSION = InvertedValue.CounterClockwise_Positive

        val TOP_NEUTRAL_MODE = NeutralModeValue.Coast
        val TOP_INVERSION = InvertedValue.Clockwise_Positive

        // --- CURRENT LIMITS ---
        const val WEDGE_STATOR_LIMIT = 60.0
        const val WEDGE_SUPPLY_LIMIT = 30.0

        const val FLOOR_STATOR_LIMIT = 20.0
        const val FLOOR_SUPPLY_LIMIT = 10.0

        const val TOP_STATOR_LIMIT = 60.0
        const val TOP_SUPPLY_LIMIT = 30.0

        // --- PHYSICAL SPECS & GEARING ---
        const val WEDGE_GEARING = 1.5
        const val WEDGE_MOI_KG_MM = 0.001

        const val FLOOR_GEARING = 27.0 / 14.0
        const val FLOOR_MOI_KG_MM = 0.005

        const val TOP_GEARING = 31.0 / 11.0
        const val TOP_MOI_KG_MM = 0.000000008 // TODO: Find

        // --- WEDGE GAINS ---
        const val WEDGE_KP = 0.5
        const val WEDGE_KI = 0.0
        const val WEDGE_KD = 0.0
        const val WEDGE_KS = 0.05
        const val WEDGE_KV = 0.15

        // --- FLOOR GAINS ---
        const val FLOOR_KP = 1.75
        const val FLOOR_KI = 0.0
        const val FLOOR_KD = 0.0
        const val FLOOR_KS = 0.05
        const val FLOOR_KV = 0.2

        // --- TOP GAINS ---
        const val TOP_KP = 0.25
        const val TOP_KI = 0.0
        const val TOP_KD = 0.0
        const val TOP_KS = 0.05
        const val TOP_KV = 0.34

        // --- SETPOINTS ---
        val SHOOTING_INDEXER_SPEED: AngularVelocity = RadiansPerSecond.of(120.0)
        val INTAKING_INDEXER_SPEED: AngularVelocity = RadiansPerSecond.of(30.0)
    }

    object AimbotConstants {
        // Aimbot PID Constants
        const val AIMBOT_KP = 10.0
        const val AIMBOT_KI = 0.0
        const val AIMBOT_KD = 0.5
    }

    object LEDConstants {
        // led constants
    }

    object VisionConstants {
        // --- CHS ANDYMARK REBUILT FIELD ---
        val REBUILT_FIELD_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark)
//        val REBUILT_FIELD_LAYOUT: AprilTagFieldLayout = AprilTagFieldLayout(frc.team449.subsystems.vision.field_map_mar_07_08_28_22.json)

        const val CAMERA_RIGHT_NAME: String = "limelight-right"
        const val CAMERA_LEFT_NAME: String = "limelight-left"

        // Robot to camera transforms

        val PV_ROBOT_TO_CAMERA_RIGHT: Transform3d = Transform3d(-0.2514092 - 0.079375 + 0.0516, 0.3030474, 0.53594, Rotation3d(0.0, 0.523599, 0.523599))
        var PV_ROBOT_TO_CAMERA_LEFT: Transform3d = Transform3d(-0.2514092 - 0.079375 + 0.0516, -0.3030474, 0.53594, Rotation3d(0.0, 0.523599, 0.523599))
        // TODO CHECK ROTATION - see if poses are accurate on actual field
        // fyi i think right camera might actually be the left one on the actual camera configs

        // --- FILTERING THRESHOLDS --
        const val MAX_AMBIGUITY: Double = 0.3
        const val MAX_Z_ERROR_METERS: Double = 0.5

        // --- STANDARD DEVIATION BASELINES ---
        // std dev baselines for 1 tag @ 1 meter dist
        const val LINEAR_STD_DEV_BASELINE_METERS: Double = 0.02
        const val ANGULAR_STD_DEV_BASELINE_RADIANS: Double = 0.06

        // Standard deviation multipliers for each camera
        // (Adjust to trust some cameras more than others)
        val cameraStdDevFactors: DoubleArray =
            doubleArrayOf(
                1.0, // Camera 0
                1.0, // Camera 1
            )

        // Multipliers to apply for MegaTag 2 observations
        const val linearStdDevMegatag2Factor: Double = 0.5 // More stable than full 3D solve
        const val angularStdDevMegatag2Factor: Double = Double.POSITIVE_INFINITY // No rotation data available
    }
}
