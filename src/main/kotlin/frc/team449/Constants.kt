package frc.team449

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.RobotBase
import kotlin.math.PI
import kotlin.math.pow

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

        val HOOD_TOLERANCE = Degrees.of(5.0) // TODO: refine

        const val FLYWHEEL_GEARING = 32.0 / 18 //
        const val HOOD_GEARING = 6.0 * 15 // TODO: rough estimate

        const val HOOD_MOMENT_OF_INERTIA = 0.0694270649
        val HOOD_LENGTH = Units.inchesToMeters(5.91)
        val FLYWHEEL_MOI = 0.5 * Units.lbsToKilograms(1.5) * Units.inchesToMeters(4.0).pow(2.0)

        const val CURRENT_HOMING_VOLTAGE = 2.0
        const val CURRENT_HOMING_STATOR_THRESH = 45.0 // amps
    }

    object LEDConstants {
        // led constants
    }

    // other subsystem constants when applicable

    object IntakeConstants {
        // config constants
        const val PIVOT_MOTOR_ID = 40
        const val PIVOT_FOLLOWER_ID = 41
        const val ROLLER_FOLLOWER_ID = 42
        const val ROLLER_MOTOR_ID = 43

        val ROLLER_FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed
        val PIVOT_FOLLOWER_ALIGNMENT = MotorAlignmentValue.Opposed
        const val PIVOT_GEARING_SENSOR_TO_MECH = 50.0 // TODO: find this value
        const val PIVOT_MOI = 0.1549510896 // TODO: verify this
        val ARM_LENGTH: Distance = Meters.of(0.2996692)

        val PIVOT_CURRENT_CONFIG: CurrentLimitsConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40.0)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(120.0)

        val PIVOT_OUTPUT_CONFIG: MotorOutputConfigs =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive) // TODO: Find

        val PIVOT_FEEDBACK_CONFIG: FeedbackConfigs =
            FeedbackConfigs()
                .withSensorToMechanismRatio(PIVOT_GEARING_SENSOR_TO_MECH)

        val pivotSlot0Configs: Slot0Configs =
            Slot0Configs()
                .withKP(5.0)
                .withKG(0.1)

        val PIVOT_CONFIG: TalonFXConfiguration =
            TalonFXConfiguration()
                .withCurrentLimits(PIVOT_CURRENT_CONFIG)
                .withMotorOutput(PIVOT_OUTPUT_CONFIG)
                .withFeedback(PIVOT_FEEDBACK_CONFIG)
                .withSlot0(pivotSlot0Configs)

        val ROLLER_CURRENT_CONFIG: CurrentLimitsConfigs =
            CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40.0)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(120.0)

        val ROLLER_LEADER_OUTPUT_CONFIG: MotorOutputConfigs =
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive) // TODO: Find

        val rollerSlot0Configs: Slot0Configs =
            Slot0Configs()
                .withKP(6.0)
                .withKV(0.12)

        val ROLLER_CONFIG: TalonFXConfiguration =
            TalonFXConfiguration()
                .withCurrentLimits(ROLLER_CURRENT_CONFIG)
                .withMotorOutput(ROLLER_LEADER_OUTPUT_CONFIG)
//                .withSlot0(rollerSlot0Configs)

        const val HOMING_DEBOUNCE_TIME = 0.5
        val HOMING_DEBOUNCE_TYPE = Debouncer.DebounceType.kRising
        val HOMING_TIME_OUT: Time = Seconds.of(0.7)

        // setpoint constants
        val STOW_POSITION: Angle = Degrees.of(92.0) // TODO: Find
        val DEPLOY_POSITION: Angle = Degrees.of(0.0) // TODO: Find

        val INTAKE_VOLTAGE: Voltage = Volts.of(8.0)
        val OUTTAKE_VOLTAGE: Voltage = Volts.of(-8.0)
        val DEPLOY_VOLTAGE: Voltage = Volts.of(-8.0)
        val STOW_VOLTAGE: Voltage = Volts.of(8.0)

//        val INTAKE_VELOCITY: AngularVelocity = RotationsPerSecond.of(45.0)


        val CURRENT_HOMING_CURRENT_LIMIT = Amps.of(20.0)
        val CURRENT_HOMING_TIME_LIMIT = Seconds.of(0.5)
        val CURRENT_HOMING_VEL_LIMIT = RadiansPerSecond.of(0.5)
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
