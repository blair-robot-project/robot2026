package frc.team449.generated

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.*

// https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html
object TunerConstants {
    // Both sets of gains need to be tuned to your individual robot.
    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    // Both sets of gains need to be tuned to your individual robot.
    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private val steerGains: Slot0Configs =
        Slot0Configs()
            .withKP(100.0)
            .withKI(0.0)
            .withKD(0.5)
            .withKS(0.1)
            .withKV(2.62)
            .withKA(0.0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)

    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private val driveGains: Slot0Configs =
        Slot0Configs()
            .withKP(0.1)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.107)
            .withKV(0.119)

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private val kSteerClosedLoopOutput: SwerveModuleConstants.ClosedLoopOutputType =
        SwerveModuleConstants.ClosedLoopOutputType.Voltage

    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private val kDriveClosedLoopOutput: SwerveModuleConstants.ClosedLoopOutputType =
        SwerveModuleConstants.ClosedLoopOutputType.Voltage

    // The type of motor used for the drive motor
    private val kDriveMotorType: DriveMotorArrangement = DriveMotorArrangement.TalonFX_Integrated

    // The type of motor used for the drive motor
    private val kSteerMotorType: SteerMotorArrangement = SteerMotorArrangement.TalonFXS_NEO_JST

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, Fused*/Sync* automatically fall back to Remote*
    private val kSteerFeedbackType: SwerveModuleConstants.SteerFeedbackType =
        SwerveModuleConstants.SteerFeedbackType.TalonFXS_PulseWidth

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private val kSlipCurrent: Current = Units.Amps.of(100.0)

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    private val driveInitialConfigs: TalonFXConfiguration =
        TalonFXConfiguration()
            .withCurrentLimits(
                CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40.0)
            )

    private val steerInitialConfigs: TalonFXSConfiguration =
        TalonFXSConfiguration()
            .withCurrentLimits(
                CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(40.0)
            )

    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private val pigeonConfigs: Pigeon2Configuration? = null

    // CAN bus that the devices are located on;
    // All swerve devices must share the same CAN bus

    val kCANBus: CANBus = CANBus("")

    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot

    val kSpeedAt12Volts: LinearVelocity = Units.FeetPerSecond.of(15.5)

    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private const val kCoupleRatio: Double = 3.5714285714285716

    private const val kDriveGearRatio: Double = 50.0 / 14 * 17 / 27 * 45 / 15
    private const val kSteerGearRatio: Double = 21.428571428571427
    private val kWheelRadius: Distance = Units.Inches.of(2.04)

    private const val kInvertLeftSide: Boolean = true
    private const val kInvertRightSide: Boolean = false

    private const val kPigeonId: Int = 0

    // These are only used for simulation
    private val kSteerInertia: MomentOfInertia = Units.KilogramSquareMeters.of(0.01)
    private val kDriveInertia: MomentOfInertia = Units.KilogramSquareMeters.of(0.01)

    // Simulated voltage necessary to overcome friction
    private val kSteerFrictionVoltage: Voltage = Units.Volts.of(0.2)
    private val kDriveFrictionVoltage: Voltage = Units.Volts.of(0.2)

    val DrivetrainConstants: SwerveDrivetrainConstants =
        SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.name)
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs)

    private val ConstantCreator: SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXSConfiguration, CANcoderConfiguration> =
        SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXSConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)

    // Front Left
    private const val kFrontLeftDriveMotorId: Int = 1
    private const val kFrontLeftSteerMotorId: Int = 2
    private const val kFrontLeftEncoderId: Int = 2
    private val kFrontLeftEncoderOffset: Angle = Units.Degrees.of(-106.52 + 180.0)
    private const val kFrontLeftSteerMotorInverted: Boolean = true
    private const val kFrontLeftEncoderInverted: Boolean = true

    private val kFrontLeftXPos: Distance = Units.Inches.of(10.875)
    private val kFrontLeftYPos: Distance = Units.Inches.of(10.875)

    // Front Right
    private const val kFrontRightDriveMotorId: Int = 3
    private const val kFrontRightSteerMotorId: Int = 4
    private const val kFrontRightEncoderId: Int = 4
    private val kFrontRightEncoderOffset: Angle = Units.Degrees.of(-317.46 + 180.0)
    private const val kFrontRightSteerMotorInverted: Boolean = true
    private const val kFrontRightEncoderInverted: Boolean = true

    private val kFrontRightXPos: Distance = Units.Inches.of(10.875)
    private val kFrontRightYPos: Distance = Units.Inches.of(-10.875)

    // Back Left
    private const val kBackLeftDriveMotorId: Int = 5
    private const val kBackLeftSteerMotorId: Int = 8
    private const val kBackLeftEncoderId: Int = 6
    private val kBackLeftEncoderOffset: Angle = Units.Degrees.of(12.22 + 180.0)
    private const val kBackLeftSteerMotorInverted: Boolean = true
    private const val kBackLeftEncoderInverted: Boolean = true

    private val kBackLeftXPos: Distance = Units.Inches.of(-10.875)
    private val kBackLeftYPos: Distance = Units.Inches.of(10.875)

    // Back Right
    private const val kBackRightDriveMotorId: Int = 7
    private const val kBackRightSteerMotorId: Int = 6
    private const val kBackRightEncoderId: Int = 8
    private val kBackRightEncoderOffset: Angle = Units.Degrees.of(254.49) // -180.0
    private const val kBackRightSteerMotorInverted: Boolean = true
    private const val kBackRightEncoderInverted: Boolean = true

    private val kBackRightXPos: Distance = Units.Inches.of(-10.875)
    private val kBackRightYPos: Distance = Units.Inches.of(-10.875)

    val FrontLeft: SwerveModuleConstants<TalonFXConfiguration, TalonFXSConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId,
            kFrontLeftDriveMotorId,
            kFrontLeftEncoderId,
            kFrontLeftEncoderOffset,
            kFrontLeftXPos,
            kFrontLeftYPos,
            kInvertLeftSide,
            kFrontLeftSteerMotorInverted,
            kFrontLeftEncoderInverted,
        )
    val FrontRight: SwerveModuleConstants<TalonFXConfiguration, TalonFXSConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId,
            kFrontRightDriveMotorId,
            kFrontRightEncoderId,
            kFrontRightEncoderOffset,
            kFrontRightXPos,
            kFrontRightYPos,
            kInvertRightSide,
            kFrontRightSteerMotorInverted,
            kFrontRightEncoderInverted,
        )
    val BackLeft: SwerveModuleConstants<TalonFXConfiguration, TalonFXSConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId,
            kBackLeftDriveMotorId,
            kBackLeftEncoderId,
            kBackLeftEncoderOffset,
            kBackLeftXPos,
            kBackLeftYPos,
            kInvertLeftSide,
            kBackLeftSteerMotorInverted,
            kBackLeftEncoderInverted,
        )
    val BackRight: SwerveModuleConstants<TalonFXConfiguration, TalonFXSConfiguration, CANcoderConfiguration> =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId,
            kBackRightDriveMotorId,
            kBackRightEncoderId,
            kBackRightEncoderOffset,
            kBackRightXPos,
            kBackRightYPos,
            kInvertRightSide,
            kBackRightSteerMotorInverted,
            kBackRightEncoderInverted,
        )
}
