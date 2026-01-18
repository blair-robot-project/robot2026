package frc.team449.subsystems.drive

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism
import org.littletonrobotics.junction.Logger
import java.util.function.Consumer

class DriveSubsystem(
    val io: DriveIO
) : SubsystemBase() {
    private val inputs: DriveIOInputsAutoLogged = DriveIOInputsAutoLogged()

    override fun periodic() {
        io.updateInputs(inputs)
        io.logModules(inputs)
        Logger.processInputs("DriveInputs", inputs)
    }

    fun setControl(request: SwerveRequest) { io.setControl(request) }

    fun resetOdometry(pose: Pose2d) { io.resetOdometry(pose) }

    fun getPose(): Pose2d {
        return inputs.Pose
    }

    fun getRobotRelativeSpeeds(): ChassisSpeeds {
        return inputs.Speeds
    }

    fun seedFieldCentric() {
        if (io is DriveIOHardware) {
            io.seedFieldCentric()
        }
    }

    // should only be called in driverStationConnected() to prevent null alliance
    fun setOperatorPerspectiveForward() {
        if (io is DriveIOHardware) {
            io.setOperatorPerspectiveForward(
                if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                    Rotation2d.kZero
                } else {
                    Rotation2d.k180deg
                }
            )
        }
    }

    fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double, visionMeasurementStdDevs: Matrix<N3, N1>) {
        io.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs)
    }

    fun setStateStdDevs(visionMeasurementStdDevs: Matrix<N3, N1>) {
        io.setStateStdDevs(visionMeasurementStdDevs)
    }

    /* Swerve requests to apply during SysId characterization */
    private val m_translationCharacterization = SwerveRequest.SysIdSwerveTranslation()
    private val m_steerCharacterization = SwerveRequest.SysIdSwerveSteerGains()
    private val m_rotationCharacterization = SwerveRequest.SysIdSwerveRotation()

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private val m_sysIdRoutineTranslation = SysIdRoutine(
        SysIdRoutine.Config(
            null, // Use default ramp rate (1 V/s)
            Volts.of(4.0), // Reduce dynamic step voltage to 4 V to prevent brownout
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            Consumer<SysIdRoutineLog.State> { state: SysIdRoutineLog.State ->
                SignalLogger.writeString(
                    "SysIdTranslation_State",
                    state.toString()
                )
            }
        ),
        Mechanism(
            Consumer<Voltage> { output: Voltage? -> setControl(m_translationCharacterization.withVolts(output)) },
            null,
            this
        )
    )

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private val m_sysIdRoutineSteer = SysIdRoutine(
        SysIdRoutine.Config(
            null, // Use default ramp rate (1 V/s)
            Volts.of(7.0), // Use dynamic voltage of 7 V
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            Consumer<SysIdRoutineLog.State> { state: SysIdRoutineLog.State ->
                SignalLogger.writeString(
                    "SysIdSteer_State",
                    state.toString()
                )
            }
        ),
        Mechanism(
            Consumer<Voltage> { volts: Voltage? -> setControl(m_steerCharacterization.withVolts(volts)) },
            null,
            this
        )
    )

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private val m_sysIdRoutineRotation = SysIdRoutine(
        SysIdRoutine.Config( /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second), /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            Consumer<SysIdRoutineLog.State> { state: SysIdRoutineLog.State ->
                SignalLogger.writeString(
                    "SysIdRotation_State",
                    state.toString()
                )
            }
        ),
        Mechanism(
            Consumer<Voltage> { output: Voltage ->
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.`in`(Volts)))
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.`in`(Volts))
            },
            null,
            this
        )
    )

    /* The SysId routine to test */
    private val m_sysIdRoutineToApply = m_sysIdRoutineTranslation

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by [.m_sysIdRoutineToApply].
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    fun sysIdQuasistatic(direction: SysIdRoutine.Direction?): Command {
        return m_sysIdRoutineToApply.quasistatic(direction)
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by [.m_sysIdRoutineToApply].
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    fun sysIdDynamic(direction: SysIdRoutine.Direction?): Command {
        return m_sysIdRoutineToApply.dynamic(direction)
    }
}
