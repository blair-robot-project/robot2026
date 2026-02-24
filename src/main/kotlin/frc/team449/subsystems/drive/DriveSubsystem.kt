package frc.team449.subsystems.drive

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
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
import java.util.function.Supplier

class DriveSubsystem(
    val io: DriveIO
) : SubsystemBase() {
    val inputs: DriveIOInputsAutoLogged = DriveIOInputsAutoLogged()

    val pose: Pose2d
        get() = inputs.Pose

    override fun periodic() {
        io.updateInputs(inputs)
        io.logModules(inputs)
        Logger.processInputs("DriveInputs", inputs)

        Logger.recordOutput("Vision/Summary/PoseCombined", estimator.estimatedPosition)
    }

    fun setControl(request: SwerveRequest) {
        io.setControl(request)
    }

    fun resetOdometry(pose: Pose2d) {
        io.resetOdometry(pose)
    }
    val rotation: Supplier<Rotation2d> = Supplier { Rotation2d(inputs.gyroAngle * Math.PI / 180) } // TODO !!! this got changed hopefully good now

//    val pose: Supplier<Pose2d> = Supplier { inputs.Pose }

    val estimator = SwerveDrivePoseEstimator(
        SwerveDriveKinematics(
            Translation2d(0.3429, 0.3429),
            Translation2d(-0.3429, 0.3429),
            Translation2d(-0.3429, -0.3429),
            Translation2d(0.3429, -0.3429)
        ),
        inputs.Pose.rotation,
        arrayOf(
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition(),
            SwerveModulePosition()
        ),
        Pose2d()
    )

    fun getRobotRelativeSpeeds(): ChassisSpeeds = inputs.Speeds

    fun seedFieldCentric(): Command {
        return this.runOnce {
            if (io is DriveIOHardware) {
                io.seedFieldCentric()
            }
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
                },
            )
        }
    }

    fun addVisionMeasurement(
        visionRobotPoseMeters: Pose2d,
        timestampSeconds: Double,
        visionMeasurementStdDevs: Matrix<N3, N1>
    ) {
        io.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs)
    }

    fun setStateStdDevs(visionMeasurementStdDevs: Matrix<N3, N1>) {
        io.setStateStdDevs(visionMeasurementStdDevs)
    }

    /* Swerve requests to apply during SysId characterization */
    private val translationCharacterizationRequest = SwerveRequest.SysIdSwerveTranslation()
    private val steerCharacterizationRequest = SwerveRequest.SysIdSwerveSteerGains()
    private val rotationCharacterizationRequest = SwerveRequest.SysIdSwerveRotation()

    // SysId routine for characterizing translation. This is used to find PID gains for the drive motors.
    val sysIDTranslationRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(
                null, // default ramp rate (1 V/s)
                Volts.of(4.0), // dynamic step voltage
                null, // default timeout (10 s)
            ) { state: SysIdRoutineLog.State ->
                Logger.recordOutput(
                    "SysIdTranslation_State",
                    state.toString(),
                )
            },
            Mechanism(
                { output: Voltage -> setControl(translationCharacterizationRequest.withVolts(output)) },
                null,
                this,
            ),
        )

    // SysId routine for characterizing steer. This is used to find PID gains for the steer motors.
    val sysIDSteerRoutine =
        SysIdRoutine(
            SysIdRoutine.Config(
                null, // default ramp rate (1 V/s)
                Volts.of(7.0), // dynamic voltage of 7 V
                null, // default timeout (10 s)
            ) { state: SysIdRoutineLog.State ->
                Logger.recordOutput(
                    "SysIdSteer_State",
                    state.toString(),
                )
            },
            Mechanism(
                { volts: Voltage -> setControl(steerCharacterizationRequest.withVolts(volts)) },
                null,
                this,
            ),
        )

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    val sysIDRotationRoutine =
        SysIdRoutine(
            SysIdRoutine.Config( // This is in radians per second², but SysId only supports "volts per second"
                Volts.of(Math.PI / 6).per(Second), // This is in radians per second, but SysId only supports "volts"
                Volts.of(Math.PI),
                null,
            ) // Use default timeout (10 s)
            // Log state with SignalLogger class
            { state: SysIdRoutineLog.State ->
                Logger.recordOutput(
                    "SysIdRotation_State",
                    state.toString(),
                )
            },
            Mechanism(
                { output: Voltage ->
                    // output is actually radians per second, but SysId only supports "volts"
                    setControl(rotationCharacterizationRequest.withRotationalRate(output.`in`(Volts)))
                    Logger.recordOutput("Rotational_Rate", output.`in`(Volts)) // log requested output for SysId
                },
                null,
                this,
            ),
        )
}
