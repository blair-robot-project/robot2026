package frc.team449.subsystems.drive

import choreo.trajectory.SwerveSample
import com.ctre.phoenix6.swerve.SwerveRequest
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyFieldSpeeds
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.controller.PIDController
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
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger

class DriveSubsystem(
    val io: DriveIO
) : SubsystemBase() {
    private val inputs: DriveIOInputsAutoLogged = DriveIOInputsAutoLogged()

    private val autoApplyFieldSpeeds = ApplyFieldSpeeds()

    private val m_translationCharacterization = SwerveRequest.SysIdSwerveTranslation()
    private val m_steerCharacterization = SwerveRequest.SysIdSwerveSteerGains()
    private val m_rotationCharacterization = SwerveRequest.SysIdSwerveRotation()

    private val autoXController = PIDController(5.0, 0.0, 0.0)
    private val autoYController = PIDController(5.0, 0.0, 0.0)
    private val autoThetaController = PIDController(5.0, 0.0, 0.0).apply {
        enableContinuousInput(-Math.PI, Math.PI)
    }

    val sysIdRoutineTranslation = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            null,
            null
        ) { state: SysIdRoutineLog.State -> Logger.recordOutput("SysIdTranslation_State", state.toString()) },
        SysIdRoutine.Mechanism(
            { volts: Voltage -> setControl(m_translationCharacterization.withVolts(volts.`in`(Volts))) },
            null,
            this
        )
    )

    val sysIdRoutineSteer = SysIdRoutine(
        SysIdRoutine.Config(
            null,
            null,
            null
        ) { state: SysIdRoutineLog.State -> Logger.recordOutput("SysIdSteer_State", state.toString()) },
        SysIdRoutine.Mechanism(
            { volts: Voltage -> setControl(m_steerCharacterization.withVolts(volts.`in`(Volts))) },
            null,
            this
        )
    )

    val sysIdRoutineRotation = SysIdRoutine(
        SysIdRoutine.Config( /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second), /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null
        ) // Use default timeout (10 s)
        { state: SysIdRoutineLog.State -> Logger.recordOutput("SysIdRotation_State", state.toString()) },
        SysIdRoutine.Mechanism(
            { output: Voltage ->
                setControl(m_rotationCharacterization.withRotationalRate(output.`in`(Volts)))
                Logger.recordOutput("Rotational_Rate", output.`in`(Volts))
            },
            null,
            this
        )
    )

    override fun periodic() {
        io.updateInputs(inputs)
        io.logModules(inputs)
        Logger.processInputs("DriveInputs", inputs)
    }

    fun setControl(request: SwerveRequest) { io.setControl(request) }

    fun resetOdometry(pose: Pose2d) { io.resetOdometry(pose) }

    fun getPose(): Pose2d { return inputs.Pose }

    fun getRobotRelativeSpeeds(): ChassisSpeeds {
        return inputs.Speeds
    }

    fun seedFieldCentric() {
        if (io is DriveIOHardware) {
            io.seedFieldCentric()
        }
    }

    fun followTrajectory(sample: SwerveSample) {
        val pose = getPose()
        val targetSpeeds = sample.chassisSpeeds

        targetSpeeds.vxMetersPerSecond += autoXController.calculate(pose.x, sample.x)
        targetSpeeds.vyMetersPerSecond += autoYController.calculate(pose.y, sample.y)
        targetSpeeds.omegaRadiansPerSecond += autoThetaController.calculate(pose.rotation.radians, sample.heading)

        setControl(
            autoApplyFieldSpeeds.withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        )
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
}
