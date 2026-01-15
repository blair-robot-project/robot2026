package frc.team449.subsystems.drive

import com.ctre.phoenix6.SignalLogger
import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
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
import frc.team449.Constants
import frc.team449.Robot
import org.littletonrobotics.junction.Logger

class DriveSubsystem(
    val io: DriveIO
) : SubsystemBase() {
    private val inputs: DriveIOInputsAutoLogged = DriveIOInputsAutoLogged()

    private val m_translationCharacterization = SwerveRequest.SysIdSwerveTranslation()
    private val m_steerCharacterization = SwerveRequest.SysIdSwerveSteerGains()
    private val m_rotationCharacterization = SwerveRequest.SysIdSwerveRotation()

    private val sysIdRoutineTranslation = SysIdRoutine(
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

    private val sysIdRoutineSteer = SysIdRoutine(
        SysIdRoutine.Config(
            null, null, null
        ) { state: SysIdRoutineLog.State -> Logger.recordOutput("SysIdSteer_State", state.toString()) },
        SysIdRoutine.Mechanism(
            { volts: Voltage -> setControl(m_steerCharacterization.withVolts(volts.`in`(Volts))) },
            null,
            this
        )
    )

    private val sysIdRoutineRotation = SysIdRoutine(
        SysIdRoutine.Config( /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second), /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null
        ) // Use default timeout (10 s)
        { state: SysIdRoutineLog.State ->
            SignalLogger.writeString(
                "SysIdRotation_State",
                state.toString()
            )
        },
        SysIdRoutine.Mechanism(
            { output: Voltage ->
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.`in`(Volts)))
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.`in`(Volts))
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


    private val xController: PIDController
        get() = PIDController(5.0, 0.0, 0.0)
    private val yController: PIDController
        get() = PIDController(5.0, 0.0, 0.0)
    private val headingController: PIDController
        get() = PIDController(5.0, 0.0, 0.0)

    var desiredAngle = 0.0
    var desiredOmega = 0.0

    init {
        headingController.enableContinuousInput(-Math.PI, Math.PI)


    }

    fun followTrajectory(robot: Robot, sample: SwerveSample) {
        desiredAngle = MathUtil.angleModulus(sample.heading)
        desiredOmega = sample.omega

        val speeds = ChassisSpeeds(
            sample.vx + xController.calculate(getPose().x, sample.x),
            sample.vy + yController.calculate(getPose().y, sample.y),
            sample.omega + headingController.calculate(getPose().rotation.minus(Rotation2d.fromRadians(MathUtil.angleModulus(sample.heading))).radians,)
        )

        val newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            heading
        )

        println("desiredAngle: $desiredAngle " )
        println("Angle: $heading " )
        println("desiredOmega: $desiredOmega " )
        println("Omega: ${speeds.omegaRadiansPerSecond} " )
        // Apply generated speeds
        setControl(
            SwerveRequest.RobotCentric()
                .withVelocityX(newSpeeds.vxMetersPerSecond)
                .withVelocityY(newSpeeds.vyMetersPerSecond)
                .withRotationalRate(newSpeeds.omegaRadiansPerSecond)
        )

    }

    var heading: Rotation2d
        get() = Rotation2d(MathUtil.angleModulus(getPose().rotation.radians))
        set(value) {
            inputs.Pose = Pose2d(Translation2d(getPose().x, getPose().y), value)
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
