package frc.team449.subsystems.drive

import choreo.trajectory.SwerveSample
import com.ctre.phoenix6.swerve.SwerveRequest
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.team449.Constants
import frc.team449.Robot
import org.littletonrobotics.junction.Logger

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
    val config: RobotConfig = RobotConfig.fromGUISettings()

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
