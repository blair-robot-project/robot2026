package frc.team449.subsystems.drive

import com.ctre.phoenix6.swerve.SwerveModule
import com.ctre.phoenix6.swerve.SwerveRequest
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class DriveSubsystem(
    val io: DriveIO
) : SubsystemBase() {
    val inputs: DriveIOInputsAutoLogged = DriveIOInputsAutoLogged()

    private val stopRequest: ApplyRobotSpeeds = ApplyRobotSpeeds().withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)

    override fun periodic() {
        io.updateInputs(inputs)
        io.logModules(inputs)
        Logger.processInputs("DriveInputs", inputs)
    }

    fun setControl(request: SwerveRequest) { io.setControl(request) }

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

    fun resetOdometry(pose: Pose2d) { io.resetOdometry(pose) }

    fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double, visionMeasurementStdDevs: Matrix<N3, N1>) {
        io.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs)
    }

    fun setStateStdDevs(visionMeasurementStdDevs: Matrix<N3, N1>) {
        io.setStateStdDevs(visionMeasurementStdDevs)
    }
}
