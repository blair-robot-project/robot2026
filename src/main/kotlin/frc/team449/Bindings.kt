package frc.team449

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.team449.commands.SwerveRequestCommand
import kotlin.math.abs

class Bindings(
    val robotContainer: RobotContainer
) {
    val driver = robotContainer.driveController
    val operator = robotContainer.operatorController

    val joysticksMovedPastDeadband: Trigger =
        Trigger {
            abs(driver.leftY) > Constants.DriveConstants.INTERRUPT_DEADBAND ||
                abs(driver.leftX) > Constants.DriveConstants.INTERRUPT_DEADBAND ||
                abs(driver.rightX) > Constants.DriveConstants.INTERRUPT_DEADBAND
        }.debounce(0.1)

    fun setDefaultCommands() {
        robotContainer.drive.defaultCommand =
            SwerveRequestCommand(
                robotContainer.drive,
                { -driver.leftY },
                { -driver.leftX },
                { -driver.rightX },
            )
    }

    fun bindControls() {
        driver
            .x()
            .onTrue(
                robotContainer.drive
                    .xLock()
                    .until(joysticksMovedPastDeadband),
            )

        driver
            .povUp()
            .onTrue(robotContainer.drive.seedFieldCentric())
    }
}
