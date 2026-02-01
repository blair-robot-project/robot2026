package frc.team449.subsystems.shooter

import edu.wpi.first.units.measure.Angle

class ShooterIOSim : ShooterIO {

    // lots to do

    override fun run(position: Double) {
        // set voltage
    }

    override fun stop() {
        // stop voltage
    }

    override fun setHood(angle: Angle) {
        super.setHood(angle)
    }

    override fun holdHood() {
        super.holdHood()
    }
}
