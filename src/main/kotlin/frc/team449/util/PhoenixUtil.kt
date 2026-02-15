package frc.team449.util

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import java.util.function.Supplier

object PhoenixUtil {
    /** Attempts to run the command until no error is produced.  */
    fun tryUntilOk(maxAttempts: Int, command: Supplier<StatusCode>) {
        for (i in 0 until maxAttempts) {
            val error = command.get()
            if (error.isOK) break
        }
    }

    private var rioSignals: Array<BaseStatusSignal> = emptyArray()

    fun registerSignals(vararg signals: BaseStatusSignal) {
        rioSignals += signals
    }

    fun refreshAll() {
        if (rioSignals.isEmpty()) {
            return
        }

        BaseStatusSignal.refreshAll(*rioSignals)
    }


}
