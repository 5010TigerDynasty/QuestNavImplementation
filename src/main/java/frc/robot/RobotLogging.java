package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Records everything on NetworkTables (and console output) to a {@code .wpilog} file that
 * AdvantageScope opens directly.
 *
 * <p>Where the file goes: on the roboRIO, a USB stick's {@code /logs} folder if one is plugged in,
 * otherwise {@code /home/lvuser/logs}. In simulation, a {@code logs/} folder in the project
 * directory (already git-ignored). Each NT topic is stored as {@code NT:/<topic path>}.
 */
public final class RobotLogging {
    private RobotLogging() {}

    /** Starts logging to the default location. Call once, as early as possible in robot startup. */
    public static void start() {
        DataLogManager.start();
        enableStandardLogging();
    }

    /** Starts logging into {@code directory}; used by tests. */
    static void start(String directory) {
        DataLogManager.start(directory);
        enableStandardLogging();
    }

    private static void enableStandardLogging() {
        // DataLogManager records NT and console output by default; be explicit so it stays that way.
        DataLogManager.logNetworkTables(true);
        DataLogManager.logConsoleOutput(true);
        // Driver Station state, enabled/mode, and joystick inputs, alongside the NT data.
        DriverStation.startDataLog(DataLogManager.getLog());
    }
}
