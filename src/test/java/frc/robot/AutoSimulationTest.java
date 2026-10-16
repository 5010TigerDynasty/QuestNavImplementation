package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.BLine.Path;

/**
 * Runs each BLine path through the real RobotContainer in HAL + CTRE simulation, the same
 * code path as `./gradlew simulateJava`, and checks the simulated robot ends up at the
 * path's final waypoint. Runs in real time because the CTRE sim loop uses wall-clock time.
 */
class AutoSimulationTest {
    private static final double TIMEOUT_SECONDS = 15.0;
    private static final double LOOP_SECONDS = 0.02;

    private RobotContainer container;

    static List<String> pathNames() {
        return BLineAutos.discoverPathNames();
    }

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.resetData();
        // Blue alliance so the path is not flipped; the flip itself is covered in BLineAutosTest.
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        container = new RobotContainer();
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        CommandScheduler.getInstance().clearComposedCommands();
        // Release the CTRE devices and sim thread so the next test's drivetrain starts clean.
        container.drivetrain.close();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    /** Mimics picking an auto on the dashboard; TimedRobot calls updateValues() every loop. */
    private static void select(String auto) {
        SmartDashboard.putString("Auto Chooser/selected", auto);
        SmartDashboard.updateValues();
    }

    @Test
    void selectedPathIsDrawnOnTheField() {
        select("1-meter-right");
        container.periodic();
        assertEquals(new Path("1-meter-right").getTranslations().size(), container.autos.overlayPoses().size());

        select("Drive Forward (default)");
        container.periodic();
        assertTrue(container.autos.overlayPoses().isEmpty(), "overlay should clear for a non-path auto");
    }

    @ParameterizedTest(name = "robot finishes {0} at its last waypoint")
    @MethodSource("pathNames")
    void robotReachesEndOfPath(String name) throws InterruptedException {
        Path path = new Path(name);
        List<Translation2d> points = path.getTranslations();
        Translation2d end = points.get(points.size() - 1);
        double tolerance = path.getEndTranslationToleranceMeters();

        Command auto = container.autos.get(name);

        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        CommandScheduler.getInstance().schedule(auto);

        long deadline = System.nanoTime() + (long) (TIMEOUT_SECONDS * 1e9);
        while (CommandScheduler.getInstance().isScheduled(auto) && System.nanoTime() < deadline) {
            CommandScheduler.getInstance().run();
            Thread.sleep((long) (LOOP_SECONDS * 1000));
        }

        assertTrue(!CommandScheduler.getInstance().isScheduled(auto),
                name + " did not finish within " + TIMEOUT_SECONDS + " s");
        Pose2d finalPose = container.drivetrain.getPose();
        assertEquals(0.0, finalPose.getTranslation().getDistance(end), tolerance + 0.05,
                name + " finished at " + finalPose + " but should end near " + end);
    }
}
