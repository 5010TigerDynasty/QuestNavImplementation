package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.BLine.Path;

/** Pure-logic tests: no HAL or robot hardware simulation required. */
class BLineAutosTest {
    @Test
    void discoversEveryPathInDeployDirectory() {
        List<String> names = BLineAutos.discoverPathNames();
        assertTrue(names.containsAll(List.of(
                "1-meter-right", "hubtorighttrench", "lefttrenchtohub", "righttrenchtolefttrench")),
                "discovered: " + names);
    }

    @Test
    void everyDiscoveredPathLoadsAndIsValid() {
        for (String name : BLineAutos.discoverPathNames()) {
            assertTrue(new Path(name).isValid(), name + " should be a valid BLine path");
        }
    }

    @Test
    void redAllianceOverlayIsFlippedAndDoesNotMutateTheOriginal() {
        Path blue = new Path("1-meter-right");
        Pose2d before = blue.getStartPose();

        Path red = BLineAutos.pathForAlliance(blue, true);

        assertNotEquals(before.getX(), red.getStartPose().getX(), 1e-6, "red overlay should be flipped");
        assertEquals(before, blue.getStartPose(), "original path must be unchanged");
        assertEquals(before, BLineAutos.pathForAlliance(blue, false).getStartPose());
    }
}
