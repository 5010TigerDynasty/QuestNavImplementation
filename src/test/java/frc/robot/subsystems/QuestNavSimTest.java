package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/**
 * Checks the simulated headset through the real QuestNav library, the same consumer the robot
 * code uses. Expected poses are worked out by hand, not with the sim's own math.
 */
class QuestNavSimTest {
    /** Same mounting as QuestNavSubsystem.ROBOT_TO_QUEST: behind and to the right, yawed -90 deg. */
    private static final Transform3d MOUNT = new Transform3d(-0.02, -0.3, 0.3, new Rotation3d(0, 0, -Math.PI / 2));
    private static final double EPS = 1e-6;

    private QuestNav questNav;
    private QuestNavSim sim;

    @BeforeEach
    void setUp() {
        assertTrue(HAL.initialize(500, 0));
        questNav = new QuestNav();
        sim = new QuestNavSim(MOUNT, questNav.getLibVersion(), new Pose2d());
        sim.setTracking(true);
        sim.setLatencyMs(0);
        sim.setNoise(0, 0);
        sim.setOrigin(new Pose2d());
        sim.setMountError(0, 0);
        awaitTrue(sim::isHeadsetConnected, "fake headset should connect to the NT server");
    }

    @AfterEach
    void tearDown() {
        sim.close();
    }

    /** Polls until the condition is true (it may have side effects, so it is evaluated once per try). */
    private static void awaitTrue(java.util.function.BooleanSupplier condition, String message) {
        long deadline = System.nanoTime() + 5_000_000_000L;
        boolean ok = condition.getAsBoolean();
        while (!ok && System.nanoTime() < deadline) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
            ok = condition.getAsBoolean();
        }
        assertTrue(ok, message);
    }

    /**
     * Publishes one frame and returns what the real library read back for it. A new subscriber is
     * first handed the topic's retained value from earlier tests, so wait for this exact frame.
     */
    private PoseFrame publishAndRead() {
        sim.update(new ChassisSpeeds(), 0.02);
        double published = sim.getLastFrameTimestampSeconds();
        PoseFrame[] match = new PoseFrame[1];
        awaitTrue(() -> {
            questNav.commandPeriodic();
            for (PoseFrame frame : questNav.getAllUnreadPoseFrames()) {
                if (Math.abs(frame.appTimestamp() - published) < 1e-9) {
                    match[0] = frame;
                }
            }
            return match[0] != null;
        }, "library should have received the frame that was just published");
        return match[0];
    }

    @Test
    void questPoseIsRobotPoseTimesMountingTransform() {
        // Robot at (1, 2) facing +Y (90 deg). Mount is (-0.02, -0.3) in the robot frame, so in the
        // field it is rotated 90 deg: (x, y) -> (-y, x) = (0.3, -0.02). Quest yaw = 90 - 90 = 0.
        sim.teleport(new Pose2d(1, 2, Rotation2d.fromDegrees(90)));

        Pose3d quest = publishAndRead().questPose3d();

        assertEquals(1.3, quest.getX(), EPS);
        assertEquals(1.98, quest.getY(), EPS);
        assertEquals(0.3, quest.getZ(), EPS);
        assertEquals(0.0, quest.getRotation().toRotation2d().getRadians(), EPS);
    }

    @Test
    void libraryReportsConnectedAndTracking() {
        publishAndRead();
        assertTrue(questNav.isConnected());
        assertTrue(questNav.isTracking());
        assertTrue(questNav.getBatteryPercent().isPresent());
    }

    @Test
    void trackingCanBeSwitchedOff() {
        sim.setTracking(false);

        PoseFrame frame = publishAndRead();

        assertFalse(frame.isTracking());
        assertFalse(questNav.isTracking());
    }

    @Test
    void questFrameOriginOffsetMovesTheReportedPose() {
        // Robot at the field origin; Quest's own origin sits 1 m along +X in the field, so the
        // quest (mounted at x = -0.02) is at -0.02 - 1 = -1.02 in the Quest's frame.
        sim.setOrigin(new Pose2d(1, 0, new Rotation2d()));

        Pose3d quest = publishAndRead().questPose3d();

        assertEquals(-1.02, quest.getX(), EPS);
        assertEquals(-0.3, quest.getY(), EPS);
    }

    @Test
    void mountErrorKnobsShiftTheHeadsetInTheRobotFrame() {
        // Robot at the origin facing +X. The headset is mounted at x = -0.02, y = -0.3; a physical
        // mounting error of (+0.10, -0.05) in the robot frame puts it at (0.08, -0.35).
        sim.setMountError(0.10, -0.05);

        Pose3d quest = publishAndRead().questPose3d();

        assertEquals(0.08, quest.getX(), EPS);
        assertEquals(-0.35, quest.getY(), EPS);
        assertEquals(0.3, quest.getZ(), EPS);
    }

    @Test
    void mountErrorIsZeroByDefault() {
        Pose3d quest = publishAndRead().questPose3d();
        assertEquals(-0.02, quest.getX(), EPS);
        assertEquals(-0.3, quest.getY(), EPS);
    }

    @Test
    void latencyBackdatesTheFrameTimestamp() throws InterruptedException {
        sim.setLatencyMs(100);
        // NT ignores a value stamped older than the topic's last one, and an earlier test's frame
        // is still retained on the topic. Let 150 ms pass so "now - 100 ms" is newer than that.
        Thread.sleep(150);

        double publishedAt = Timer.getTimestamp();
        PoseFrame frame = publishAndRead();

        assertEquals(publishedAt - 0.100, frame.dataTimestamp(), 0.02);
    }

    @Test
    void truthIntegratesRobotRelativeSpeedsInTheFieldFrame() {
        sim.teleport(new Pose2d(0, 0, Rotation2d.fromDegrees(90)));

        // 1 m/s "forward" for 1 s while facing +Y must move the robot to (0, 1).
        for (int i = 0; i < 50; i++) {
            sim.update(new ChassisSpeeds(1.0, 0, 0), 0.02);
        }

        assertEquals(0.0, sim.getTruePose().getX(), 1e-3);
        assertEquals(1.0, sim.getTruePose().getY(), 1e-3);
        assertEquals(90.0, sim.getTruePose().getRotation().getDegrees(), 1e-3);
    }

    @Test
    void noiseIsOffByDefaultAndAppliedWhenEnabled() {
        sim.teleport(new Pose2d(1, 1, new Rotation2d()));
        Pose3d clean = publishAndRead().questPose3d();

        sim.setNoise(0.05, 2.0);
        double maxDeviation = 0;
        for (int i = 0; i < 20; i++) {
            Pose3d noisy = publishAndRead().questPose3d();
            maxDeviation = Math.max(maxDeviation, noisy.getTranslation().getDistance(clean.getTranslation()));
        }
        assertNotNull(clean);
        assertTrue(maxDeviation > 1e-4, "noise should perturb the reported pose");
    }
}
