package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.QuestNavSim;

/**
 * Runs QuestNavSubsystem's position code end to end: simulated headset, real QuestNav library,
 * the subsystem's periodic(), and the drivetrain's pose estimator, in HAL + CTRE simulation.
 */
class QuestNavSimulationTest {
    private RobotContainer container;
    private QuestNavSim sim;

    @BeforeEach
    void setUp() throws InterruptedException {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.resetData();
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        container = new RobotContainer();
        sim = container.questNavSubsystem.getSim().orElseThrow();
        sim.setLatencyMs(0);
        sim.setNoise(0, 0);
        sim.setTracking(true);
        long deadline = System.nanoTime() + 5_000_000_000L;
        while (!sim.isHeadsetConnected() && System.nanoTime() < deadline) {
            Thread.sleep(5);
        }
        assertTrue(sim.isHeadsetConnected(), "fake headset should connect to the NT server");
    }

    @AfterEach
    void tearDown() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().unregisterAllSubsystems();
        CommandScheduler.getInstance().clearComposedCommands();
        sim.close();
        container.drivetrain.close();
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    private void runLoops(double seconds) throws InterruptedException {
        long end = System.nanoTime() + (long) (seconds * 1e9);
        while (System.nanoTime() < end) {
            CommandScheduler.getInstance().run();
            container.periodic();
            Thread.sleep(20);
        }
    }

    @Test
    void visionPullsAKnockedOffEstimateBackToTheTruePose() throws InterruptedException {
        runLoops(0.5);
        // Corrupt only the estimator (the sim's true pose does not move).
        container.drivetrain.resetTranslation(new Translation2d(2.0, 2.0));
        assertTrue(container.drivetrain.getPose().getTranslation().getDistance(sim.getTruePose().getTranslation()) > 2.0);

        runLoops(3.0);

        double error = container.drivetrain.getPose().getTranslation().getDistance(sim.getTruePose().getTranslation());
        assertEquals(0.0, error, 0.05, "estimate should converge to the true pose, error was " + error);
    }

    @Test
    void estimateStaysPutWhenVisionIsNotTracking() throws InterruptedException {
        runLoops(0.5);
        sim.setTracking(false);
        container.drivetrain.resetTranslation(new Translation2d(2.0, 2.0));

        runLoops(1.0);

        assertEquals(2.0, container.drivetrain.getPose().getX(), 0.05);
        assertEquals(2.0, container.drivetrain.getPose().getY(), 0.05);
    }

    @Test
    void resetPoseTeleportsTheSimulatedRobotAndQuestTogether() throws InterruptedException {
        runLoops(0.3);
        Pose2d target = new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(45));

        container.drivetrain.resetPose(target);
        runLoops(1.0);

        assertEquals(0.0, sim.getTruePose().getTranslation().getDistance(target.getTranslation()), 1e-3);
        // Quest still agrees with the reset, so vision must not drag the estimate away from it.
        assertEquals(0.0, container.drivetrain.getPose().getTranslation().getDistance(target.getTranslation()), 0.05);
    }

    @Test
    void getRobotPoseWorksInSimulationAndFollowsSoftReset() throws InterruptedException {
        runLoops(0.5);
        assertTrue(container.questNavSubsystem.getRobotPose().isPresent(), "needs a frame from the sim headset");

        Pose3d target = new Pose3d(1.0, 1.0, 0.0, new Rotation3d());
        container.questNavSubsystem.resetPose(target);
        // Move the true robot 0.5 m along +X without changing heading.
        container.drivetrain.resetPose(new Pose2d(0.5, 0.0, new Rotation2d()));
        runLoops(0.5);

        Pose3d pose = container.questNavSubsystem.getRobotPose().orElseThrow();
        assertEquals(1.5, pose.getX(), 0.02);
        assertEquals(1.0, pose.getY(), 0.02);
    }

    @Test
    void offsetCalibrationRecoversAWrongMountingTransform() throws InterruptedException {
        // The code assumes one mount; physically the headset is 10 cm forward and 5 cm right of that.
        // Rotation is about the true robot center, so calibration should report that shift.
        Transform3d assumed = container.questNavSubsystem.getRobotToQuest();
        Translation3d shifted = assumed.getTranslation().plus(new Translation3d(0.10, -0.05, 0.0));
        sim.setMountTransform(new Transform3d(shifted, assumed.getRotation()));
        // Start at a clean origin so the spin is the only motion.
        container.drivetrain.resetPose(new Pose2d());
        runLoops(0.5);

        DriverStationSim.setAutonomous(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        Command calibrate = container.questNavSubsystem.determineOffsetToRobotCenter(container.drivetrain);
        CommandScheduler.getInstance().schedule(calibrate);
        runLoops(4.5); // one 3 s spin plus margin; the command repeats, so the average is still valid
        CommandScheduler.getInstance().cancel(calibrate);

        double[] offset = SmartDashboard.getNumberArray(
                "questnav/Quest Calculated Offset to Robot Center", new double[0]);
        assertEquals(2, offset.length, "calibration should have produced a sample");
        assertEquals(0.10, offset[0], 0.02);
        assertEquals(-0.05, offset[1], 0.02);
    }
}
