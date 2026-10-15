package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
    QuestNav questNav = new QuestNav();
    CommandSwerveDrivetrain drivetrain;
    Transform3d ROBOT_TO_QUEST = new Transform3d(-0.02, -0.3, 0.3, new Rotation3d(0, 0, -Math.PI / 2)); // Translation
                                                                                                        // and rotation
                                                                                                        // from robot to
                                                                                                        // QuestNav
                                                                                                        // camera
    Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02, 0.0872665); // Translation from robot to QuestNav
                                                                               // camera

    // Soft reset state — applied in code without sending a command to the headset
    private Pose3d softResetBase = new Pose3d();
    private Pose3d softResetTarget = new Pose3d();
    private PoseFrame latestPoseFrame = null;

    private Translation2d _calculatedOffsetToRobotCenter = new Translation2d();
    private int _calculatedOffsetToRobotCenterCount = 0;
    private String networkTableRoot = "questnav";

    public QuestNavSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // Initialization code for the QuestNav subsystem
        questNav.onConnected(() -> DriverStation.reportWarning("QuestNav connected", false));
        questNav.onDisconnected(() -> DriverStation.reportError("QuestNav disconnected", false));
    }

    @Override
    public void periodic() {
        questNav.commandPeriodic();
        PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

        SmartDashboard.putBoolean("QuestNav/Connected", questNav.isConnected());
        SmartDashboard.putBoolean("QuestNav/Tracking", questNav.isTracking());
        SmartDashboard.putNumber("QuestNav/Latency", questNav.getLatency());
        questNav.getBatteryPercent().ifPresent(
                b -> SmartDashboard.putNumber("QuestNav/Battery%", b));
        questNav.getTrackingLostCounter().ifPresent(
                c -> SmartDashboard.putNumber("QuestNav/TrackingLostCount", c));

        for (PoseFrame poseFrame : poseFrames) {
            if (poseFrame.isTracking()) {
                Pose3d questPose = poseFrame.questPose3d();
                double timestamp = poseFrame.dataTimestamp(); // Timestamp of the pose frame data

                Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
                latestPoseFrame = poseFrame;
                drivetrain.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
            }
        }

        // This method will be called once per scheduler run
    }

    /**
     * Performs a soft reset: recalculates the pose offset in code without sending
     * any command to the
     * Quest headset. This is instantaneous and avoids the delay of a hard reset
     * command. The
     * soft-reset transform is stored and applied on top of the raw headset pose in
     * {@link
     * #getRobotPose()}, {@link #getPosition()}, and {@link #getRotation()}.
     */
    public void softReset(Pose3d pose) {
        // Remember the raw pose at reset time and the pose it should map to; later
        // poses are the motion since reset, applied in the target pose's frame.
        softResetBase = getProcessedRobotPose();
        softResetTarget = pose;
    }

    boolean isConnected() {
        return questNav.isConnected();
    }

    private Pose3d getProcessedRobotPose() {
        if (latestPoseFrame == null) {
            return new Pose3d();
        }
        return getRobotPoseFromQuestPose(latestPoseFrame.questPose3d());
    }

    private Pose3d getRobotPoseFromQuestPose(Pose3d questPose) {
        return questPose.transformBy(ROBOT_TO_QUEST.inverse());
    }

    public void resetPose(Pose3d pose) {
        if (isConnected()) {
            SmartDashboard.putBoolean(networkTableRoot + "/Reset Pose", true);
            softReset(pose);
        }
    }

    public Optional<Pose3d> getRobotPose() {
        if (RobotBase.isReal()) {
            if (latestPoseFrame == null) {
                return Optional.empty();
            }
            return Optional.of(softResetTarget.transformBy(
                    new Transform3d(softResetBase, getProcessedRobotPose())));
        } else {
            return Optional.empty();
        }
    }

    /** Minimum (1 - cos(angle)) for a sample to be usable; the estimate blows up near 0. */
    private static final double MIN_ONE_MINUS_COS = 0.1;

    /**
     * Offset of the tracked robot origin from the rotation center, from the pose
     * change since the last soft reset. Empty if there is no usable sample.
     */
    private Optional<Translation2d> calculateOffsetToRobotCenter() {
        if (!isConnected()) {
            return Optional.empty();
        }
        Optional<Pose2d> currentPose = getRobotPose().map(Pose3d::toPose2d);
        if (currentPose.isEmpty()) {
            return Optional.empty();
        }

        Rotation2d angle = currentPose.get().getRotation();
        Translation2d displacement = currentPose.get().getTranslation();

        double denominator = 2 * (1 - angle.getCos());
        if (denominator < 2 * MIN_ONE_MINUS_COS) {
            return Optional.empty();
        }

        double x = ((angle.getCos() - 1) * displacement.getX() + angle.getSin() * displacement.getY())
                / denominator;
        double y = ((-1 * angle.getSin()) * displacement.getX() + (angle.getCos() - 1) * displacement.getY())
                / denominator;

        return Optional.of(new Translation2d(x, y));
    }

    public Command determineOffsetToRobotCenter(CommandSwerveDrivetrain drivetrain) {
        // Each iteration soft-resets the pose, spins ~1.5 rad, then takes one sample, so
        // every sample uses a well-conditioned angle. Bind with whileTrue to stop it.
        return Commands.repeatingSequence(
                Commands.runOnce(() -> resetPose(new Pose3d())),
                drivetrain.applyRequest(() -> new SwerveRequest.FieldCentric()
                        .withVelocityX(0.0)
                        .withVelocityY(0.0)
                        .withRotationalRate(0.5))
                        .withTimeout(3.0),
                Commands.runOnce(
                        () -> calculateOffsetToRobotCenter().ifPresent(offset -> {
                            _calculatedOffsetToRobotCenter = _calculatedOffsetToRobotCenter
                                    .times(
                                            (double) _calculatedOffsetToRobotCenterCount
                                                    / (_calculatedOffsetToRobotCenterCount + 1))
                                    .plus(offset.div(_calculatedOffsetToRobotCenterCount + 1));
                            _calculatedOffsetToRobotCenterCount++;

                            SmartDashboard.putNumberArray(
                                    networkTableRoot + "/Quest Calculated Offset to Robot Center",
                                    new double[] {
                                            _calculatedOffsetToRobotCenter.getX(), _calculatedOffsetToRobotCenter.getY()
                                    });
                        })))
                .beforeStarting(() -> {
                    _calculatedOffsetToRobotCenter = new Translation2d();
                    _calculatedOffsetToRobotCenterCount = 0;
                });
    }
}
