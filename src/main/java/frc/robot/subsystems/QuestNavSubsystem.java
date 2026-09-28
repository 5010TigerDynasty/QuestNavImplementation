package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
    QuestNav questNav = new QuestNav();
    CommandSwerveDrivetrain drivetrain;
    Transform3d ROBOT_TO_QUEST = new Transform3d(-0.02, -0.3 , 0.3, new Rotation3d(0, 0, -Math.PI/2)); // Translation and rotation from robot to QuestNav camera
    Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02, 0.02 , 0.0872665); // Translation from robot to QuestNav camera
    
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


        for(PoseFrame poseFrame : poseFrames) {
            if(poseFrame.isTracking()) {
                Pose3d questPose = poseFrame.questPose3d();
                double timestamp = poseFrame.dataTimestamp(); // Timestamp of the pose frame data

                Pose3d robotPose = questPose.transformBy(ROBOT_TO_QUEST.inverse());
        
                drivetrain.addVisionMeasurement(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
        }
        }
     
        // This method will be called once per scheduler run
    }
    

}

