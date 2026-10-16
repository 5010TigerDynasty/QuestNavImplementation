package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.lib.BLine.FollowPath;

/**
 * Publishes everything BLine's {@link FollowPath} logs (target poses, path points, flags) to
 * NetworkTables under {@code /BLine/<key>}, so it can be viewed in AdvantageScope or the sim GUI.
 */
public final class BLineLogging {
    private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("BLine");

    private static final Map<String, StructPublisher<Pose2d>> poses = new HashMap<>();
    private static final Map<String, StructArrayPublisher<Translation2d>> translationLists = new HashMap<>();
    private static final Map<String, BooleanPublisher> booleans = new HashMap<>();
    private static final Map<String, DoublePublisher> doubles = new HashMap<>();

    private BLineLogging() {}

    /** Routes all of BLine's logging callbacks to NetworkTables. Safe to call more than once. */
    public static void install() {
        FollowPath.setPoseLoggingConsumer(pair -> poses
                .computeIfAbsent(pair.getFirst(), k -> table.getStructTopic(k, Pose2d.struct).publish())
                .set(pair.getSecond()));
        FollowPath.setTranslationListLoggingConsumer(pair -> translationLists
                .computeIfAbsent(pair.getFirst(), k -> table.getStructArrayTopic(k, Translation2d.struct).publish())
                .set(pair.getSecond()));
        FollowPath.setBooleanLoggingConsumer(pair -> booleans
                .computeIfAbsent(pair.getFirst(), k -> table.getBooleanTopic(k).publish())
                .set(pair.getSecond()));
        FollowPath.setDoubleLoggingConsumer(pair -> doubles
                .computeIfAbsent(pair.getFirst(), k -> table.getDoubleTopic(k).publish())
                .set(pair.getSecond()));
    }
}
