package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BLine.BLineField;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;

/**
 * Autonomous chooser for BLine paths. Every {@code deploy/autos/paths/*.json} file becomes a
 * selectable auto, so paths made in the BLine editor show up without code changes.
 *
 * <p>Also owns a {@link Field2d} ("BLine Field" on SmartDashboard) that shows the robot pose and
 * the selected path, for the sim GUI or AdvantageScope. Call {@link #periodic()} every loop.
 */
public class BLineAutos {
    private static final String PATHS_DIRECTORY = "autos/paths";
    private static final String DEFAULT_AUTO_NAME = "Drive Forward (default)";
    private static final String PATH_OVERLAY_NAME = "Selected Auto";

    private record Entry(Command command, Path path) {}

    private final Map<String, Entry> entries = new LinkedHashMap<>();
    private final SendableChooser<String> chooser = new SendableChooser<>();
    private final Field2d field = new Field2d();
    private final Supplier<Pose2d> robotPose;

    private String drawnAutoName = null;
    private boolean drawnRed = false;
    private String overlayObjectName = null;

    /**
     * @param builder     builder used to turn each path file into a {@link FollowPath} command
     * @param defaultAuto auto selected when the user has not picked one (not a BLine path)
     * @param robotPose   current robot pose, drawn on the field
     */
    public BLineAutos(FollowPath.Builder builder, Command defaultAuto, Supplier<Pose2d> robotPose) {
        this.robotPose = robotPose;
        BLineLogging.install();

        entries.put(DEFAULT_AUTO_NAME, new Entry(defaultAuto, null));
        chooser.setDefaultOption(DEFAULT_AUTO_NAME, DEFAULT_AUTO_NAME);

        for (String name : discoverPathNames()) {
            Path path = new Path(name);
            if (!path.isValid()) {
                DriverStation.reportWarning("BLine path '" + name + "' is not valid; not added to chooser", false);
                continue;
            }
            entries.put(name, new Entry(builder.build(path), path));
            chooser.addOption(name, name);
        }

        SmartDashboard.putData("Auto Chooser", chooser);
        SmartDashboard.putData("BLine Field", field);
    }

    /** Names (without ".json") of every path file in the deploy directory, sorted. */
    public static List<String> discoverPathNames() {
        File[] files = new File(Filesystem.getDeployDirectory(), PATHS_DIRECTORY)
                .listFiles((dir, fileName) -> fileName.endsWith(".json"));
        if (files == null) {
            return List.of();
        }
        return Arrays.stream(files)
                .map(f -> f.getName().substring(0, f.getName().length() - ".json".length()))
                .sorted()
                .toList();
    }

    /**
     * The path as it will actually be driven: {@link FollowPath} flips paths for the red
     * alliance, so the drawn overlay must too. Never modifies {@code path}.
     */
    public static Path pathForAlliance(Path path, boolean red) {
        if (!red) {
            return path;
        }
        Path flipped = path.copy();
        flipped.flip();
        return flipped;
    }

    /** The command for a named auto (a path file name, without ".json"). */
    public Command get(String name) {
        Entry entry = entries.get(name);
        if (entry == null) {
            throw new IllegalArgumentException("No auto named '" + name + "'. Available: " + entries.keySet());
        }
        return entry.command();
    }

    /** The command currently chosen on the dashboard. */
    public Command getSelected() {
        return get(chooser.getSelected());
    }

    /** Updates the robot pose and selected-path overlay on the field. Call every robot loop. */
    public void periodic() {
        field.setRobotPose(robotPose.get());

        String selected = chooser.getSelected();
        boolean red = DriverStation.getAlliance().map(a -> a == Alliance.Red).orElse(false);
        if (selected.equals(drawnAutoName) && red == drawnRed) {
            return;
        }
        drawnAutoName = selected;
        drawnRed = red;

        Path path = entries.get(selected).path();
        if (path != null) {
            overlayObjectName = BLineField.drawPath(field, PATH_OVERLAY_NAME, pathForAlliance(path, red));
        } else if (overlayObjectName != null) {
            field.getObject(overlayObjectName).setPoses(new ArrayList<Pose2d>());
        }
    }

    /** Overlay poses currently on the field; exposed for tests. */
    List<Pose2d> overlayPoses() {
        return overlayObjectName == null ? List.of() : field.getObject(overlayObjectName).getPoses();
    }
}
