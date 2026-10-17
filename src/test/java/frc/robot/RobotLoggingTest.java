package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class RobotLoggingTest {
    @Test
    void networkTablesValuesEndUpInTheWpilogFile() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        Path dir = Files.createTempDirectory("wpilog-test");

        RobotLogging.start(dir.toString());
        SmartDashboard.putNumber("LoggingTest/value", 42.0);
        Thread.sleep(600); // the log thread writes to disk periodically
        DataLogManager.stop();

        File[] logs = dir.toFile().listFiles((d, name) -> name.endsWith(".wpilog"));
        assertTrue(logs != null && logs.length == 1, "expected exactly one .wpilog in " + dir);

        DataLogReader reader = new DataLogReader(logs[0].getPath());
        assertTrue(reader.isValid());
        List<String> entryNames = new ArrayList<>();
        for (DataLogRecord record : reader) {
            if (record.isStart()) {
                entryNames.add(record.getStartData().name);
            }
        }
        assertTrue(entryNames.contains("NT:/SmartDashboard/LoggingTest/value"), "entries were: " + entryNames);
    }
}
