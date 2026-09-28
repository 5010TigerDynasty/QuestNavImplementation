package frc.robot.subsystems;

public class Utility {
    

    public static double controllerDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband ) {
            return 0.0;
        } else {
            return value;
        }
    }
}
