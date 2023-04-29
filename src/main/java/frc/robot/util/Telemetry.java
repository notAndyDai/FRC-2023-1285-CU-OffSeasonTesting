package frc.robot.util;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.loops.DriveLoop;
import frc.robot.util.Telemetry;

public class Telemetry {

    private static DriveLoop driveLoop;

    public static void init() {
        driveLoop = DriveLoop.getInstance();
    }

    public static void update() {
    }

    public static void updateDashBoard() {
        SmartDashboard.putString("Drive Loop State", String.valueOf(driveLoop.getState()));
    }

}
