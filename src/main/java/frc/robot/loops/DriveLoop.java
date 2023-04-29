package frc.robot.loops;

import frc.robot.subsystems.DriveTrain;

public class DriveLoop extends DriveTrain {

    private static DriveLoop mInstance;
    private DriveStates driveState;

    public static DriveLoop getInstance() { // if there is no instance of Drive Loop class it will make one
        if (mInstance == null)
            mInstance = new DriveLoop();
        return mInstance;
    }

    public enum DriveStates { // variables assigned for DriveStates
        PATH_FOLLOWING,
        DISABLED
    }

    private DriveLoop() { // set DriveLoop as "DISABLED"
        driveState = DriveStates.DISABLED;
    }

    @Override
    public void periodic() { // Check what driveState the robot is in(Every ime the scheduel is run)
        odometry.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());

        switch (driveState) {
            case PATH_FOLLOWING:
                break;
            case DISABLED:
                super.stopPower();
                break;
            default:
                super.stopPower();
                break;

        }
    }// periodic

    public void setState(DriveStates state) {
        driveState = state;
    }

    public DriveStates getState() {
        return driveState;
    }
}
