package frc.robot.loops;

import frc.robot.subsystems.DriveTrain;
import frc.robot.util.NumberConstants;
import frc.robot.util.OI;

public class DriveLoop extends DriveTrain {

    private static DriveLoop mInstance;
    private DriveStates driveState;
    private OI oi;

    public static DriveLoop getInstance() { // if there is no instance of Drive Loop class it will make one
        if (mInstance == null)
            mInstance = new DriveLoop();
        return mInstance;
    }

    public enum DriveStates { // variables assigned for DriveStates
        PATH_FOLLOWING,
        TELEOP,
        DISABLED
    }

    private DriveLoop() { // set DriveLoop as "DISABLED"
        driveState = DriveStates.DISABLED;
        oi = OI.getInstance();
    }

    @Override
    public void periodic() { // Check what driveState the robot is in(Every ime the scheduel is run)
        odometry.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());

        switch (driveState) {
            case PATH_FOLLOWING:
                break;
            case TELEOP:
                tankDrive();
                break;
            case DISABLED:
                super.stopPower();
                break;
            default:
                super.stopPower();
                break;

        }
    }// periodic

    private void tankDrive() {
        double left = oi.getDriveLeftY();
        double right = oi.getDriveRightY();

        if (oi.getDriveDPadUp()) {
            super.setPower(0.2, 0.2);
        } else if (oi.getDriveDPadDown()) {
            super.setPower(-0.2, -0.2);
        } else {
            if (oi.getDriveLeftBumper()) {
                super.setPower(left * NumberConstants.DRIVE_BOOST_POWER, right * NumberConstants.DRIVE_BOOST_POWER);
            } else {
                super.setPower(Math.signum(left) * left * left * NumberConstants.DRIVE_TELEOP_MAX_POWER,
                        Math.signum(right) * right * right * NumberConstants.DRIVE_TELEOP_MAX_POWER);
            }
        }
    }

    public void setState(DriveStates state) {
        driveState = state;
    }

    public DriveStates getState() {
        return driveState;
    }
}
