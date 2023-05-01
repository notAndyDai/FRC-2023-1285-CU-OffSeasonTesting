package frc.robot.util;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    private static OI mInstance;
    public Joystick drivePad;
    public Joystick toolPad;

    // Create a single instance of the OI
    public static OI getInstance() {
        if (mInstance == null)
            mInstance = new OI();
        return mInstance;
    }

    /**
     * Initializes the joystick objects
     */
    public OI() {
        drivePad = new Joystick(GamepadConstants.DRIVE_USB_PORT);
        toolPad = new Joystick(GamepadConstants.TOOL_USB_PORT);
    }

    /**
     * Used to return the drivePad's right joystick y-axis value
     *
     * @return Returns y-value from right joystick on the drivePad
     */
    public double getDriveRightY() {
        double joy = drivePad.getRawAxis(GamepadConstants.RIGHT_ANALOG_Y);
        return Math.abs(joy) < 0.05 ? 0.0 : -joy;
    }

    /**
     * Used to return the toolPad's right joystick y-axis value
     *
     * @return Returns y-value from right joystick on the toolPad
     */
    public double getToolRightY() {
        double joy = toolPad.getRawAxis(GamepadConstants.RIGHT_ANALOG_Y);
        return Math.abs(joy) < 0.05 ? 0.0 : -joy;
    }

    public boolean getToolRightUp() {
        double joy = toolPad.getRawAxis(GamepadConstants.RIGHT_ANALOG_Y);
        return joy < -0.9;
    }

    public boolean getToolRightDown() {
        double joy = toolPad.getRawAxis(GamepadConstants.RIGHT_ANALOG_Y);
        return joy > 0.9;
    }

    /**
     * Used to return the drivePad's left joystick y-axis value
     *
     * @return Returns y-value from left joystick on the drivePad
     */
    public double getDriveLeftY() {
        double joy = drivePad.getRawAxis(GamepadConstants.LEFT_ANALOG_Y);
        return Math.abs(joy) < 0.05 ? 0.0 : -joy;
    }

    /**
     * Used to return the toolPad's left joystick y-axis value
     *
     * @return Returns y-value from left joystick on the toolPad
     */
    public double getToolLeftY() {
        double joy = toolPad.getRawAxis(GamepadConstants.LEFT_ANALOG_Y);
        return Math.abs(joy) < 0.05 ? 0.0 : -joy;
    }

    public boolean getToolLeftUp() {
        double joy = toolPad.getRawAxis(GamepadConstants.LEFT_ANALOG_Y);
        return joy > 0.9;
    }

    public boolean getToolLeftDown() {
        double joy = toolPad.getRawAxis(GamepadConstants.LEFT_ANALOG_Y);
        return joy < -0.9;
    }

    /**
     * Used to return the drivePad's right joystick x-axis value
     *
     * @return Returns x-value from right joystick on the drivePad
     */
    public double getDriveRightX() {
        double joy = drivePad.getRawAxis(GamepadConstants.RIGHT_ANALOG_X);
        return Math.abs(joy) < 0.05 ? 0.0 : joy;
    }

    /**
     * Used to return the toolPad's right joystick x-axis value
     *
     * @return Returns x-value from right joystick on the toolPad
     */
    public double getToolRightX() {
        double joy = toolPad.getRawAxis(GamepadConstants.RIGHT_ANALOG_X);
        return Math.abs(joy) < 0.05 ? 0.0 : joy;
    }

    /**
     * Used to return the drivePad's left joystick x-axis value
     *
     * @return Returns x-value from left joystick on the drivePad
     */
    public double getDriveLeftX() {
        double joy = drivePad.getRawAxis(GamepadConstants.LEFT_ANALOG_X);
        return Math.abs(joy) < 0.05 ? 0.0 : joy;
    }

    /**
     * Used to return the toolPad's left joystick x-axis value
     *
     * @return Returns x-value from left joystick on the toolPad
     */
    public double getToolLeftX() {
        double joy = toolPad.getRawAxis(GamepadConstants.LEFT_ANALOG_X);
        return Math.abs(joy) < 0.05 ? 0.0 : joy;
    }

    // Drive DPAD up
    /**
     * Returns a boolean value representing whether the drive DPadUp button is
     * pressed.
     * 
     * @return A boolean value representing whether the drive DPadUp button is
     *         pressed.
     */
    public boolean getDriveDPadUp() {
        return drivePad.getPOV(0) == GamepadConstants.DPAD_UP;
    }

    // Drive DPAD right
    /**
     * Returns a boolean value representing whether the drive DPadRight button is
     * pressed.
     * 
     * @return A boolean value representing whether the drive DPadRight button is
     *         pressed.
     */
    public boolean getDriveDPadRight() {
        return drivePad.getPOV(0) == GamepadConstants.DPAD_RIGHT;
    }

    // Drive DPAD down
    /**
     * Returns a boolean value representing whether the drive DPadDown button is
     * pressed.
     * 
     * @return A boolean value representing whether the drive DPadDown button is
     *         pressed.
     */
    public boolean getDriveDPadDown() {
        return drivePad.getPOV(0) == GamepadConstants.DPAD_DOWN;
    }

    // Drive DPAD left
    /**
     * Returns a boolean value representing whether the drive DPadLeft button is
     * pressed.
     * 
     * @return A boolean value representing whether the drive DPadLeft button is
     *         pressed.
     */
    public boolean getDriveDPadLeft() {
        return drivePad.getPOV(0) == GamepadConstants.DPAD_LEFT;
    }

    // Tool DPAD up
    /**
     * Returns a boolean value representing whether the tool DPadUp button is
     * pressed.
     * 
     * @return A boolean value representing whether the tool DPadUp button is
     *         pressed.
     */
    public boolean getToolDPadUp() {
        return toolPad.getPOV(0) == GamepadConstants.DPAD_UP;
    }

    // Tool DPAD right
    /**
     * Returns a boolean value representing whether the tool DPadRight button is
     * pressed.
     * 
     * @return A boolean value representing whether the tool DPadRight button is
     *         pressed.
     */
    public boolean getToolDPadRight() {
        return toolPad.getPOV(0) == GamepadConstants.DPAD_RIGHT;
    }

    // Tool DPAD down
    /**
     * Returns a boolean value representing whether the tool DPadDown button is
     * pressed.
     * 
     * @return A boolean value representing whether the tool DPadDown button is
     *         pressed.
     */
    public boolean getToolDPadDown() {
        return toolPad.getPOV(0) == GamepadConstants.DPAD_DOWN;
    }

    // Tool DPAD left
    /**
     * Returns a boolean value representing whether the tool DPadLeft button is
     * pressed.
     * 
     * @return A boolean value representing whether the tool DPadLeft button is
     *         pressed.
     */
    public boolean getToolDPadLeft() {
        return toolPad.getPOV(0) == GamepadConstants.DPAD_LEFT;
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveRightTrigger() {
        return drivePad.getRawButton(GamepadConstants.RIGHT_TRIGGER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolRightTrigger() {
        return toolPad.getRawButton(GamepadConstants.RIGHT_TRIGGER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveLeftTrigger() {
        return drivePad.getRawButton(GamepadConstants.LEFT_TRIGGER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolLeftTrigger() {
        return toolPad.getRawButton(GamepadConstants.LEFT_TRIGGER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveLeftBumper() {
        return drivePad.getRawButton(GamepadConstants.LEFT_BUMPER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolLeftBumper() {
        return toolPad.getRawButton(GamepadConstants.LEFT_BUMPER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveRightBumper() {
        return drivePad.getRawButton(GamepadConstants.RIGHT_BUMPER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolRightBumper() {
        return toolPad.getRawButton(GamepadConstants.RIGHT_BUMPER);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolXButton() {
        return toolPad.getRawButton(GamepadConstants.X_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolAButton() {
        return toolPad.getRawButton(GamepadConstants.A_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolBButton() {
        return toolPad.getRawButton(GamepadConstants.B_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolYButton() {
        return toolPad.getRawButton(GamepadConstants.Y_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveXButton() {
        return drivePad.getRawButton(GamepadConstants.X_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveAButton() {
        return drivePad.getRawButton(GamepadConstants.A_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveBButton() {
        return drivePad.getRawButton(GamepadConstants.B_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveYButton() {
        return drivePad.getRawButton(GamepadConstants.Y_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveBackButton() {
        return toolPad.getRawButton(GamepadConstants.BACK_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolStartButton() {
        return toolPad.getRawButton(GamepadConstants.START_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolBackButton() {
        return toolPad.getRawButton(GamepadConstants.BACK_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolLeftButton() {
        return toolPad.getRawButton(GamepadConstants.LEFT_ANALOG_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getToolRightButton() {
        return toolPad.getRawButton(GamepadConstants.RIGHT_ANALOG_BUTTON);
    }

    /**
     * @return Returns corresponding value (true or false) when button is pressed
     */
    public boolean getDriveStartButton() {
        return drivePad.getRawButton(GamepadConstants.START_BUTTON);
    }
}
