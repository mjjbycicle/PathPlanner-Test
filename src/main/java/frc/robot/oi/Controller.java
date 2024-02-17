package frc.robot.oi;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.DEADZONE_VALUE;

@SuppressWarnings("unused")
public class Controller {
    private static final int NUM_BUTTONS = 10;
    private static final int NUM_POV_BUTTONS = 4;

    private final Joystick stick;
    private final Trigger[] buttons;

    public Controller(int joystickID) {
        stick = new Joystick(joystickID);

        buttons = new Trigger[NUM_BUTTONS + NUM_POV_BUTTONS];

        for (int i = 0; i < NUM_BUTTONS; i++) {
            buttons[i] = new JoystickButton(stick, i);
        }

        for (int i = 0; i < NUM_POV_BUTTONS; i++) {
            buttons[i + NUM_BUTTONS] = new POVButton(stick, i * 90);
        }
    }

    /**
     * Creates a dead zone for joysticks; the controllers sticks are a little loose
     * and so there is a margin of error for where they should be considered
     * neutral
     */
    private static double deadzone(double value) {
        // When the axis is LESS than the magic number, the
        // joystick is in the loose position so return zero - as if the
        // joystick was not moved
        if (Math.abs(value) < DEADZONE_VALUE) {
            return 0;
        }

        // When the joystick is MORE than the magic number,
        // scale the value so that the point right after the
        // deadzone is 0 so the robot does not jerk forward
        // when it passes the deadzone.
        return (value / Math.abs(value)) * ((Math.abs(value) - DEADZONE_VALUE) / (1 - DEADZONE_VALUE));
    }

    public double getAxis(OI.Axes axis) {
        double rawAxisValue = stick.getRawAxis(axis.getID());
        double multiplier = axis.shouldInvert() ? -1 : 1;
        return deadzone(multiplier * rawAxisValue);

    }

    public Trigger getButton(OI.Buttons button) {
        return buttons[button.getID()];
    }
}