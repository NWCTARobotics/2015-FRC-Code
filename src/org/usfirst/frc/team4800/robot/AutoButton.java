package org.usfirst.frc.team4800.robot;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * A class for tracking when buttons are pressed
 * 
 * @author Michael Jorgensen of Team 4800
 */
public class AutoButton {

    private GenericHID g;
    private int button;
    private DetectionType t;
    private boolean last = false;

    /**
     * Creates a new button. Create a new button object for each button you want
     * to track.
     * 
     * @param g the GenericHID the button is attached to, usually a
     *            {@link edu.wpi.first.wpilibj.Joystick}
     * @param button the button on the GenericHID to track, a value 1 through 12
     * @param t the DetectionType to use. See
     *            {@link com.team4800.buttonapi.Button.DetectionType}
     * @throws IllegalArgumentException thrown if the GeneridHID is null
     * @throws IllegalArgumentException if the DectionType is null
     * @throws IndexOutOfBoundsException if the button is not a value 1 through
     *             12
     */
    public AutoButton(GenericHID g, int button, DetectionType t) {
        if (g == null) {
            throw new IllegalArgumentException("The GenericHID cannot be null");
        }

        if (t == null) {
            throw new IllegalArgumentException("The DetectionType cannot be null");
        }

        if (button < 1 || button > 12) {
            throw new IndexOutOfBoundsException("The button must be a value 1 through 12. Given: " + button);
        }
        this.g = g;
        this.button = button;
        this.t = t;
    }

    /**
     * Detection types for determining when to say the button has been pressed.
     * 
     * @author Michael Jorgensen of Team 4800
     */
    public enum DetectionType {
        /**
         * The method {@link com.team4800.buttonapi.Button#isPressed()} will
         * return true when the button is initially pressed. It will not return
         * true while it is being pressed or released.
         */
        ON_PRESS(),

        /**
         * The method {@link com.team4800.buttonapi.Button#isPressed()} will
         * return true when the button is pressed and then released. It will not
         * return true when the button is first pressed or while it is being
         * pressed.
         */
        ON_RELEASE,

        /**
         * The method {@link com.team4800.buttonapi.Button#isPressed()} will
         * return true when the button is being pressed, false if it is not.
         * <p>
         * Same as {@link edu.wpi.first.wpilibj.Joystick#getRawButton(int)}
         */
        HOLD,

        /**
         * The method {@link com.team4800.buttonapi.Button#isPressed()} will
         * return true when the button is first pressed and when it is released.
         * It will not return true while the button is being pressed.
         */
        ON_PRESS_AND_RELEASE;
    }

    /**
     * Returns true based on the last value of the button, the current value,
     * and what DetectionType was set.
     * 
     * @return true if the DetectionType is satisfied
     */
    public boolean isPressed() {
        switch (t) {
        case ON_PRESS: {
            if (g.getRawButton(button)) {
                if (!last) {
                    last = true;
                    return true;
                } else {
                    return false;
                }
            } else {
                last = false;
                return false;
            }
        }
        case ON_RELEASE: {
            if (!g.getRawButton(button)) {
                if (last) {
                    last = false;
                    return true;
                } else {
                    last = true;
                    return false;
                }
            } else {
                last = true;
                return false;
            }
        }
        case HOLD: {
            return g.getRawButton(button);
        }
        case ON_PRESS_AND_RELEASE: {
            boolean current = g.getRawButton(button);
            if (current != last) {
                last = current;
                return true;
            } else {
                return false;
            }
        }
        default:
            return false;
        }
    }

    /**
     * Gets the current detection type.
     * 
     * @return the detection type currently being used
     */
    public DetectionType getDetectionType() {
        return t;
    }

    /**
     * Sets the detection type to the given type.
     * 
     * @param t the new detection type
     */
    public void setDetectionType(DetectionType t) {
        this.t = t;
    }

    /**
     * Gets the button currently being tracked.
     * 
     * @return the button being tracked
     */
    public int getButton() {
        return button;
    }

    /**
     * Sets the button to track to the given button. <b>Value must be 1 through
     * 12</b>
     * 
     * @param button the new button to be tracked
     * @throws IndexOutOfBoundsException if the button is not a value 1 through
     *             12
     */
    public void setButton(int button) {
        if (button < 1 || button > 12) {
            throw new IndexOutOfBoundsException("The button must be a value 1 through 12. Given: " + button);
        }
        this.button = button;
    }

    /**
     * Gets the GenericHID being tracked
     * 
     * @return the tracked GenericHID
     */
    public GenericHID getGenericHID() {
        return g;
    }
}