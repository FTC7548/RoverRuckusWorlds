package org.firstinspires.ftc.teamcode.util;

/**
 * Created by dchotzen-hartzell19 on 10/31/17.
 */

public abstract class ToggleServo {

    public boolean alreadyPressed;
    public boolean state = false;

    public abstract void toggleTrue();
    public abstract void toggleFalse();
    public abstract boolean toggleCondition();

    public void update() {
        if (toggleCondition()) {
            if (!alreadyPressed) {
                execute();
                alreadyPressed = true;
            }
        } else
            alreadyPressed = false;
    }

    public void execute() {
        if (state) {
            toggleFalse();
        } else {
            toggleTrue();
        }
        state = !state;
    }
}
