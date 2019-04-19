package org.firstinspires.ftc.teamcode.util;

public abstract class Macro {

    public Thread thread;
    private Runnable[] r;

    public Macro(Runnable[] r) {
        thread = new Thread();
        this.r = r;
    }

    public void update() {
        if (condition() != -1) {
            if (!thread.isAlive()) {
                thread = new Thread(r[condition()]);
                thread.start();
            }
        }
    }

    public abstract int condition();
}
