package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Lift {

    private Robot r;

    public Lift(Robot r) {
        this.r = r;
    }

    public boolean encLiftActive = false;


    // Lift actions

    public void forward() {
        r.BASKET_EXT_L.setPosition(ServoPos.BASKET_EXT_L_FORWARD.pos);
        r.BASKET_EXT_R.setPosition(ServoPos.BASKET_EXT_R_FORWARD.pos);
        r.BASKET_PIVOT.setPosition(ServoPos.BASKET_PIVOT_DUMP.pos);
    }

    public void back() {
        r.BASKET_EXT_L.setPosition(ServoPos.BASKET_EXT_L_BACK.pos);
        r.BASKET_EXT_R.setPosition(ServoPos.BASKET_EXT_R_BACK.pos);
        r.BASKET_PIVOT.setPosition(ServoPos.BASKET_PIVOT_INTAKE.pos);
    }

    public void mid() {
        r.BASKET_EXT_L.setPosition(ServoPos.BASKET_EXT_L_MID.pos);
        r.BASKET_EXT_R.setPosition(ServoPos.BASKET_EXT_R_MID.pos);
        r.BASKET_PIVOT.setPosition(ServoPos.BASKET_PIVOT_INTAKE.pos);
    }

    public void backNoPivot() {
        r.BASKET_EXT_L.setPosition(ServoPos.BASKET_EXT_L_BACK.pos);
        r.BASKET_EXT_R.setPosition(ServoPos.BASKET_EXT_R_BACK.pos);
    }

    public void midNoPivot() {
        r.BASKET_EXT_L.setPosition(ServoPos.BASKET_EXT_L_MID.pos);
        r.BASKET_EXT_R.setPosition(ServoPos.BASKET_EXT_R_MID.pos);
    }

    public void openGate() {
        r.HOLDER_GATE.setPosition(ServoPos.GATE_UP.pos);
    }

    public void closeGate() {
        r.HOLDER_GATE.setPosition(ServoPos.GATE_DOWN.pos);
    }

    public void bucketIntake() {
        r.BASKET_PIVOT.setPosition(ServoPos.BASKET_PIVOT_INTAKE.pos);
    }

    public void bucketDump() {
        r.BASKET_PIVOT.setPosition(ServoPos.BASKET_PIVOT_DUMP.pos);
    }

    public void lock() {
        r.HOOK_L.setPosition(ServoPos.HOOK_L_LOCKED.pos);
        r.HOOK_R.setPosition(ServoPos.HOOK_R_LOCKED.pos);
    }

    public void unlock() {
        r.HOOK_L.setPosition(ServoPos.HOOK_L_UNLOCKED.pos);
        r.HOOK_R.setPosition(ServoPos.HOOK_R_UNLOCKED.pos);
    }

    public void setPwr(double pwr) {
        r.LIFT_L.setPower(pwr);
        r.LIFT_R.setPower(pwr);
    }

    public void encLiftTest(int ticks) {
        int finalTicks = r.LIFT_L.getCurrentPosition() + ticks;
        encLiftActive = true;

        r.LIFT_L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        r.LIFT_R.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (ticks > 0) {
            while (r.LIFT_L.getCurrentPosition() < finalTicks && encLiftActive) {
                if (finalTicks - r.LIFT_L.getCurrentPosition() < 1100) {
                    forward();
                }
                r.lift.setPwr(1);
            }
            r.lift.setPwr(0);
        } else {
            while (r.LIFT_L.getCurrentPosition() > finalTicks && encLiftActive) {
                r.lift.setPwr(-1);
            }
            r.lift.setPwr(0);
        }

    }

    public void stopEncLiftTest() {
        encLiftActive = false;
    }

    private void setLiftMode(DcMotor.RunMode mode) {
        r.LIFT_L.setMode(mode);
        r.LIFT_R.setMode(mode);
    }

    // TODO: Find values for all these servos
    private enum ServoPos {
        BASKET_EXT_L_FORWARD (.93), // towards 0 = down
        BASKET_EXT_L_BACK (0.13), // the one that is not servo 5
        BASKET_EXT_L_MID (0.15),

        BASKET_EXT_R_FORWARD (.95), // towards 0 = down
        BASKET_EXT_R_BACK (0.15), // servo 5
        BASKET_EXT_R_MID (0.15),

        HOOK_L_LOCKED (.99), // 3
        HOOK_L_UNLOCKED (.87), // towards 1 = more locked

        HOOK_R_LOCKED (.01), // 6
        HOOK_R_UNLOCKED (.13), // towards 0 = more locked

        GATE_UP (0),
        GATE_DOWN (0.5),

        BASKET_PIVOT_INTAKE (0.51), // towards 0 = more up from down pos
        BASKET_PIVOT_DUMP (0.33);

        public final double pos;

        ServoPos(double pos) {
            this.pos = pos;
        }
    }

}
