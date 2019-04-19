package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="depot side")
public class AutonomousDepotSide extends AutonomousOpMode {

    @Override
    public void startOpMode() {
        unlatch();
        sleep(500);
        drivePID(1, 2.5, 5, 0);

        if (BLOCK_POS == -1) {
            turnPID(45, 0.8, 5);
            drivePID(0.8, 13, 5, 0);
            turnPID(-45, 0.8, 5);
            drivePID(0.8, 8, 5, 0);


        } else if (BLOCK_POS == 0) {
            drivePID(0.8, 16, 5, 0);

        } else {
            turnPID(-45, 0.9, 5);
            drivePID(1, 10, 5, 0);
            drivePID(1, -10, 5, 0);
            turnPID(75, 0.8, 3);
            drivePID(1, 16, 4, 0);
            turnPID(315, 0.8, 5);
            //drivePID(0.7, 5, 4, 0);
            //drivePID(0.8, 15, 4, 0);
            new Thread(new Runnable() {
                @Override
                public void run() {
                    r.extender.setPower(1);
                    sleep(750);
                    r.extender.setPower(0);
                }
            }).start();
            drivePID(1, 6, 5, 0);
            sleep(200);
        }

        r.extender.extendOut();
        sleep(800);
        r.extender.extendStore();

        if (BLOCK_POS == -1) {
            drivePID(1, -10, 4, 0);
            turnNoPID(90, 0.8, 0);
            turnPID(135, 0.8, 5);
            liftDown();
            drivePID(1, 10, 4, 0);

        } else if (BLOCK_POS == 0) {
            drivePID(0.8, -16, 5, 0);
            turnPID(75, 0.8, 3);
            drivePID(1, 15, 4, 0);
            turnPID(135, 0.8, 5);
            drivePID(0.7, 5, 4, 0);

        } else {
            new Thread(new Runnable() {
                @Override
                public void run() {
                    r.extender.setPower(-1);
                    sleep(750);
                    r.extender.setPower(0);
                }
            }).start();
            drivePID(1, -4, 5, 0);
            sleep(400);
            turnNoPID(0, 0.9, 0);
            turnPID(135, 0.9, 5);
            new Thread(new Runnable() {
                @Override
                public void run() {
                    r.extender.setPower(1);
                    sleep(650);
                    r.extender.setPower(0);
                }
            }).start();
            drivePID(1, 6, 3, 0);
            sleep(100);
            return;


        }

        r.extender.setPower(1);
        sleep(650);
        r.extender.setPower(0);



    }
}
