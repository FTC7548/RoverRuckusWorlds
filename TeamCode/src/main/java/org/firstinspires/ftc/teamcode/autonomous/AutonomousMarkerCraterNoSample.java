package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Crater Side, Depot, 1-Sample")
public class AutonomousMarkerCraterNoSample extends AutonomousOpMode {

    @Override
    public void startOpMode() {
        unlatch();
        sleep(500);
        drivePID(1, 3, 5, 0);
        //moveLift(0.5, 0);
        r.LIFT_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        r.LIFT_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if (BLOCK_POS == -1) {
            turnPID(45, 0.8, 5);
            drivePID(0.8, 9, 5, 0);
            drivePID(0.8, -9, 5, 0);
        } else if (BLOCK_POS == 0) {
            drivePID(0.8, 7, 5, 0);
            drivePID(0.8, -7, 5, 0);
        } else {
            turnPID(-45, 0.7, 5);
            drivePID(0.8, 9, 5, 0);
            drivePID(0.8, -9, 5, 0);
        }

        r.LIFT_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        r.LIFT_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turnPID(75, 0.8, 0);
        drivePID(1, 17, 5, 0);
        turnPID(135, 0.8, 0);
        drivePID(1, 15, 5, 0);
        r.extender.extendOut();
        sleep(800);
        r.extender.extendStore();
        sleep(500);
        turnNoPID(90, 0.8, 0);
        turnPID(315, 0.8, 5);
        drivePID(1, 16, 4, 0);

        r.extender.setPower(0.8F);
        sleep(500);
        r.extender.setPower(0);
        r.extender.extendOut();






        /*
        r.lift.unlock();
        r.extender.extendIn();
        sleep(100);
        r.lift.setPwr(1);
        sleep(1300);
        r.lift.setPwr(0);
        drivePID(8, 0.4, 3);
        sleep(100);
        r.lift.setPwr(-1);
        r.lift.back();
        sleep(700);
        r.lift.setPwr(0);
        sleep(100);
        r.extender.extendOut();
        r.extender.pivotDown();
        sleep(10000); */
    }
}
