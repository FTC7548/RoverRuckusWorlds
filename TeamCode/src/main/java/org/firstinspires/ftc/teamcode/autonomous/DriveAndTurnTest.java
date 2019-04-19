package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="drive & turn test")
public class DriveAndTurnTest extends AutonomousOpMode {

    @Override
    public void startOpMode() {
        drivePID(0.7, DriveAndTurnTestConfig.DRIVE_LENGTH, 5, 0);
        sleep(500);
        turnPID(DriveAndTurnTestConfig.TURN_HEADING, 0.7, 5);
    }



}
