package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="hi andy")
public class TensorFlowAutoTest extends AutonomousOpMode {

    @Override
    public void startOpMode() {

        telemetry.addData("block pos", BLOCK_POS);
        telemetry.update();
        sleep(10000);

    }
}
