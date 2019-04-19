package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Macro;
import org.firstinspires.ftc.teamcode.util.Robot;
import org.firstinspires.ftc.teamcode.util.ToggleServo;

@TeleOp(name="TeleOP")
public class TeleOP extends OpMode {

    private Robot r;
    private ToggleServo     bucketArm,
                            bucketPivot,
                            bucketGate,
                            liftLock,
                            intakePivot,
                            intakeBasketPivot,
                            intakeBasketGate,
                            intakeToggle;

    private Macro macroMgr;

    public boolean intakeToggleOn = false;

    public void init() {
        r = new Robot(hardwareMap);

        r.lift.unlock();
        r.lift.back();

        bucketArm = new ToggleServo() {

            @Override
            public void toggleTrue() {
                r.lift.forward();
                r.extender.extendOut();
            }

            @Override
            public void toggleFalse() {
                r.lift.back();
            }

            @Override
            public boolean toggleCondition() {
                return gamepad1.a;
            }
        };


        liftLock = new ToggleServo() {
            @Override
            public void toggleTrue() {
                r.lift.lock();
            }

            @Override
            public void toggleFalse() {
                r.lift.unlock();
            }

            @Override
            public boolean toggleCondition() {
                return gamepad1.b;
            }
        };

        intakePivot = new ToggleServo() {
            @Override
            public void toggleTrue() {
                r.extender.extendOut();
                ElapsedTime time = new ElapsedTime();
                while (time.seconds() < 0.25) {

                }
                r.extender.pivotDown();
                //r.extender.intake(1);
            }

            @Override
            public void toggleFalse() {
                r.INTAKE_PIVOT.setPosition(0.9);
                r.extender.extendIn();
                /*
                r.extender.extendIn();
                ElapsedTime time = new ElapsedTime();
                while (time.seconds() < 0.25) {

                }
                r.extender.pivotUp();
                */
                /*
                if (!(gamepad1.right_trigger > 0.4)) {
                    r.extender.intake(-1);
                } else {
                    r.extender.intake(1);
                }
                */

            }

            @Override
            public boolean toggleCondition() {
                return gamepad2.y;
            }
        };

        intakeBasketPivot = new ToggleServo() {
            @Override
            public void toggleTrue() {
                r.extender.pivotDown();
            }

            @Override
            public void toggleFalse() {
                r.extender.pivotUp();
            }

            @Override
            public boolean toggleCondition() {
                return gamepad2.x;
            }
        };

        intakeToggle = new ToggleServo() {
            @Override
            public void toggleTrue() {
                intakeToggleOn = true;
            }

            @Override
            public void toggleFalse() {
                intakeToggleOn = false;
            }

            @Override
            public boolean toggleCondition() {
                return gamepad2.right_bumper;
            }
        };

        Runnable[] runnables = {
                new Runnable() {
                    @Override
                    public void run() {
                        intakePivot.toggleTrue();
                        intakePivot.state = true;
                        r.lift.encLiftTest(2100);
                    }

                },
                new Runnable() {
                    @Override
                    public void run() {
                        bucketArm.toggleFalse();
                        ElapsedTime time = new ElapsedTime();
                        while (time.seconds() < 0.25) {

                        }
                        r.lift.encLiftTest(-2100);
                    }
                }};
        macroMgr = new Macro(runnables)
        {
            @Override
            public int condition() {
                if (gamepad1.dpad_up) return 0;
                if (gamepad1.dpad_down) return 1;
                else return -1;
            }
        };

    }

    public void start() {

    }

    public void loop() {
        macroMgr.update();
        bucketArm.update();
        //bucketGate.update();
        liftLock.update();
        intakePivot.update();
        intakeBasketPivot.update();
        //intakeBasketGate.update();
        intakeToggle.update();

        if (gamepad1.right_bumper) {
            r.setDrivePwr(gamepad1.left_stick_y * 0.5, gamepad1.right_stick_y * 0.5);
        } else {
            r.setDrivePwr(gamepad1.left_stick_y, gamepad1.right_stick_y);
        }

        if (gamepad2.dpad_up) {
            r.extender.setPower(1F);

        } else if (gamepad2.dpad_down) {
            r.extender.setPower(-1F);
        } else {
            r.extender.setPower(0F);
        }

        if (gamepad1.left_trigger > 0.4) {
            r.lift.setPwr(-1);
        } else if (gamepad1.left_bumper) {
            if (gamepad1.right_bumper) {
                r.lift.setPwr(0.5);
            } else {
                r.lift.setPwr(1);
            }
        } else {
            if (!macroMgr.thread.isAlive()) {
                r.lift.setPwr(0);
            }
        }

        if (gamepad2.right_trigger > 0.4) {
            r.extender.intake(1);
        } else if (intakeToggleOn) {
            r.extender.intake(-1);
        } else {
            r.extender.intake(0);
        }

        /*
        if (gamepad2.right_trigger > 0.4) {
            r.extender.intake(1);
        } else if (gamepad2.right_bumper) {
            r.extender.intake(-1);
        } else {
            r.extender.intake(0);
        }
        */

        if (gamepad1.right_trigger > 0.4) {
            r.lift.openGate();
        } else {
            r.lift.closeGate();
        }

        if (gamepad2.dpad_right) {
            r.extender.gateDown();
            r.INTAKE_PIVOT.setPosition(0.9);
            r.extender.extendIn();
            intakePivot.state = false;

        } else {
            r.extender.gateUp();
        }

        if (gamepad1.dpad_left) {
            r.lift.stopEncLiftTest();
        }

        telemetry.addData("servo #1 position", r.INTAKE_EXT_R.getPosition());
        telemetry.addData("lift_pos", r.LIFT_L.getCurrentPosition());
        telemetry.update();

    }

    @Override
    public void stop() {
        r.lift.stopEncLiftTest();
        r.lift.setPwr(0);
    }

}
