package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.ConceptTensorFlowObjectDetection;
import org.firstinspires.ftc.teamcode.util.Robot;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public abstract class AutonomousOpMode extends LinearOpMode {

    protected Robot r;

    private double DRIVE_KP = 0.001;
    private double DRIVE_KD = 0;

    private final double DRIVE_PID_A = 3;
    private final double DRIVE_PID_B = 0.1;
    private final double TURN_PID_A = 1;
    private final double TURN_PID_B = 0.3;

    private final double WHL_DIAM = 4;
    private final int PPR = 1890;
    private final double PPI = PPR / (WHL_DIAM * Math.PI);

    private final int COMPLETE_THRESHOLD = 50;

    private ElapsedTime runtime = new ElapsedTime();

    private Telemetry dashboardTelemetry;
    public abstract void startOpMode();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";


    private static final String VUFORIA_KEY = "AfAbEIf/////AAAAGcuoYXidf0JOqPaKWgm8WkA1zDYGkoLnlPknk8vq7RH0" +
            "naH/HM6ewoB3oq7pbgiM2sNyKG1wEcWsnmR3NhOXCFpXFylxkILdAua00dD5Cq+6pvZTlsb756HP3XJa3ajT" +
            "/3kjS0ZBaKFT3FaV3ZZCTALJWCRIfkg0YOay9FTneZMgt0r8n7ybZ8lhfN2VqAyqOGQZHOFENunfyA54DKvSZ" +
            "aJn4fHqBAAvE//7p62dfcqtGxm/4JBYoeTwGXcjJO9S3tW9UyttI7Hu3n4HnfCyS9TFu2vo/uMD96hKYtdAu+" +
            "VMlycqdU8ggE5ndbAG7WC9XdLRzRlm3mwou3gFWrtyuZD5eSqS+oGxpo+nSlX7afnk";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    int BLOCK_POS = -2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
        telemetry.addData(">", "please wait");
        r = new Robot(hardwareMap);
        //r.lift.lock();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        r.lift.lock();

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        telemetry.clear();
        telemetry.addData(">", "Now we're ready to go. Vuforia is on. It's showtime.");
        telemetry.update();

        waitForStart();

        BLOCK_POS = sense();
        telemetry.addData("pos", BLOCK_POS);
        telemetry.update();

        startOpMode();

    }

    public void drivePID(double speed, double inches, double timeout, double heading) {
        if (opModeIsActive()) {


            r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            setTargets((int)(inches * PPI));

            double errorAve, integral = 0, previousError;

            double      K_P = AutoConfig.P,
                        K_I = AutoConfig.I,
                        K_D = AutoConfig.D;


            errorAve = (getError(r.DRIVE_LB) + getError(r.DRIVE_RB) +
                    getError(r.DRIVE_LF) + getError(r.DRIVE_RF))/4;

            previousError = errorAve;
            runtime.reset();

            while (opModeIsActive() && !isComplete()) {
                errorAve = (getError(r.DRIVE_LB) + getError(r.DRIVE_RB) +
                        getError(r.DRIVE_LF) + getError(r.DRIVE_RF))/4;
                integral += errorAve;

                double ramp = 0;
                double initialPwr = 0.2;
                if (runtime.seconds() < AutoConfig.T_F && Math.abs(inches) > 3) {
                    double b = speed - initialPwr;
                    ramp = (b * (1 - runtime.seconds() / AutoConfig.T_F)) * (errorAve/Math.abs(errorAve));
                }

                double pwr = Range.clip(K_P * errorAve + K_I * integral + K_D * (errorAve - previousError), -speed, speed) - ramp;

                double offset = AutoConfig.H_P * (yaw() - heading);

                double l_pwr = Range.clip(pwr - offset, -1, 1);
                double r_pwr = Range.clip(pwr + offset, -1, 1);

                r.setDrivePwr(l_pwr, r_pwr);

                dashboardTelemetry.addData("power", pwr);
                dashboardTelemetry.addData("ramp", ramp);
                dashboardTelemetry.addData("error", errorAve);
                dashboardTelemetry.addData("three", 3);
                dashboardTelemetry.update();

                previousError = errorAve;

            }

            r.setDrivePwr(0, 0);
        }
    }

    public void setTargets(int increase) {
        r.DRIVE_LB.setTargetPosition(r.DRIVE_LB.getCurrentPosition() + increase);
        r.DRIVE_LF.setTargetPosition(r.DRIVE_LF.getCurrentPosition() + increase);
        r.DRIVE_RF.setTargetPosition(r.DRIVE_RF.getCurrentPosition() + increase);
        r.DRIVE_RB.setTargetPosition(r.DRIVE_RB.getCurrentPosition() + increase);

    }



    public double yaw() {
        return r.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public int getError(DcMotorEx motor) {
        return motor.getTargetPosition() - motor.getCurrentPosition();
    }

    public boolean isComplete() {
        return  Math.abs(getError(r.DRIVE_LB)) < COMPLETE_THRESHOLD &&
                Math.abs(getError(r.DRIVE_RB)) < COMPLETE_THRESHOLD &&
                Math.abs(getError(r.DRIVE_LF)) < COMPLETE_THRESHOLD &&
                Math.abs(getError(r.DRIVE_RF)) < COMPLETE_THRESHOLD;
    }

    public void turnPID(double tgt_heading, double max_pwr, double timeout) {
        if (opModeIsActive()) {

            r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double error, initialError, integral;

            error = getError(tgt_heading);
            initialError = error;


            runtime.reset();

            while (opModeIsActive() && (Math.abs(error) > 3)) {

                error = getError(tgt_heading);


                double ramp = 0;
                double pwr = Range.clip(AutoConfig.TURN_P * error, -max_pwr, max_pwr) - ramp;
                r.setDrivePwr(pwr, -pwr);

                dashboardTelemetry.addData("power", pwr);
                dashboardTelemetry.addData("ramp", ramp);
                dashboardTelemetry.addData("error", error);
                dashboardTelemetry.addData("heading", yaw());
                dashboardTelemetry.update();


            }

            r.setDrivePwr(0, 0);
        }
    }


    public void turnNoPID(double tgt_heading, double max_pwr, double timeout) {
        if (opModeIsActive()) {

            r.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            double error, initialError, integral;

            error = getError(tgt_heading);
            initialError = error;


            runtime.reset();

            while (opModeIsActive() && (Math.abs(error) > 10)) {

                error = getError(tgt_heading);
                double errorSign = Math.abs(error) / error;



                //double ramp = 0;
                //double pwr = Range.clip(AutoConfig.TURN_P * error, -max_pwr, max_pwr) - ramp;
                r.setDrivePwr(errorSign * max_pwr, -errorSign * max_pwr);

                dashboardTelemetry.addData("power", max_pwr);
                dashboardTelemetry.addData("ramp", 0);
                dashboardTelemetry.addData("error", error);
                dashboardTelemetry.addData("heading", yaw());
                dashboardTelemetry.update();


            }

            r.setDrivePwr(0, 0);
        }
    }

    public double getError(double tgt) {
        double error = tgt - yaw();
        while (error > 180 || error < -180) {
            if (error > 180) error -= 360;
            if (error < -180) error += 360;
        }
        return error;
    }

    public void runTurn() {

    }

    public void unlatch() {
        r.BASKET_PIVOT.setPosition(1);
        r.extender.extendStore();
        r.extender.pivotUp();
        sleep(250);
        r.lift.midNoPivot();
        sleep(250);
        r.lift.setPwr(-1);
        sleep(500);
        r.lift.unlock();
        sleep(200);
        r.lift.setPwr(1);
        sleep(100);
        r.lift.backNoPivot();
        sleep(800);
        r.lift.setPwr(0);
        r.lift.backNoPivot();


    }

    public void liftDown() {
        new Thread(new Runnable() {
            @Override
            public void run() {
                r.lift.setPwr(-0.2);
                sleep(750);
                r.lift.setPwr(0);

            }
        }).start();
    }


    public void moveLift(double pwr, int ticks) {
        new Thread(new LiftMover(ticks, pwr)).start();
    }

    public void liftEncoder(double pwr, int target) {
        r.LIFT_L.setTargetPosition(target);
        r.LIFT_R.setTargetPosition(target);

        r.LIFT_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        r.LIFT_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        r.LIFT_L.setPower(pwr);
        r.LIFT_R.setPower(pwr);

        while (opModeIsActive() && r.LIFT_L.isBusy() && r.LIFT_R.isBusy()) {

            sleep(100);
        }

        r.LIFT_L.setPower(0);
        r.LIFT_R.setPower(0);

        r.LIFT_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.LIFT_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    class LiftMover implements Runnable {

        private int target;
        private double pwr;

        public LiftMover(int target, double pwr) {
            this.target = target;
            this.pwr = pwr;
        }

        @Override
        public void run() {
            liftEncoder(pwr, target);
        }
    }


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }



    private double getX(Recognition r) {
        return (r.getLeft() + r.getRight()) / 2;
    }

    private double getY(Recognition r) {
        return (r.getBottom() + r.getTop()) / 2;
    }

    public int sense() {
        runtime.reset();
        if (tfod != null) {
            tfod.activate();
            while (runtime.seconds() < 3 && opModeIsActive() && !isStopRequested()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getRecognitions();
                    List<Recognition> newStuff = new ArrayList<>();

                    // filter x-values to detect only the right ones
                    // filter out y  positions
                    int goldPos = -2;
                    int goldIndex = -1;
                    if (updatedRecognitions != null) {
                        for (Recognition r : updatedRecognitions) {
                            if (getX(r) < 175) {
                                if (r.getLabel() == LABEL_GOLD_MINERAL) {
                                    newStuff.add(r);
                                    telemetry.addData("Info", "Position: (%s, %s) | Size: %s"
                                            , getX(r), getY(r), r.getHeight() * r.getWidth());

                                }
                            }

                        }

                        if (updatedRecognitions.size() > 0) {
                            telemetry.addData("img w/h", "%s x %s",
                                    tfod.getRecognitions().get(0).getImageWidth(), tfod.getRecognitions().get(0).getImageHeight());
                        }

                    } else {
                        telemetry.addLine("no shapes found");
                    }


                    if (newStuff.size() > 0) {
                        Collections.sort(newStuff, new SortBySize());
                        Recognition bestGuess = newStuff.get(0);
                        if (getY(bestGuess) < 350) {
                            telemetry.addLine("I think it's left");
                            return -1;

                        } else if (getY(bestGuess) < 750) {
                            telemetry.addLine("I think it's center");
                            return 0;

                        } else {
                            telemetry.addLine("I think it's right, lmao");
                            return 1;

                        }
                    } else {
                        telemetry.addLine("No Gold Found :(");
                    }
                }

                telemetry.update();
            }
        }

        return -2;
    }





    class SortBySize implements Comparator<Recognition> {
        public int compare(Recognition a, Recognition b) {
            return (int)Math.round(Math.ceil(b.getHeight() * b.getWidth() - a.getHeight() * a.getWidth()));
        }
    }
}
