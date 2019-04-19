package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Robot is the wrapper class to keep track of all of the hardware components in the same place.
 * It must be instantiated with an instance of HardwareMap to reference. Each OpMode creates an
 * instance of Robot to reference.
 *
 * @author chotzen
 * @version 0
 */
public class Robot {

    /**
     * Drive motors
     */
    public DcMotorEx DRIVE_LF,
                        DRIVE_LB,
                        DRIVE_RF,
                        DRIVE_RB;

    /**
     * Lift motors
     */
    public DcMotor LIFT_L,
                        LIFT_R;
    /**
     * Extension motors
     */
    public DcMotor EXTEND_L,
                        EXTEND_R;

    public CRServo INTAKE_1,
                        INTAKE_2;

    /**
     * Lift Servos
     */

    public Servo HOOK_L,
                        HOOK_R,
                        BASKET_EXT_L,
                        BASKET_EXT_R,
                        HOLDER_GATE,
                        BASKET_PIVOT;

    /**
     * Extending Servos
     */

    public Servo INTAKE_EXT_L,
                        INTAKE_EXT_R,
                        INTAKE_GATE,
                        INTAKE_PIVOT;

    /**
     * Util Wrapper Classes
     */

    public Lift         lift;
    public Extender     extender;

    public IntegratingGyroscope gyro;
    public NavxMicroNavigationSensor navxMicro;

    /**
     * Internal gyro in the REV module
     */
    public BNO055IMU IMU;

    public HardwareMap hm;

    /**
     * Constructor to initialize all class fields
     * @param hm    Instance of HardwareMap from OpMode
     */
    public Robot(HardwareMap hm) {
        this.hm = hm;

        DRIVE_LF = (DcMotorEx)hm.dcMotor.get("drive_lf");
        DRIVE_RF = (DcMotorEx)hm.dcMotor.get("drive_rf");
        DRIVE_LB = (DcMotorEx)hm.dcMotor.get("drive_lb");
        DRIVE_RB = (DcMotorEx)hm.dcMotor.get("drive_rb");
        DRIVE_LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DRIVE_LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DRIVE_RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DRIVE_RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DRIVE_RF.setDirection(DcMotor.Direction.REVERSE);
        DRIVE_RB.setDirection(DcMotor.Direction.REVERSE);

        LIFT_L = hm.dcMotor.get("lift_l");
        LIFT_R = hm.dcMotor.get("lift_r");

        LIFT_L.setDirection(DcMotor.Direction.REVERSE);

        LIFT_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LIFT_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //LIFT_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //LIFT_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        HOOK_L = hm.servo.get("hook_l"); // 3
        HOOK_R = hm.servo.get("hook_r"); // 6

        BASKET_EXT_L = hm.servo.get("basket_ext_l"); // 2
        BASKET_EXT_R = hm.servo.get("basket_ext_r"); // 5
        HOLDER_GATE = hm.servo.get("holder_gate"); // 4
        BASKET_PIVOT = hm.servo.get("basket_pivot"); // 1

        EXTEND_L = hm.dcMotor.get("extend_l");
        EXTEND_R = hm.dcMotor.get("extend_r");

        EXTEND_L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        EXTEND_R.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        INTAKE_1 = hm.crservo.get("intake_1"); // 2
        INTAKE_2 = hm.crservo.get("intake_2"); // 3

        INTAKE_EXT_L = hm.servo.get("intake_ext_l"); // 6
        INTAKE_EXT_R = hm.servo.get("intake_ext_r"); // 1
        INTAKE_GATE = hm.servo.get("intake_gate"); // 5
        INTAKE_PIVOT = hm.servo.get("intake_pivot"); // 4

        lift = new Lift(this);
        extender = new Extender(this);

        //navxMicro = hm.get(NavxMicroNavigationSensor.class, "navx");
        //gyro = (IntegratingGyroscope)navxMicro;



        gyroInit();

    }

    public void gyroInit() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        IMU = hm.get(BNO055IMU.class, "imu");
        IMU.initialize(parameters);
    }

    /**
     * Sets the power of the drive motors
     * @param l_pwr Power of left drive motors
     * @param r_pwr Power of the right drive motors
     */
    public void setDrivePwr(double l_pwr, double r_pwr) {
        DRIVE_LF.setPower(l_pwr);
        DRIVE_LB.setPower(l_pwr);
        DRIVE_RF.setPower(r_pwr);
        DRIVE_RB.setPower(r_pwr);
    }

    /**
     * Sets the motor mode for the four drive motors
     * @param mode Mode for drive motors
     */
    public void setMode(DcMotor.RunMode mode) {
        DRIVE_LF.setMode(mode);
        DRIVE_LB.setMode(mode);
        DRIVE_RF.setMode(mode);
        DRIVE_RB.setMode(mode);
    }

    /*
    public enum ServoPosition {
        PREVUP_UP (1),
        PREVUP_DOWN (.88),
        PREVDOWN_UP (0),
        PREVDOWN_DOWN (.12);

        public final double position;

        RatchetPosition(double position) {
            this.position = position;
        }
    } */

}
