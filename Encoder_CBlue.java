package archive_21;

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Encoder_CBlue")
//@Disabled
public class Encoder_CBlue extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    static DcMotor leftFront;
    static DcMotor rightFront;
    static DcMotor leftBack;
    static DcMotor rightBack;
    DcMotor spinner;
    Servo depositor;
    DcMotor  collector;
    DcMotor extension;
    DcMotor  turret;
    Servo    flipper;


    public  double COUNTS_PER_MOTOR_REV;
    public  double DRIVE_GEAR_REDUCTION;
    public  double WHEEL_DIAMETER_INCH;
    public  double COUNTS_PER_INCH;
    public  double ROBOT_DIAMETER;
    public  double ROBOT_CIRCUMFERENCE;

    double power = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        spinner = hardwareMap.dcMotor.get("spinner");
        depositor = hardwareMap.servo.get("depositor");
        turret = hardwareMap.dcMotor.get("turret");
        collector = hardwareMap.dcMotor.get("collector");
        extension = hardwareMap.dcMotor.get("extension");
        flipper = hardwareMap.servo.get("flipper");
        ColorSensor colorturret = hardwareMap.colorSensor.get("colorturret");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        runtime.reset();

        flipper.setPosition(0.5);
        sleep(1500);
        collector.setPower(1);
        sleep(500);
        while (colorturret.blue() < 800) {
            turret.setPower(-0.3);
        }
        turret.setPower(0);
        extension.setPower(1);
        sleep(1500);
        depositor.setPosition(0.5);
        sleep(500);
        flipper.setPosition(0.2);
        sleep(500);
        depositor.setPosition(1);
        sleep(500);

        encoderStrafeINCH(6, 0.5);
        //encoderMoveINCH(17, 17, 0.5);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //spinner starts
        spinner.setPower(0.35);
        sleep(1500);
        spinner.setPower(0.0);

        //spins faster
        spinner.setPower(0.35);
        sleep(2000);
        spinner.setPower(0.0);

        //runs forward
        encoderMoveINCH(-17, -17, 0.5);

        //strafe left
        encoderStrafeINCH(20,   0.2);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    int leftPos = 0;
    int rightPos = 0;

    public void encoderMoveBasic(double leftTarget, double rightTarget, double power){

        leftPos += leftTarget;
        rightPos += rightTarget;

        leftFront.setTargetPosition(leftPos);
        leftBack.setTargetPosition(leftPos);
        rightFront.setTargetPosition(rightPos);
        rightBack.setTargetPosition(rightPos);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

        /*public void encoderStrafe (double distance, double power){

            int ticks = (int)(distance * 301.59);

            leftFront.setTargetPosition(leftFront.getCurrentPosition()-ticks);
            leftBack.setTargetPosition(leftBack.getCurrentPosition()+ticks);
            rightFront.setTargetPosition(rightFront.getCurrentPosition()+ticks);
            rightBack.setTargetPosition(rightBack.getCurrentPosition()-ticks);

            leftFront.setPower(-power);
            rightFront.setPower(power);
            leftBack.setPower(power);
            rightBack.setPower(-power);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }*/

    public void encoderStrafeINCH(double inches, double power){
        int ticks = (int)(inches*COUNTS_PER_INCH);

        leftFront.setTargetPosition(leftFront.getCurrentPosition()-ticks);
        leftBack.setTargetPosition(leftBack.getCurrentPosition()+ticks);
        rightFront.setTargetPosition(rightFront.getCurrentPosition()+ticks);
        rightBack.setTargetPosition(rightBack.getCurrentPosition()-ticks);

        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //continue moving
        while (opModeIsActive() && ((leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()))) {
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }

    public void encoderMoveINCH(double leftINCH, double rightINCH, double power){
        int newLeftTarget = leftFront.getCurrentPosition()+(int)(leftINCH*COUNTS_PER_INCH);
        int newRightTarget = rightFront.getCurrentPosition()+(int)(rightINCH*COUNTS_PER_INCH);
        double leftSpeed;
        double rightSpeed;

        leftFront.setTargetPosition(newLeftTarget);
        leftBack.setTargetPosition(newLeftTarget);
        rightFront.setTargetPosition(newRightTarget);
        rightBack.setTargetPosition(newRightTarget);

        if (Math.abs(leftINCH) > Math.abs(rightINCH)) {
            leftSpeed = power;
            rightSpeed = (power * rightINCH) / leftINCH;
        } else {
            rightSpeed = power;
            leftSpeed = (power * leftINCH) / rightINCH;
        }

        leftFront.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        leftBack.setPower(leftSpeed);
        rightBack.setPower(rightSpeed);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //continue movingg
        while (opModeIsActive() && ((leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()))) {
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }
}


