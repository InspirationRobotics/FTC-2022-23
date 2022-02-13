package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Encoder_WRed")
//@Disabled
public class Encoder_WRed extends LinearOpMode{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    static DcMotor leftFront;
    static DcMotor rightFront;
    static DcMotor leftBack;
    static DcMotor rightBack;
    DcMotor spinner;
    Servo dropper;
    Servo flipperLeft;
    DcMotor extender;

    public double encoder_ticks_per_rotation;    // eg: TETRIX Motor Encoder
    public double gear_ratio;     // 56/24
    public double wheel_circumference;     // For figuring circumference
    public static double encoder_ticks_per_cm;

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
        dropper = hardwareMap.servo.get("dropper");
        extender = hardwareMap.dcMotor.get("extender");
        flipperLeft = hardwareMap.servo.get("flipperLeft");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        encoder_ticks_per_rotation = 1440;    // eg: TETRIX Motor Encoder
        gear_ratio = 30.1;     //NOT SURE IF THIS IS ACCURATE
        wheel_circumference = 301.59;    //NOT SURE IF THIS IS ACCURATE
        encoder_ticks_per_cm = encoder_ticks_per_rotation/(wheel_circumference*gear_ratio);

        waitForStart();
        runtime.reset();

        encoderMoveCM(-900, -900, 0.5);

        extender.setPower(1);
        sleep(1500);
        flipperLeft.setPosition(0);
        sleep(1500);
        //deposit element onto shipping hub
        dropper.setPosition(1);
        sleep(1500);
        //set dropper back into position
        dropper.setPosition(0);
        sleep(1500);

        encoderMoveCM(350, 350, 0.5);

        encoderMoveCM(-370, 370, 0.5);

        //encoderStrafe(1000, 1000, 0.5);

        encoderMoveCM(1300, 1300, 1);
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

    public void encoderMoveCM(double leftCM, double rightCM, double power){
        int newLeftTarget = leftFront.getCurrentPosition()+(int)(leftCM*encoder_ticks_per_cm);
        int newRightTarget = rightFront.getCurrentPosition()+(int)(rightCM*encoder_ticks_per_cm);
        double leftSpeed;
        double rightSpeed;

        leftFront.setTargetPosition(newLeftTarget);
        leftBack.setTargetPosition(newLeftTarget);
        rightFront.setTargetPosition(newRightTarget);
        rightBack.setTargetPosition(newRightTarget);

        if (Math.abs(leftCM) > Math.abs(rightCM)) {
            leftSpeed = power;
            rightSpeed = (power * rightCM) / leftCM;
        } else {
            rightSpeed = power;
            leftSpeed = (power * leftCM) / rightCM;
        }

        leftFront.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        leftBack.setPower(leftSpeed);
        rightBack.setPower(rightSpeed);


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
}



