package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



@Autonomous(name = "encoder")
public class EncoderEx extends LinearOpMode {

    //initialization, like setup
    public double encoder_ticks_per_rotation;    // eg: TETRIX Motor Encoder
    public double gear_ratio;     // 56/24
    public double wheel_circumference;     // For figuring circumference
    public double encoder_ticks_per_cm;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    public void runOpMode(){

        //also initialization

        encoder_ticks_per_rotation = 1440;
        gear_ratio = 9.6;
        wheel_circumference = 314* Math.PI;
        encoder_ticks_per_cm = (encoder_ticks_per_rotation) /
                (wheel_circumference * gear_ratio);


        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        //running code

    }

    //where you put functions
    public void encoderDrive(double speed,
                             double leftCM, double rightCM) {
        int newLeftTarget;
        int newRightTarget;
        double leftSpeed;
        double rightSpeed;

        //sets motors to move to set position.
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Sets new position to run to based off encoders
        newLeftTarget = leftFront.getCurrentPosition() + (int) (leftCM * encoder_ticks_per_cm);
        newRightTarget = rightFront.getCurrentPosition() + (int) (rightCM * encoder_ticks_per_cm);
        newLeftTarget = leftBack.getCurrentPosition() + (int) (leftCM * encoder_ticks_per_cm);
        newRightTarget = rightBack.getCurrentPosition() + (int) (rightCM * encoder_ticks_per_cm);
        leftFront.setTargetPosition(newRightTarget);
        rightFront.setTargetPosition(newLeftTarget);
        leftBack.setTargetPosition(newRightTarget);
        rightBack.setTargetPosition(newLeftTarget);


        //starts motion
        if (Math.abs(leftCM) > Math.abs(rightCM)) {
            leftSpeed = speed;
            rightSpeed = (speed * rightCM) / leftCM;
        } else {
            rightSpeed = speed;
            leftSpeed = (speed * leftCM) / rightCM;
        }

        leftFront.setPower(Math.abs(leftSpeed));
        rightFront.setPower(Math.abs(rightSpeed));
        leftBack.setPower(Math.abs(leftSpeed));
        rightBack.setPower(Math.abs(rightSpeed));



        //continue moving
        while (opModeIsActive() &&
                ((leftFront.isBusy() && rightFront.isBusy())))
        {
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
    }
}
