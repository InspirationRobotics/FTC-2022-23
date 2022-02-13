package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Encoder_WBlue")
//@Disabled
public class Encoder_WBlue extends LinearOpMode{
        private ElapsedTime runtime = new ElapsedTime();
        static DcMotor leftFront;
        static DcMotor rightFront;
        static DcMotor leftBack;
        static DcMotor rightBack;
        DcMotor spinner;
        Servo dropper;
        Servo flipperLeft;
        DcMotor extender;


       // public double wheel_circumference;     // For figuring circumference
       //public double wheel_diameter;
      public double gear_ratio;     // 56/24
      public static double encoder_ticks_per_inch;
      public double encoder_ticks_per_rotation;// eg: TETRIX Motor Encoder
      public double DRIVE_GEAR_REDUCTION;
      public double WHEEL_DIAMETER_INCHES;
      public double COUNTS_PER_INCH;

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

            /*gear_ratio = 30.16;//NOT SURE IF THIS IS ACCURATE
            wheel_diameter = 18;
            wheel_circumference = wheel_diameter * 3.1415; //NOT SURE IF THIS IS ACCURATE*/
            encoder_ticks_per_inch = (encoder_ticks_per_rotation * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES) * 3.14;
            gear_ratio = 19.2;//NOT SURE IF THIS IS ACCURATE
            WHEEL_DIAMETER_INCHES = 96;
            encoder_ticks_per_rotation = 537.6;// eg: TETRIX Motor Encoder

            waitForStart();
            runtime.reset();

            encoderStrafe(685.8,685.8,1);

            //encoderMoveBasic(1000, 1000, 0.5);
            encoderMoveCM(55.88, 55.88, -0.5);

            encoderMoveCM(15.24, 15.24, 0.5);

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

            leftFront.setPower(-0.25);
            rightFront.setPower(0.25);
            leftBack.setPower(-0.25);
            rightBack.setPower(0.25);

            sleep(1100);

            leftFront.setPower(0.0);
            rightFront.setPower(0.0);
            leftBack.setPower(0.0);
            rightBack.setPower(0.0);

            encoderMoveCM(127, 127, 0.5);

            //strafe to right
            leftFront.setPower(-0.5);
            rightFront.setPower(0.5);
            leftBack.setPower(0.5);
            rightBack.setPower(-0.5);

            sleep(1200);

            leftFront.setPower(0.0);
            rightFront.setPower(0.0);
            leftBack.setPower(0.0);
            rightBack.setPower(0.0);

            //spinner starts
            spinner.setPower(0.35);
            sleep(1500);
            spinner.setPower(0.0);

            //spins faster
            spinner.setPower(0.35);
            sleep(2000);
            spinner.setPower(0.0);

            //strafe to right
            leftFront.setPower(0.25);
            rightFront.setPower(-0.25);
            leftBack.setPower(-0.25);
            rightBack.setPower(0.25);

            sleep(1000);

            leftFront.setPower(0.0);
            rightFront.setPower(0.0);
            leftBack.setPower(0.0);
            rightBack.setPower(0.0);

            encoderMoveCM(7.62, 7.62, 0.5);

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

        public void encoderStrafe (double leftCM, double rightCM, double power){

            int newLeftTarget = leftFront.getCurrentPosition()+(int)(leftCM*encoder_ticks_per_inch);
            int newRightTarget = rightFront.getCurrentPosition()+(int)(rightCM*encoder_ticks_per_inch);
            double leftSpeed;
            double rightSpeed;


            leftFront.setPower(-power);
            leftBack.setPower(power);
            rightFront.setPower(power);
            rightBack.setPower(-power);


            leftFront.setTargetPosition(leftPos);
            leftBack.setTargetPosition(leftPos);
            rightFront.setTargetPosition(rightPos);
            rightBack.setTargetPosition(rightPos);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
        public void encoderMoveCM(double leftCM, double rightCM, double power){
           int newLeftTarget = leftFront.getCurrentPosition()+(int)(leftCM*encoder_ticks_per_inch);
           int newRightTarget = rightFront.getCurrentPosition()+(int)(rightCM*encoder_ticks_per_inch);
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



