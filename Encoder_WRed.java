package archive_21;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Encoder_WRed")
//@Disabled
public class Encoder_WRed extends LinearOpMode{
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

        COUNTS_PER_MOTOR_REV = 537.6;
        DRIVE_GEAR_REDUCTION = 19.2/1;     // This is < 1.0 if geared UP (32 teeth to 16 teeth)
        WHEEL_DIAMETER_INCH = 96;     // For figuring circumference
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH* 3.1415);
        ROBOT_CIRCUMFERENCE = ROBOT_DIAMETER * 3.1415;

        waitForStart();
        runtime.reset();

        waitForStart();
        if (opModeIsActive()) {
/*
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                    }
                    telemetry.update();

                }
            }

            /*telemetry.addData("Blue", colorturret.blue());
            telemetry.addData("Red", colorturret.red());
            telemetry.update();*/
collector.setPower(0.5);
sleep(1000);
            flipper.setPosition(0.5);
            sleep(1000);
            depositor.setPosition(0.5);
            flipper.setPosition(-.9);
collector.setPower(0);
        }

            while (colorturret.blue() < 800) {
                turret.setPower(-0.2);
            }
            turret.setPower(0);

        extension.setPower(1.0);
        sleep(1500);
        extension.setPower(0);
        depositor.setPosition(0.5);
        sleep(1000);
        flipper.setPosition(0.2);
        sleep(1000);
        depositor.setPosition(1);
        sleep(1000);
        flipper.setPosition(0.5);
        sleep(1000);
        extension.setPower(-1);
        sleep(1000);
        extension.setPower(0);

        while (colorturret.red() < 1000) {
                turret.setPower(0.2);
            }
            turret.setPower(0);

        extension.setPower(1.0);
        sleep(1500);
        extension.setPower(0);
        depositor.setPosition(0.5);
        sleep(1000);
        flipper.setPosition(0.2);
        sleep(1000);
        depositor.setPosition(1);
        sleep(1000);
        flipper.setPosition(0.5);
        sleep(1000);
        extension.setPower(-1);
        sleep(1000);        telemetry.addData("Status", "Initialized");
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
        colorturret = hardwareMap.colorSensor.get("colorturret");



        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        COUNTS_PER_MOTOR_REV = 537.6;
        DRIVE_GEAR_REDUCTION = 19.2/1;     // This is < 1.0 if geared UP (32 teeth to 16 teeth)
        WHEEL_DIAMETER_INCH = 96;     // For figuring circumference
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH* 3.1415);
        ROBOT_CIRCUMFERENCE = ROBOT_DIAMETER * 3.1415;

        waitForStart();
        runtime.reset();

        waitForStart();
        if (opModeIsActive()) {
/*
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                    }
                    telemetry.update();

                }
            }

            /*telemetry.addData("Blue", colorturret.blue());
            telemetry.addData("Red", colorturret.red());
            telemetry.update();*/
            flipper.setPosition(0.5);
            sleep(1000);
            depositor.setPosition(0.5);
            flipper.setPosition(-.9);

        }

        while (colorturret.blue() < 800) {
            turret.setPower(-0.2);
        }
        turret.setPower(0);

        extension.setPower(1.0);
        sleep(1500);
        extension.setPower(0);
        depositor.setPosition(0.5);
        sleep(1000);
        flipper.setPosition(0.2);
        sleep(1000);
        depositor.setPosition(1);
        sleep(1000);
        flipper.setPosition(0.5);
        sleep(1000);
        extension.setPower(-1);
        sleep(1000);
        extension.setPower(0);

        while (colorturret.red() < 1000) {
            turret.setPower(0.2);
        }
        turret.setPower(0);

        extension.setPower(1.0);
        sleep(1500);
        extension.setPower(0);
        depositor.setPosition(0.5);
        sleep(1000);
        flipper.setPosition(0.2);
        sleep(1000);
        depositor.setPosition(1);
        sleep(1000);
        flipper.setPosition(0.5);
        sleep(1000);
        extension.setPower(-1);
        sleep(1000);
        extension.setPower(0);

        while (colorturret.red() < 1000) {
            turret.setPower(-0.2);
        }
        turret.setPower(0);

        depositor.setPosition(0.85);
        collector.setPower(1);
        sleep(1000);
        flipper.setPosition(0.95);
        collector.setPower(0);

        encoderMoveINCH(30,30,1);

    }
/*
        //strafe left
        encoderStrafeINCH(-34, 0.5);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //runs backwards
        encoderMoveINCH(-28, -28, 0.5);

        depositor.setPosition(0.5);
        sleep(500);
        //dropper starts and ends
        extension.setPower(-1);
        sleep(1500);
        flipper.setPosition(0);
        sleep(1500);

        //deposit element onto shipping hub
        depositor.setPosition(1);
        sleep(1500);

        //set dropper back into position
        depositor.setPosition(0);
        sleep(1500);

        //runs forward
        encoderMoveINCH(20, 20, 0.4);

        //turn
        encoderMoveINCH(-25, 25, 0.4);

        //strafe left
        encoderStrafeINCH(-25, 0.4);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //runs forward
        encoderMoveINCH(70, 70, 0.3);*/

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


        //continue movinggg
        while (opModeIsActive() && ((leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()))) {
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }
}

