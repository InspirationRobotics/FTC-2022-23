package archive_21;

`import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name="Encoder_WBlue")

public class Encoder_WBlue extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
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

    public double COUNTS_PER_MOTOR_REV;
    public double DRIVE_GEAR_REDUCTION;
    public double WHEEL_DIAMETER_INCH;
    public double COUNTS_PER_INCH;
    public double ROBOT_DIAMETER;
    public double ROBOT_CIRCUMFERENCE;

    double power = 0.5;

    private static final String VUFORIA_KEY =
            "AQFvn1X/////AAABmfD5QbW72U/KuuZnETqfRvxtsghR7zWjrmjy6vu4H4g1VnCMWG5HPq3XhUD1kyMrbjP5lRruLzhYzgHtr9dK4TCjv0K0GTEK8Ww52Y++exIFmBRDTm84bjzBw0CPo0oeSx82fTU+c0S8B5Q4QFYWSCP5FD1pJESKDz3H5WnA0LoEFMkjpjwhMEbYqpVW6PdvO+QqSHBKBQjzKKstLDbEiZI+7A+A++dEaNehTtpy3cL3Jz0Jui6w6Fu3M/Dij9EwVHJTEEOJS4LEQdDV4U3lfkAuVxVxb0SGChr/1qcd+YzKQ/Cv3a5rKmMja54LkCfQ2dgRI3FapdPVZglZ2rr1nc0KTKI5T+Q3lzzhpc1brGZS";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 25.0 / 50.0);
        }
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

        if (opModeIsActive()) {

            /*flipper.setPosition(0.5);
            depositor.setPosition(0.5);
            flipper.setPosition(-.9);*/
            flipper.setPosition(0.5);
            sleep(1000);
            depositor.setPosition(0.5);
            flipper.setPosition(-.9);

            while (colorturret.blue() < 800) {
                turret.setPower(0.4);
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
    }


        int leftPos = 0;
        int rightPos = 0;

        public void encoderStrafeINCH ( double inches, double power){
            int ticks = (int) (inches * COUNTS_PER_INCH);

            leftFront.setTargetPosition(leftFront.getCurrentPosition() - ticks);
            leftBack.setTargetPosition(leftBack.getCurrentPosition() + ticks);
            rightFront.setTargetPosition(rightFront.getCurrentPosition() + ticks);
            rightBack.setTargetPosition(rightBack.getCurrentPosition() - ticks);

            leftFront.setPower(power);
            rightFront.setPower(power);
            leftBack.setPower(power);
            rightBack.setPower(power);

            leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

    public void encoderMoveINCH(double leftINCH, double rightINCH, double power) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int newLeftTarget = leftFront.getCurrentPosition() + (int) (leftINCH * COUNTS_PER_INCH);
        int newRightTarget = rightFront.getCurrentPosition() + (int) (rightINCH * COUNTS_PER_INCH);
        double leftSpeed;
        double rightSpeed;

        leftFront.setTargetPosition(newLeftTarget);
        leftBack.setTargetPosition(newLeftTarget);
        rightFront.setTargetPosition(-newRightTarget);
        rightBack.setTargetPosition(newRightTarget);

        if (Math.abs(leftINCH) > Math.abs(rightINCH)) {
            leftSpeed = power;
            rightSpeed = (power * rightINCH) / leftINCH;
        } else {
            rightSpeed = power;
            leftSpeed = (power * leftINCH) / rightINCH;
        }

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(leftSpeed);
        rightFront.setPower(rightSpeed);
        leftBack.setPower(leftSpeed);
        rightBack.setPower(rightSpeed);


        //continue moving
        while (opModeIsActive() && ((leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()))) {
            idle();
        }

        // Stop all motion;
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

    }



    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public int VuforiaFindDuckPos () {

        int duckPos = 3;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                    telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    telemetry.addData(String.format("  ImageWidth, ImageHeight (%d)", i), "%d , %d",
                            recognition.getImageWidth(), recognition.getImageHeight());
                    telemetry.addData(String.format("  width, height (%d)", i), "%.03f , %.03f",
                            recognition.getWidth(), recognition.getHeight());
                    i++;

                    if(recognition.getLabel() == "Duck"){
                        float duckLeft = recognition.getLeft();
                        int imageWidth = recognition.getImageWidth();
                        int p50 = imageWidth/2;
                        int p100 = imageWidth;

                        if(duckLeft <= p50) {
                            duckPos = 1;
                            telemetry.addData("left", null);
                        }

                        else if (duckLeft >= p50 ) {
                            duckPos = 2;
                            telemetry.addData("middle", null);
                        }

                        else{
                            duckPos = 3;
                            telemetry.addData("right", null);
                        }
                    }

                }
            }
        }
        telemetry.addData("Position", duckPos);
        telemetry.update();

        return duckPos;
    }
}
