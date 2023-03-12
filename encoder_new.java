package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;



@Autonomous(name = "encoder_new")
public class encoder_new extends LinearOpMode {

    public OpenCvInternalCamera phoneCam;
    public PowerPlayDeterminationPipeline pipeline;


    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backRight = null;
    public DcMotor backLeft = null;
    //ColorSensor color_sensor;

    public double pink_area;
    public double green_area;
    public double purple_area;

    public static final double SERVO_POSITION = 0.7;
    public static final double SERVO_RETRACTED_POSITION = 1.0;

    /* Declare OpMode members. */
    // private ElapsedTime run  = new ElapsedTime();
    Robot robot = new Robot(); // use the class created to define a Pushbot's hardware
    double clawOffset = 0.0;                  // Servo mid position
    final double CLAW_SPEED = 0.02;                 // sets rate to move servo
    double TPR = 384.5;
    double TP360 = TPR * 8;

    public static final double STRAFE_SPEED = 0.5;
    public static final String VUFORIA_KEY = "AQFvn1X/////AAABmfD5QbW72U/KuuZnETqfRvxtsghR7zWjrmjy6vu4H4g1VnCMWG5HPq3XhUD1kyMrbjP5lRruLzhYzgHtr9dK4TCjv0K0GTEK8Ww52Y++exIFmBRDTm84bjzBw0CPo0oeSx82fTU+c0S8B5Q4QFYWSCP5FD1pJESKDz3H5WnA0LoEFMkjpjwhMEbYqpVW6PdvO+QqSHBKBQjzKKstLDbEiZI+7A+A++dEaNehTtpy3cL3Jz0Jui6w6Fu3M/Dij9EwVHJTEEOJS4LEQdDV4U3lfkAuVxVxb0SGChr/1qcd+YzKQ/Cv3a5rKmMja54LkCfQ2dgRI3FapdPVZglZ2rr1nc0KTKI5T+Q3lzzhpc1brGZS";
    VuforiaLocalizer vuforia = null;

    static DcMotor lift;
    public Servo flipper;
    public Servo clawRight;
    public Servo clawLeft;

    public double COUNTS_PER_MOTOR_REV;
    public double DRIVE_GEAR_REDUCTION;
    public double WHEEL_DIAMETER_INCH;
    public double COUNTS_PER_INCH;
    public double ROBOT_DIAMETER;
    public double ROBOT_CIRCUMFERENCE;

    double power = 0.5;


    public void cvinit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        //color_sensor = hardwareMap.colorSensor.get("color");


        telemetry.addData("Say", "Hello Driver");
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.useExtendedTracking = false;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        this.vuforia = ClassFactory.getInstance().createVuforia(parameters);
//        // Load the trackable objects from the Assets file, and give them meaningful names
//        VuforiaTrackables targetsPowerPlay = this.vuforia.loadTrackablesFromAsset("PowerPlay");
//        targetsPowerPlay.get(0).setName("Red Audience Wall");
//        targetsPowerPlay.get(1).setName("Red Rear Wall");
//        targetsPowerPlay.get(2).setName("Blue Audience Wall");
//        targetsPowerPlay.get(3).setName("Blue Rear Wall");
//
//        // Start tracking targets in the background
//        targetsPowerPlay.activate();

        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new PowerPlayDeterminationPipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.MAXIMIZE_EFFICIENCY);
        phoneCam.pauseViewport();

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                //phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addData("Camera OnError", 0);

            }
        });


    }

    public void closeCamera() {
        phoneCam.stopStreaming();
        phoneCam.closeCameraDevice();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */


    /* computer vision start ------------------------------------------------ */
    public class PowerPlayDeterminationPipeline extends OpenCvPipeline {
        private boolean showContours = true;
        int ringnum = 0;
        int frameNumber = 0;

        /* bounding rect and contours */
        private List<MatOfPoint> contours = new ArrayList<>();
        Rect bounding_rect_orange_global = new Rect();
        private List<MatOfPoint> contours_orange = new ArrayList<>();
        private List<MatOfPoint> contours_green = new ArrayList<>();
        private List<MatOfPoint> contours_pink = new ArrayList<>();
        private List<MatOfPoint> contours_purple = new ArrayList<>();

        private Rect roi = new Rect(109, 0, 234, 198);

        public synchronized void setShowCountours(boolean enabled) {
            showContours = enabled;
        }

        public synchronized List<MatOfPoint> getContours() {
            return contours;
        }

        double largest_area;

        public Mat processFrame(Mat rgba) {

            Size size = new Size(352, 198);
            frameNumber++;
            telemetry.addData("GotFrame ", frameNumber);
            telemetry.addData("Say", "Got Frame");

            Imgproc.resize(rgba, rgba, size);
            rgba = new Mat(rgba.clone(), roi);

            /* bounding boxes */
            Rect bounding_rect_orange = new Rect();

            /* matricies: hsv, thresholded, and rgba/thresholded cropped */
            Mat hsv = new Mat();
            Mat grey = new Mat();
            Mat thresholded_green = new Mat();
            Mat thresholded_purple = new Mat();
            Mat thresholded_pink = new Mat();

            /* change colorspace */
            Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);

            /* threshold */
            // Core.inRange(hsv, new Scalar(47, 150, 150), new Scalar(93, 255, 255), thresholded_green);
            // Core.inRange(hsv, new Scalar(313, 150, 150), new Scalar(93, 255, 255), thresholded_purple);
            // Core.inRange(hsv, new Scalar(148, 100, 100), new Scalar(179, 255, 255), thresholded_pink);
            Core.inRange(hsv, new Scalar(47, 100, 100), new Scalar(93, 255, 255), thresholded_green);
            Core.inRange(hsv, new Scalar(109, 100, 100), new Scalar(93, 255, 255), thresholded_purple);
            Core.inRange(hsv, new Scalar(148, 100, 100), new Scalar(179, 255, 255), thresholded_pink);
            /* find contours */
            contours_orange = new ArrayList<>();
            contours_green = new ArrayList<>();
            contours_purple = new ArrayList<>();
            contours_pink = new ArrayList<>();
            Imgproc.findContours(thresholded_green, contours_green, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(thresholded_purple, contours_purple, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            //Imgproc.findContours(thresholded_pink, contours_pink, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            /* create a bounding rect based on the largest contour */

            if (showContours && !contours_green.isEmpty()) {

                largest_area = 0;
                for (int i = 0; i < contours_green.size(); i++) /* iterate through the contours */ {
                    double area = Imgproc.contourArea(contours_green.get(i));  /* get contour area */
                    if (area > largest_area) {
                        largest_area = area; /* save the largest contour area */

                        /* get a bounding rectangle based on the largest contour */
                        bounding_rect_orange = Imgproc.boundingRect(contours_green.get(i));
                    }
                    green_area = largest_area;
                }

                /* draw the contours and the bounding rect */
                Imgproc.drawContours(rgba, contours_green, -1, new Scalar(255, 255, 0), 1, 8);

            }

            if (showContours && !contours_purple.isEmpty()) {

                largest_area = 0;
                for (int i = 0; i < contours_purple.size(); i++) /* iterate through the contours */ {
                    double area = Imgproc.contourArea(contours_purple.get(i));  /* get contour area */
                    if (area > largest_area) {
                        largest_area = area; /* save the largest contour area */

                        /* get a bounding rectangle based on the largest contour */
                        bounding_rect_orange = Imgproc.boundingRect(contours_purple.get(i));
                    }
                    purple_area = largest_area;
                }

                /* draw the contours and the bounding rect */
                Imgproc.drawContours(rgba, contours_purple, -1, new Scalar(255, 255, 0), 1, 8);

            }
/*
           /*if (showContours && !contours_pink.isEmpty()) {

                largest_area = 0;
                for (int i = 0; i < contours_pink.size(); i++) {
                  //  double area = Imgproc.contourArea(contours_pink.get(i));
                   // if (area > largest_area) {
                        //largest_area = area;
//

                        bounding_rect_orange = Imgproc.boundingRect(contours_pink.get(i));
                    }
                    pink_area = largest_area;
                }
                Imgproc.drawContours(rgba, contours_pink, -1, new Scalar(255, 255, 0), 1, 8);

            }*/

            bounding_rect_orange_global = bounding_rect_orange;

            telemetry.addData("Area ", largest_area);

            hsv.release();
            thresholded_green.release();
            thresholded_pink.release();
            thresholded_purple.release();

            grey.release();

            /* return the rgba matrix */
            return rgba;
        }

        @Override
        public void onViewportTapped() {
            phoneCam.resumeViewport();
        }
    }
    /* computer vision end ------------------------------------------------------------------------------------- */

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");
        lift = hardwareMap.dcMotor.get("lift");
        flipper = hardwareMap.servo.get("flipper");
        clawLeft = hardwareMap.servo.get("clawLeft");
        clawRight = hardwareMap.servo.get("clawRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        //frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        //backRight.setDirection(DcMotor.Direction.REVERSE);

        COUNTS_PER_MOTOR_REV = 537.7;
        DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP (32 teeth to 16 teeth)
        WHEEL_DIAMETER_INCH = 1.889765 * 2;     // For figuring circumference
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCH * 3.1415);
        ROBOT_CIRCUMFERENCE = ROBOT_DIAMETER * 3.1415;

        cvinit();

        waitForStart();
        runtime.reset();

        // tile forward
        //  flipper.setPosition(1);
        //  sleep(4500);

       encoderMoveINCH(49, 49, 0.3);
       encoderStrafeINCH(24, 0.8);
//         lift.setPower(1);
//         sleep(4500);
//         lift.setPower(0);
//         telemetry.addData("status", "liftworking");
//          flipper.setPosition(1);
//          sleep(4500);
//          flipper.setPosition(0);

//        if(green_area > 100){
//            //green
//            encoderStrafeINCH(6, 0.8);
//            encoderMoveINCH(12, 12, 0.3);
//        }
//        else if(purple_area < 100){
//            //purple
//            encoderStrafeINCH(-6, 0.8);
//            encoderMoveINCH(12, 12, 0.3);
//        }
//        else{
//            //pink
// z           encoderMoveINCH(12, 12, 0.3);
    }

    //encoderStrafeINCH(-12, 0.8);
    //encoderMoveINCH(20, -20, 0.3);


    // encoderMoveINCH(10, -10, 0.5);

    //deposit

    //starfe 0.5 tile
    //turn 90 right
    //pick cone


    int leftPos = 0;
    int rightPos = 0;

    public void encoderStrafeINCH(double inches, double power) {
        int ticks = (int) (inches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - ticks);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + ticks);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + ticks);
        backRight.setTargetPosition(backRight.getCurrentPosition() - ticks);

        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //continue moving
        while (opModeIsActive() && ((frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()))) {
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    public void encoderMoveINCH(double leftINCH, double rightINCH, double power) {
        int newLeftTarget = frontLeft.getCurrentPosition() + (int) (leftINCH * COUNTS_PER_INCH);
        int newRightTarget = frontRight.getCurrentPosition() + (int) (rightINCH * COUNTS_PER_INCH);
        double leftSpeed;
        double rightSpeed;

        frontLeft.setTargetPosition(newLeftTarget);
        backLeft.setTargetPosition(newLeftTarget);
        frontRight.setTargetPosition(newRightTarget);
        backRight.setTargetPosition(newRightTarget);

        if (Math.abs(leftINCH) > Math.abs(rightINCH)) {
            leftSpeed = power;
            rightSpeed = (power * rightINCH) / leftINCH;
        } else {
            rightSpeed = power;
            leftSpeed = (power * leftINCH) / rightINCH;
        }

        frontLeft.setPower(leftSpeed);
        frontRight.setPower(rightSpeed);
        backLeft.setPower(leftSpeed);
        backRight.setPower(rightSpeed);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //continue movinggg
        while (opModeIsActive() && ((frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()))) {
        }

        // Stop all motion;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }
}