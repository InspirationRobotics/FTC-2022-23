package org.firstinspires.ftc.teamcode;

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

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


@TeleOp(name="cv", group="Pushbot")
public class auto extends OpMode {

    public OpenCvInternalCamera phoneCam;
    public PowerPlayDeterminationPipeline pipeline;


    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor frontLeft   = null;
    public DcMotor  frontRight    = null;
    public DcMotor  backLeft = null;
    public DcMotor  backRight  = null;
    //ColorSensor color_sensor;

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
    VuforiaLocalizer vuforia    = null;

    public void cvinit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
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

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
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
            public void onError(int errorCode)
            {
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
    @Override
    public void init() {

        cvinit();
        robot.frontLeft = hardwareMap.dcMotor.get("frontLeft");
        robot.frontRight = hardwareMap.dcMotor.get("frontRight");
        robot.backLeft= hardwareMap.dcMotor.get("backLeft");
        robot.backRight = hardwareMap.dcMotor.get("backRight");
        //color_sensor = hardwareMap.colorSensor.get("color");


        robot.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Say", "Hello Driver");    //
    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override

    public void init_loop() {

    }


    @Override
    public void start() {

    }

    @Override
    public void loop() {
        /* gamepad 1 start ------------------------------------------------*/
        robot.frontLeft.setPower(-gamepad1.left_stick_y);
        robot.frontRight.setPower(gamepad1.right_stick_y);
        robot.backLeft.setPower(-gamepad1.left_stick_y);
        robot.backRight.setPower(-gamepad1.right_stick_y);

        if (gamepad1.left_bumper) {
            robot.frontLeft.setPower(-STRAFE_SPEED);
            robot.frontRight.setPower(STRAFE_SPEED);
            robot.backLeft.setPower(STRAFE_SPEED);
            robot.backRight.setPower(-STRAFE_SPEED);
        } else if (gamepad1.right_bumper) {
            robot.frontLeft.setPower(STRAFE_SPEED);
            robot.frontRight.setPower(-STRAFE_SPEED);
            robot.backLeft.setPower(-STRAFE_SPEED);
            robot.backRight.setPower(STRAFE_SPEED);
        } else {
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backLeft.setPower(0);
            robot.backRight.setPower(0);
        }

        /* gamepad 2 start ------------------------------------------------*/

        /*robot.lift.setPower(gamepad2.right_stick_y);
        robot.lift.setPower(0.0);*/

        if (gamepad2.left_bumper) {
            robot.flipper.setPosition(1);
            robot.flipper.setPosition(0);
        }
        if (gamepad2.right_bumper) {
            robot.flipper.setPosition(0);
            robot.flipper.setPosition(1);
        }

        if (gamepad2.a) {
            robot.clawLeft.setPosition(0);
            robot.clawRight.setPosition(0);
        }
        if (gamepad2.b) {
            robot.clawLeft.setPosition(1);
            robot.clawRight.setPosition(1);
        }
       /*     if(!gamepad1.left_bumper && gamepad1.right_bumper) {
            robot.lift.setPower(-1);
        } else if (!gamepad1.right_bumper && gamepad1.left_bumper) {
            robot.lift.setPower(1);
        } else if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
            robot.lift.setPower(0);
        }

        */
    }
    @Override
    public void stop () {
    }

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
            Imgproc.findContours(thresholded_pink, contours_pink, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            /* create a bounding rect based on the largest contour */

            if (showContours && !contours_orange.isEmpty()) {

                largest_area = 0;
                for (int i = 0; i < contours_orange.size(); i++) /* iterate through the contours */ {
                    double area = Imgproc.contourArea(contours_orange.get(i));  /* get contour area */
                    if (area > largest_area) {
                        largest_area = area; /* save the largest contour area */

                        /* get a bounding rectangle based on the largest contour */
                        bounding_rect_orange = Imgproc.boundingRect(contours_orange.get(i));
                    }
                }

                /* draw the contours and the bounding rect */
                Imgproc.drawContours(rgba, contours_orange, -1, new Scalar(255, 255, 0), 1, 8);

            }


            bounding_rect_orange_global = bounding_rect_orange;

            telemetry.addData("Area ", largest_area);

            hsv.release();
            thresholded_green.release();
            thresholded_pink.release();
            thresholded_purple.release();


            grey.release();


            if (bounding_rect_orange_global.height == 0) {
                return rgba;
            } else if (bounding_rect_orange_global.width / bounding_rect_orange_global.height > 2.5) {
                ringnum = 1;
            } else if (largest_area < 150) {
                ringnum = 0;
            } else {
                ringnum = 4;
            }

            /* return the rgba matrix */
            return rgba;
        }

        @Override
        public void onViewportTapped()
        {
            phoneCam.resumeViewport();
        }

        public int returnNum() {
            return ringnum;
        }
    }
    /* computer vision end ------------------------------------------------------------------------------------- */

}



