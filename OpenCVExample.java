

//package org.firstinspires.ftc.teamcode;

 /*
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.MatOfPoint2f;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.eazsyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;
//import java.util.ArrayList;
//import java.util.List;

@Autonomous
public class OpenCVExample extends LinearOpMode {
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(new SamplePipeline());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {

            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            if (gamepad1.a) {

                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }

            sleep(100);
        }
    }


    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.rectangle(
                    input,
                    new Point(
                            input.cols() / 4,
                            input.rows() / 4),
                    new Point(
                            input.cols() * (3f / 4f),
                            input.rows() * (3f / 4f)),
                    new Scalar(0, 255, 0), 4);


            return input;
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }
    public static class SkystoneDetector extends OpenCvPipeline {
        private DropBoxManager telemetry;

        enum SkystoneLocation {
            LEFT,
            RIGHT,
            MIDDLE
        }

        private int width; // width of the image
        SkystoneLocation location;

        public SkystoneDetector(int width) {
            this.width = width;
        }

        @Override
        public Mat processFrame(Mat input) {
            // "Mat" stands for matrix, which is basically the image that the detector will process
            // the input matrix is the image coming from the camera
            // the function will return a matrix to be drawn on your phone's screen

            // The detector detects regular stones. The camera fits two stones.
            // If it finds one regular stone then the other must be the skystone.
            // If both are regular stones, it returns NONE to tell the robot to keep looking

            // Make a working copy of the input matrix in HSV
            Mat mat = new Mat();
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

            // if something is wrong, we assume there's no skystone
            if (mat.empty()) {
                location = SkystoneLocation.MIDDLE;
                return input;
            }

            // We create a HSV range for green to detect regular stones
            // NOTE: In OpenCV's implementation,
            // Hue values are half the real value
            Scalar lowHSV = new Scalar(89, 67, 61); // lower bound HSV for green
            Scalar highHSV = new Scalar(83, 43, 83); // higher bound HSV for green
            Mat thresh = new Mat();

            // We'll get a black and white image. The white regions represent the regular stones.
            // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
            Core.inRange(mat, lowHSV, highHSV, thresh);

            // Use Canny Edge Detection to find edges
            // you might have to tune the thresholds for hysteresis
            Mat edges = new Mat();
            Imgproc.Canny(thresh, edges, 100, 300);

            // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
            // Oftentimes the edges are disconnected. findContours connects these edges.
            // We then find the bounding rectangles of those contours
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            }

            // Iterate and check whether the bounding boxes
            // cover left and/or right side of the image
            double left_x = 0.25 * width;

            double right_x = 0.75 * width;
            boolean left = false; // true if regular stone found on the left side
            boolean right = false; // "" "" on the right side
            int position = 0;
            for (int i = 0; i != boundRect.length; i++) {
                if (boundRect[i].x < left_x)
                    position = 1;
                if (boundRect[i].x + boundRect[i].width > right_x)
                    position = 2;
                    System.out.println(position);

                // draw red bounding rectangles on mat
                // the mat has been converted to HSV so we need to use HSV as well
                Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));

            }
            System.out.println(position);
            return mat;


        }

        public SkystoneLocation getLocation() {
            return this.location;
        }
    }

}

  */

