package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
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

@TeleOp(name="TRENCH_T", group="Pushbot")
public class TRENCH_T extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    static DcMotor frontLeft;
    static DcMotor frontRight;
    static DcMotor backLeft;
    static DcMotor backRight;
    public Servo clawRight   = null;
    public Servo  clawLeft    = null;
    public Servo  flipper = null;
    public DcMotor  lift  = null;

    public static final double SERVO_POSITION = 0.7;
    public static final double SERVO_RETRACTED_POSITION = 1.0;

    /* Declare OpMode members. */
    // private ElapsedTime run  = new ElapsedTime();
    Robot robot = new Robot(); // use the class created to define a Pushbot's hardware
    double clawOffset = 0.0;                  // Servo mid position
    final double CLAW_SPEED = 0.02;                 // sets rate to move servo
    double TPR = 384.5;
    double TP360 = TPR * 8;

    public static final double STRAFE_SPEED = 1;

    @Override
    public void init() {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        lift = hardwareMap.dcMotor.get("lift");
        flipper = hardwareMap.servo.get("flipper");
        clawLeft = hardwareMap.servo.get("clawLeft");
        clawRight = hardwareMap.servo.get("clawRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);


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
        frontLeft.setPower(-gamepad1.left_stick_y);
        frontRight.setPower(-gamepad1.right_stick_y);
        backLeft.setPower(-gamepad1.left_stick_y);
        backRight.setPower(-gamepad1.right_stick_y);

        if (gamepad1.right_bumper) {
            frontLeft.setPower(STRAFE_SPEED);
            frontRight.setPower(STRAFE_SPEED);
            backLeft.setPower(STRAFE_SPEED);
            backRight.setPower(STRAFE_SPEED);
        } else if (gamepad1.left_bumper) {
            frontLeft.setPower(-STRAFE_SPEED);
            frontRight.setPower(-STRAFE_SPEED);
            backLeft.setPower(-STRAFE_SPEED);
            backRight.setPower(-STRAFE_SPEED);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        /* gamepad 2 start ------------------------------------------------*/

        lift.setPower(gamepad2.right_stick_y);

        if (gamepad2.dpad_up) {
            flipper.setPosition(1);
            flipper.setPosition(0);
        }
        if (gamepad2.dpad_down) {
            flipper.setPosition(0);
            flipper.setPosition(1);
        }
        if (gamepad2.dpad_right) {
            flipper.setPosition(0.5);
            flipper.setPosition(0.5);
        }

        if (gamepad2.a) {
            clawLeft.setPosition(0);
            clawRight.setPosition(1);
        }
        if (gamepad2.x) {
            clawLeft.setPosition(0.5);
            clawRight.setPosition(0.5);
        }

        if (gamepad2.y) {
            clawLeft.setPosition(0.7);
            clawRight.setPosition(0);
        }
    }

}


