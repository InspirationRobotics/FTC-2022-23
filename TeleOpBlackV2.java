package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="TeleOpBlackV2", group="Pushbot")
public class TeleOpBlackV2 extends OpMode {

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

    @Override
    public void init() {

        robot.frontLeft = hardwareMap.dcMotor.get("frontLeft");
        robot.frontRight = hardwareMap.dcMotor.get("frontRight");
        robot.backLeft = hardwareMap.dcMotor.get("backLeft");
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

        robot.lift.setPower(gamepad2.right_stick_y);

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
}
