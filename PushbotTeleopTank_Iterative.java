package archive_21;
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
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Stormbot TeleopTank_Iterative", group="Pushbot")

public class PushbotTeleopTank_Iterative extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor  leftFront   = null;
    public DcMotor  leftBack    = null;
    public DcMotor  rightFront  = null;
    public DcMotor  rightBack  = null;
    public DcMotor  turret  = null;
   // public ColorSensor colorturret  = null;
    public DcMotor  spinner  = null;
    public DcMotor  collector   = null;
    public DcMotor  extension    = null;
    public DcMotor  depositor    = null;
    public Servo    dropper     = null;
    public Servo flipper = null;

    public static final double SERVO_POSITION = 0.7;
    public static final double SERVO_RETRACTED_POSITION = 1.0;

    /* Declare OpMode members. */
   // private ElapsedTime run  = new ElapsedTime();
    HardwarePushbot robot = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    double clawOffset = 0.0;                  // Servo mid position
    final double CLAW_SPEED = 0.02;                 // sets rate to move servo
    double TPR = 384.5;
    double TP360 = TPR * 8;

    @Override
    public void init() {

        robot.leftFront = hardwareMap.dcMotor.get("leftFront");
        robot.leftBack = hardwareMap.dcMotor.get("leftBack");
        robot.rightFront = hardwareMap.dcMotor.get("rightFront");
        robot.rightBack = hardwareMap.dcMotor.get("rightBack");
        robot.spinner = hardwareMap.dcMotor.get("spinner");
        robot.depositor = hardwareMap.servo.get("depositor");
        robot.turret = hardwareMap.dcMotor.get("turret");
        robot.collector = hardwareMap.dcMotor.get("collector");
        robot.extension = hardwareMap.dcMotor.get("extension");
        robot.flipper = hardwareMap.servo.get("flipper");
        robot.colorturret = hardwareMap.colorSensor.get("colorturret");

        robot.leftFront.setDirection(DcMotor.Direction.REVERSE);
        robot.leftBack.setDirection(DcMotor.Direction.REVERSE);


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

        robot.leftFront.setPower(-gamepad1.left_stick_y);
        robot.rightFront.setPower(gamepad1.right_stick_y);
        robot.leftBack.setPower(-gamepad1.left_stick_y);
        robot.rightBack.setPower(-gamepad1.right_stick_y);


        robot.turret.setPower(gamepad2.left_stick_y);
        robot.extension.setPower(-gamepad2.right_stick_y);
        if (gamepad2.dpad_up) {
            telemetry.addData("Blue", robot.colorturret.blue());
            telemetry.addData("Red", robot.colorturret.red());
            telemetry.update();

            while (robot.colorturret.blue() < 800) {
                turret.setPower(0.2);
            }
            turret.setPower(0);
        }

        if (gamepad2.dpad_down) {
            telemetry.addData("Blue", robot.colorturret.blue());
            telemetry.addData("Red", robot.colorturret.red());
            telemetry.update();

            while (robot.colorturret.red() < 1000) {
                turret.setPower(-0.2);
            }
            turret.setPower(0);
        }

            if (gamepad2.a) {
                robot.depositor.setPosition(0.85);
                robot.flipper.setPosition(1);
            }
            if (gamepad2.b) {
                robot.flipper.setPosition(0.5);
            }
            if (gamepad2.x) {
                robot.depositor.setPosition(0.5);
                robot.flipper.setPosition(-1);
            }

            if (gamepad1.left_trigger >= 1.0 && gamepad1.right_trigger < 1.0) {
                robot.leftFront.setPower(-1.0);
                robot.rightFront.setPower(-1.0);
                robot.leftBack.setPower(1.0);
                robot.rightBack.setPower(-1.0);

            } else if (gamepad1.left_trigger < 1.0 && gamepad1.right_trigger >= 1.0) {

                robot.leftFront.setPower(1.0);
                robot.rightFront.setPower(1.0);
                robot.leftBack.setPower(-1.0);
                robot.rightBack.setPower(1.0);
            } else {
                robot.leftFront.setPower(0);
                robot.rightFront.setPower(0);
                robot.leftBack.setPower(0);
                robot.rightBack.setPower(0);
            }


            if (gamepad2.left_trigger >= 1.0 && gamepad2.right_trigger < 1.0) {
                robot.depositor.setPosition(1);

            }
            else if (gamepad2.left_trigger < 1.0 && gamepad2.right_trigger >= 1.0) {
                robot.depositor.setPosition(0.5);
            }


            if (gamepad1.right_bumper)
                robot.collector.setPower(0.8);
            else if (gamepad1.left_bumper)
                robot.collector.setPower(-0.8);
            else
                robot.collector.setPower(0.0);

            if (gamepad2.left_bumper)
                robot.spinner.setPower(0.4);
            else if (gamepad2.right_bumper) {
                robot.spinner.setPower(-0.4);
            } else {
                robot.spinner.setPower(0);
            }

        }

        @Override
        public void stop () {
        }
    }








