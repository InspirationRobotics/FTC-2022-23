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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Pushbot: TeleopTank", group="Pushbot")

public class PushbotTeleopTank_Iterative extends OpMode{

    public static final double SERVO_POSITION = 0.0;
    public static final double SERVO_RETRACTED_POSITION = 0.3;

    /* Declare OpMode members. */
    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Init the hardware variables
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        robot.leftFront.setPower(gamepad1.left_stick_y);
        robot.rightFront.setPower(-gamepad1.right_stick_y);
        robot.leftBack.setPower(gamepad1.left_stick_y);
        robot.rightBack.setPower(-gamepad1.right_stick_y);

        if (gamepad1.left_trigger >= 1.0 && gamepad1.right_trigger < 1.0) {
            // right trigger is not pressed value of 0
            // left trigger is pressed values greater than 0.5
            robot.leftFront.setPower(-1.0);
            robot.rightFront.setPower(1.0);
            robot.leftBack.setPower(1.0);
            robot.rightBack.setPower(-1.0);

        } else if (gamepad1.left_trigger < 1.0 && gamepad1.right_trigger >= 1.0) {
            // right trigger is pressed value greater than 0.5
            // left trigger is not pressed value of 0
            robot.leftFront.setPower(1.0);
            robot.rightFront.setPower(-1.0);
            robot.leftBack.setPower(-1.0);
            robot.rightBack.setPower(1.0);
        } else {
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightBack.setPower(0);


        }

        // Use gamepad buttons to move the arm up (Y) and down (A)
      if (gamepad1.right_bumper)
          robot.collector.setPower(1);
       else if (gamepad1.left_bumper)
           robot.collector.setPower(-1);
       else
           robot.collector.setPower(0.0);

      if(gamepad2.left_bumper)robot.spinner.setPower(0.4);
      else if (gamepad2.right_bumper) {
           robot.spinner.setPower(-0.4);
      }
       else {
       robot.spinner.setPower(0);
      }
       robot.extender.setPower(-gamepad2.left_stick_y);

        if(gamepad2.left_trigger > 0.1){
            robot.dropper.setPosition(SERVO_POSITION);
        }

        if(gamepad2.right_trigger > 0.1){
            robot.dropper.setPosition(SERVO_RETRACTED_POSITION);
        }

        if (gamepad2.dpad_down) {
            robot.flipperLeft.setPosition(0.5);
            robot.flipperRight.setPosition(0);
            robot.dropper.setPosition(1);
        }

        if (gamepad2.dpad_up){
            robot.flipperLeft.setPosition(0.0);
            robot.flipperRight.setPosition(0.5);
            robot.dropper.setPosition(0.3);


        }

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */



    @Override
    public void stop() {
    }
}
