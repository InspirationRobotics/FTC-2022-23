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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="WarehousePark_RED", group="SimpleAuto")
//@Disabled
public class WarehousePark_RED extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor collector;
    Servo dropper;

    double power = 0.5;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        collector = hardwareMap.dcMotor.get("collector");
        dropper = hardwareMap.servo.get("dropper");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        //deposit element onto shipping hub
        dropper.setPosition(0.5);
        sleep(3000);

        //going backward
        leftFront.setPower(-0.22);
        rightFront.setPower(-0.22);
        leftBack.setPower(-0.22);
        rightBack.setPower(-0.22);

        sleep(2050);

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);

        //deposit element onto shipping hub
        dropper.setPosition(0.0);
        sleep(2000);
        dropper.setPosition(0.5);
        sleep(2000);

        //going forward
        leftFront.setPower(0.23);
        rightFront.setPower(0.23);
        leftBack.setPower(0.23);
        rightBack.setPower(0.23);

        sleep(2600);

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);

        //going back
        leftFront.setPower(-0.23);
        rightFront.setPower(-0.23);
        leftBack.setPower(-0.23);
        rightBack.setPower(-0.23);

        sleep(2600);

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);
// bringing dropper back
        dropper.setPosition(0.9);
        sleep(2000);

        //tank turn
        leftFront.setPower(-1*0.234);
        rightFront.setPower(-1*-0.234);
        leftBack.setPower(-1*0.234);
        rightBack.setPower(-1*-0.234);

        sleep(2100);

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);

        //going forward

        leftFront.setPower(1.0);
        rightFront.setPower(1.0);
        leftBack.setPower(1.0);
        rightBack.setPower(1.0);

        sleep(950);

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);

        //going forward, but slowing it down.

        leftFront.setPower(0.3);
        rightFront.setPower(0.3);
        leftBack.setPower(0.3);
        rightBack.setPower(0.3);

        sleep(1500);

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);

        //strafe to left
        leftFront.setPower(-1*0.2);
        rightFront.setPower(-1*0.2);
        leftBack.setPower(-1*-0.2);
        rightBack.setPower(-1*-0.2);

        sleep(2000);

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);

        //going a little backwards
        leftFront.setPower(0.2);
        rightFront.setPower(0.2);
        leftBack.setPower(0.2);
        rightBack.setPower(0.2);

        sleep(1000);

        leftFront.setPower(0.0);
        rightFront.setPower(0.0);
        leftBack.setPower(0.0);
        rightBack.setPower(0.0);

        //collector intakes

        collector.setPower (-0.9);
        sleep(3300);
        collector.setPower(0.0);

        //collector outakes

        collector.setPower (0.7);
        sleep(3300);
        collector.setPower(0.0);







    }
}
