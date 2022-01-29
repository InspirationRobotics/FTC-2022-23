package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.EncoderEx;

@Autonomous (name = "Vuforia")
public class Vuforia extends LinearOpMode
{
 VuforiaLocalizer vurforiaLocalizer;
 VuforiaLocalizer.Parameters parameters;
 //VuforiaTrackables visionTargets = //vuforiaLocalizer.loa;
 VuforiaTrackable target;
 VuforiaTrackableDefaultListener listener;

 OpenGLMatrix lastknownLocation;
 OpenGLMatrix driverhubLocation;

 public static final String VUFORIA_KEY = "";
 //need key

 public void runOpMode () throws InterruptedException
 {


  waitForStart();

  while(opModeIsActive())
  {


   telemetry.update();
   idle();
  }
 }

 public void setupVuforia()
 {

  parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
  parameters.vuforiaLicenseKey = VUFORIA_KEY;
  parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
 // vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

 }
}
//*

