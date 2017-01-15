package org.firstinspires.ftc.teamcode;

/**
 * Created by FTC2875 on 9/25/2016.
 */

import android.graphics.Color;
import android.hardware.SensorManager;
import android.view.View;

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;

import com.qualcomm.ftccommon.AboutActivity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.PWMOutputController;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.ftccommon.external.SoundPlayingRobotMonitor;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.AppUtil;
import org.firstinspires.inspection.RcInspectionActivity;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

import java.io.File;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

import ftc.vision.BeaconProcessor;
import ftc.vision.ImageProcessorResult;
import ftc.vision.ImageUtil;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.opencv.android.JavaCameraView;

@Autonomous(name="Autonomous Beacon", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class Auto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /*-----------------------------------------------------------------------
    | Motor Declarations
    *-----------------------------------------------------------------------*/
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftBackMotor = null;
    DcMotor rightBackMotor = null;



    /*-----------------------------------------------------------------------
    | Sensor Declarations
    *-----------------------------------------------------------------------*/
    AnalogInput bottomLeftSensor = null;
    AnalogInput bottomRightSensor = null;
    AnalogInput bottomMiddleSensor = null;
    DeviceInterfaceModule CDI = null;

    /*-----------------------------------------------------------------------
    | Global Variables
    *-----------------------------------------------------------------------*/
    double colorThreshold = 3  ;
    boolean beaconChosen = false;
    boolean beaconLoop = false;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /*-----------------------------------------------------------------------
        | Motor Hardware Declarations
        *-----------------------------------------------------------------------*/
        leftFrontMotor = hardwareMap.dcMotor.get("left front");
        rightFrontMotor = hardwareMap.dcMotor.get("right front");
        leftBackMotor = hardwareMap.dcMotor.get("left back");
        rightBackMotor = hardwareMap.dcMotor.get("right back");

        /*-----------------------------------------------------------------------
        | Sensor Hardware Declarations
        *-----------------------------------------------------------------------*/
        bottomLeftSensor = hardwareMap.analogInput.get("bottom left");
        bottomRightSensor = hardwareMap.analogInput.get("bottom right");
        bottomMiddleSensor = hardwareMap.analogInput.get("bottom middle");
        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        /*-------------------------------------------------------------------------------
        | Encoder crap
        | "Reverse" the motor that runs backwards when connected directly to the battery
        *------------------------------------------------------------------------------*/
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        /*-----------------------------------------------------------------------
        | MAIN LOOP MAIN LOOP MAIN LOOP MAIN LOOP MAIN LOOP
        | MAIN LOOP MAIN LOOP MAIN LOOP MAIN LOOP MAIN LOOP
        | MAIN LOOP MAIN LOOP MAIN LOOP MAIN LOOP MAIN LOOP
        *-----------------------------------------------------------------------*/
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            fun();
            //goToBeacon();

            //beaconLineFollow();
            //chooseBeacon();
            //hitBeacon();
            forwardFor(16, 0.2);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
            sleep(10000);
        }
    }

    /*-----------------------------------------------------------------------
    | GAMEPLAY PROGRESSION METHODS
    *-----------------------------------------------------------------------*/
    public void goToBeacon() throws InterruptedException {
        //drives to the white line
        //strafeLeftFor(60, 1);
        forwardFor(60, 1);
        // TODO: Find a Way to strafe using gyro on phone to auto correct strafe. Replace encoders with gyros
    }

    public void beaconLineFollow() throws InterruptedException {
        // line follows the white line
        // 4 = not white

        boolean foundLine = false;
        while(!foundLine) {

            double middleColor = bottomMiddleSensor.getVoltage(); //
            double leftColor = bottomLeftSensor.getVoltage();     //
            double rightColor = bottomRightSensor.getVoltage();   //
            telemetry.addData("Sensor Right: ", rightColor);      // Sensor Reading Code
            telemetry.addData("Sensor Left: ", leftColor);        //
            telemetry.addData("Sensor Middle: ", middleColor);    //
            telemetry.update();                                   //

            boolean unknownState = false;
            if (middleColor < colorThreshold) { // aligned perfectly
                foundLine = true;
                stopMotors();
            } else if (leftColor < colorThreshold) {  // aligned too right
                leftFor(1, 0.1);
            } else if (rightColor < colorThreshold) { // aligned too left
                rightFor(1, 0.1);
            } else {                                  // unknown alignment
                leftFor(1, 0.1);
                rightFor(1, 0.1);

                if (unknownState == true) {
                    rightFor(1, 0.1);
                    leftFor(1, 0.1);
                    unknownState = false;
                }
                unknownState = true;
            }
        }

        // we have found the line
        forwardFor(4, 1);
    }

    public void chooseBeacon() throws InterruptedException {
        // chooses which beacon to go to
        // assuming we are on red team
        telemetry.addData("Status: ", "Choosing Beacon");
        telemetry.update();

        if(!beaconChosen) {
            grabFrame();
            beaconChosen = true;
        }
        ImageProcessorResult imageProcessorResult = FtcRobotControllerActivity.frameGrabber.getResult();
        BeaconColorResult result = (BeaconColorResult) imageProcessorResult.getResult();

        BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
        if (leftColor.toString().equals("RED") && !beaconLoop) {
            // telemetry data
            telemetry.addData("Left:", "RED");
            telemetry.addData("Right:,", "BLUE");
            telemetry.update();

            // go to the left and hit red button
            strafeLeftFor(3, 0.5);
            stopMotors();
            beaconLoop = true;

        } else if (leftColor.toString().equals("BLUE") && !beaconLoop) {
            // telemetry data
            telemetry.addData("Left:", "BLUE");
            telemetry.addData("Right:", "RED");
            telemetry.update();

            // go to the right and hit red button
            strafeRightFor(2, 0.5);
            stopMotors();
            beaconLoop = true;
        }

    }

    public void hitBeacon() throws InterruptedException{
        forwardFor(10, 0.1);
    }

    /*-----------------------------------------------------------------------
    | MOVEMENT FUNCTIONS
    | NOTE: Yellow wire is in front for encoders
    *-----------------------------------------------------------------------*/
    public void forwardFor(int inches, double speed) throws InterruptedException {
        encoderMove(inches, inches, -inches, -inches, speed);
    }

    public void rightFor(int inches, double speed) throws InterruptedException {
        encoderMove(inches, inches, inches, inches, speed);
    }

    public void leftFor(int inches, double speed) throws InterruptedException {
        encoderMove(inches, inches, inches, inches, speed);
    }

    public void strafeLeftFor(int inches, double speed) throws InterruptedException {
        encoderMove(-inches, inches, -inches, inches, speed);
    }

    public void strafeRightFor(int inches, double speed) throws InterruptedException {
        encoderMove(inches, -inches, inches, -inches, speed);

    }

    public void encoderMove(int leftFront,int leftBack,int rightFront,int rightBack, double speed) throws InterruptedException{
        leftFrontMotor.setTargetPosition((int)(leftFront * COUNTS_PER_INCH) + leftFrontMotor.getCurrentPosition());
        leftBackMotor.setTargetPosition((int)(leftBack * COUNTS_PER_INCH) + leftBackMotor.getCurrentPosition());
        rightFrontMotor.setTargetPosition((int)(rightFront * COUNTS_PER_INCH) + rightFrontMotor.getCurrentPosition());
        rightBackMotor.setTargetPosition((int)(rightBack * COUNTS_PER_INCH) + rightBackMotor.getCurrentPosition());

        // Sets to run to position
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // RUN
        leftFrontMotor.setPower(Math.abs(speed));
        leftBackMotor.setPower(Math.abs(speed));
        rightFrontMotor.setPower(Math.abs(speed));
        rightBackMotor.setPower(Math.abs(speed));

        while (leftFrontMotor.isBusy() || leftBackMotor.isBusy() || rightFrontMotor.isBusy() || rightBackMotor.isBusy()) {
            idle();
            telemetry.addData("Current Position Left Front: ", leftFrontMotor.getCurrentPosition());
            telemetry.addData("Target Position Left Front: ", leftFrontMotor.getTargetPosition());
            telemetry.addLine();

            telemetry.addData("Current Position Left Back: ", leftBackMotor.getCurrentPosition());
            telemetry.addData("Target Position Left Back: ", leftBackMotor.getTargetPosition());
            telemetry.addLine();

            telemetry.addData("Current Position Right Front: ", rightFrontMotor.getCurrentPosition());
            telemetry.addData("Target Position Right Front: ", rightFrontMotor.getTargetPosition());
            telemetry.addLine();

            telemetry.addData("Current Position Right Back: ", rightBackMotor.getCurrentPosition());
            telemetry.addData("Target Position Right Back: ", rightBackMotor.getTargetPosition());
            telemetry.update();
        }

        stopMotors();

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void stopMotors(){
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }

    /*-----------------------------------------------------------------------
    | MISC FUNCTIONS
    *-----------------------------------------------------------------------*/
    public void fun() throws InterruptedException {
        CDI.setLED(0, true);
        sleep(500);
        CDI.setLED(0, false);
        sleep(500);
        CDI.setLED(1, true);
        sleep(500);
        CDI.setLED(1, false);
        sleep(500);

    }

    public void grabFrame() {
        FtcRobotControllerActivity.frameGrabber.grabSingleFrame();
        while (!FtcRobotControllerActivity.frameGrabber.isResultReady()) {
            try {
                Thread.sleep(5); //sleep for 5 milliseconds
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        Object result = FtcRobotControllerActivity.frameGrabber.getResult();

    }
}