package org.firstinspires.ftc.teamcode;

/**
 * Created by FTC2875 on 9/25/2016.
 */

import android.content.Context;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.view.View;

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;

import com.qualcomm.ftccommon.AboutActivity;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
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
import java.util.ArrayList;
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

@Autonomous(name="Autonomous Beacon Red", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class AutoRed extends LinearOpMode implements SensorEventListener {

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
    double colorThreshold = 3.1  ;
    boolean beaconChosen = false;
    boolean beaconLoop = false;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    private static SensorManager mSensorManager;
    private static Sensor mSensor;

    private float axisX;
    private float axisY;
    private float axisZ;

    private float initRotation;
    private boolean rotationFlag;

    private double speedFactor = 1;
    private final int rotationThreshold = 0;
    private ModernRoboticsI2cGyro gyro = null;
    private static final double LITTLE_VALUE = 0.7;
    private static final double BIGGER_VALUE = 1.25;

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
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

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

        mSensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        mSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);

        mSensorManager.registerListener(this, mSensor, SensorManager.SENSOR_DELAY_FASTEST);

        gyro.calibrate();
        while (gyro.isCalibrating()) {

        }
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
//            fun();
//            goToBeacon();
//
//            beaconLineFollow();
            chooseBeacon();
            hitBeacon();
            sleep(10000);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    /*-----------------------------------------------------------------------
    | GAMEPLAY PROGRESSION METHODS
    *-----------------------------------------------------------------------*/
    public void goToBeacon() throws InterruptedException {
        //drives to the white line
        strafeLeftFor(56, 0.5);
        forwardFor(23, 0.5);

    }

    public void beaconLineFollow() throws InterruptedException {
        // line follows the white line
        // 4 = not white
        boolean unknownState = false;
        boolean foundLine = false;
        int i = 0;

        while(!foundLine) {
            double middleColor = bottomMiddleSensor.getVoltage(); //
            double leftColor = bottomLeftSensor.getVoltage();     //
            double rightColor = bottomRightSensor.getVoltage();   //
            telemetry.addData("Sensor Right: ", rightColor);      // Sensor Reading Code
            telemetry.addData("Sensor Left: ", leftColor);        //
            telemetry.addData("Sensor Middle: ", middleColor);    //
            telemetry.update();                                   //


            if (middleColor < colorThreshold) { // aligned perfectly
                foundLine = true;
                stopMotors();
            } else if (leftColor < colorThreshold) {  // aligned too right
                strafeLeftFor(2, 0.2);
            } else if (rightColor < colorThreshold) { // aligned too left
                strafeRightFor(2, 0.2);
            } else {                                  // unknown alignment
//                strafeRightFor(2, 0.2);
//
//                if (unknownState) {
//                    strafeLeftFor(3+i, 0.2);
//                    unknownState = false;
//                } else {
//                    strafeRightFor(3+i, 0.2);
//                    unknownState = true;
//                }

                while (leftColor < colorThreshold && rightColor < colorThreshold) {
                    strafeRightFor(1, 0.3);
                    leftColor = bottomLeftSensor.getVoltage();     //
                    rightColor = bottomRightSensor.getVoltage();   //
                }

            }
            i++;
        }

        // we have found the line
        forwardFor(2, 0.3);
    }

    public void chooseBeacon() throws InterruptedException {
        // chooses which beacon to go to
        // assuming we are on red team
        telemetry.addData("Status: ", "Choosing Beacon");
        telemetry.update();



        if(!beaconChosen) {
            try {
                grabFrame();
                beaconChosen = true;
            } catch (NullPointerException npe) {
                npe.printStackTrace();
            }
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
        forwardFor(6, 0.3);
    }

    /*-----------------------------------------------------------------------
    | MOVEMENT FUNCTIONS
    | NOTE: Yellow wire is in front for encoders
    *-----------------------------------------------------------------------*/
    public void forwardFor(int inches, double speed) throws InterruptedException {
        encoderMove(inches, inches, -inches, -inches, speed, false, false);
    }

    public void rightFor(int inches, double speed) throws InterruptedException {
        encoderMove(inches, inches, inches, inches, speed, false, false);
    }

    public void leftFor(int inches, double speed) throws InterruptedException {
        encoderMove(inches, inches, inches, inches, speed, false, false);
    }

    public void strafeLeftFor(int inches, double speed) throws InterruptedException {
        encoderMove(-inches, inches, -inches, inches, speed, true, false);
    }

    public void strafeRightFor(int inches, double speed) throws InterruptedException {
        encoderMove(inches, -inches, inches, -inches, speed, true, true);

    }

    public void encoderMove(int leftFront,int leftBack,int rightFront,int rightBack, double speed, boolean isStrafeing, boolean isRight) throws InterruptedException{
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ElapsedTime time = new ElapsedTime();
        time.reset();
        float axisSum = 0;
        int count =0;
        while(time.milliseconds() < 1000) {
            axisSum += axisY;
            count++;
        }
        float averageY = axisSum / count;

        float initRotation = averageY;

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
        leftFrontMotor.setPower(Math.abs(speed * speedFactor));
        leftBackMotor.setPower(Math.abs(speed));
        rightFrontMotor.setPower(Math.abs(speed * speedFactor));
        rightBackMotor.setPower(Math.abs(speed));

        gyro.resetZAxisIntegrator();
        while (leftFrontMotor.isBusy() && leftBackMotor.isBusy() && rightFrontMotor.isBusy() && rightBackMotor.isBusy()) {
//        ElapsedTime newtime = new ElapsedTime();
//        newtime.reset();
//        //while(newtime.milliseconds() < 3000) {
            int leftFrontCurrent = leftFrontMotor.getCurrentPosition();
            int rightFrontCurrent = rightFrontMotor.getCurrentPosition();
            int leftBackCurrent = leftBackMotor.getCurrentPosition();
            int rightBackCurrent = rightBackMotor.getCurrentPosition();

            int leftFrontDifference = Math.abs(leftFrontMotor.getTargetPosition()) - Math.abs(leftFrontCurrent);
            int rightFrontDifference = Math.abs(rightFrontMotor.getTargetPosition()) - Math.abs(rightFrontCurrent);
            int leftBackDifference = Math.abs(leftBackMotor.getTargetPosition()) - Math.abs(leftBackCurrent);
            int rightBackDifference = Math.abs(rightBackMotor.getTargetPosition()) - Math.abs(rightBackCurrent);
            int zIntegration = 666;
            idle();
            if(isStrafeing) {
                zIntegration = gyro.getIntegratedZValue();
                if (isRight) {
//                    if (initRotation < axisY - rotationThreshold) {
//                        speedFactor = 0.3f / (axisY - initRotation);
//                    } else if (initRotation > axisY + rotationThreshold) {
//                        speedFactor = 1.7f * (initRotation - axisY);
//                    }

                    if (zIntegration < 0) {
                        speedFactor = LITTLE_VALUE;
                    } else {
                        speedFactor = BIGGER_VALUE;
                    }
                } else {
//                    if (initRotation < axisY - rotationThreshold) {
//                        speedFactor = 1.7f * (axisY - initRotation);
//                    } else if (initRotation > axisY + rotationThreshold) {
//                        speedFactor = 0.3f / (initRotation - axisY);
//                    }

                    if (zIntegration < 0) {
                        speedFactor = BIGGER_VALUE;
                    } else {
                        speedFactor = LITTLE_VALUE;
                    }
                }
            }

            leftFrontMotor.setPower(Math.abs(speed * speedFactor));
            leftBackMotor.setPower(Math.abs(speed));
            rightFrontMotor.setPower(Math.abs(speed * speedFactor));
            rightBackMotor.setPower(Math.abs(speed));

            telemetry.addData("Difference Left Front: ", leftFrontDifference);
            telemetry.addData("Speed Left Front: ", leftFrontMotor.getPower());
            telemetry.addLine();

            telemetry.addData("Difference Left Back: ", leftBackDifference);
            telemetry.addData("Speed Left Back: ", leftBackMotor.getPower());
            telemetry.addLine();

            telemetry.addData("Difference Right Front: ", rightFrontDifference);
            telemetry.addData("Speed Right Front: ", rightFrontMotor.getPower());
            telemetry.addLine();

            telemetry.addData("Difference Right Back: ", rightBackDifference);
            telemetry.addData("Speed Right Back: ", rightBackMotor.getPower());
            telemetry.addLine();

            if (rightBackDifference < 200
                    || leftBackDifference < 200
                    || rightFrontDifference < 200
                    || leftFrontDifference < 200) {
                leftFrontMotor.setPower(leftFrontMotor.getPower() - 0.10);
                leftBackMotor.setPower(leftBackMotor.getPower() - 0.10);
                rightFrontMotor.setPower(rightFrontMotor.getPower() - 0.10);
                rightBackMotor.setPower(rightBackMotor.getPower() - 0.10);
            }

            if (rightBackDifference < 40
                    || leftBackDifference < 40
                    || rightFrontDifference < 40
                    || leftFrontDifference < 40) {
                stopMotors();
                break;
            }
            telemetry.addData("Z Integration: ", zIntegration);
            telemetry.addData("Factor: ", speedFactor);
            telemetry.update();
        }
        telemetry.update();

        //stopMotors();

//        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

    @Override
    public void onSensorChanged(SensorEvent event) {
        axisX = event.values[0];
        axisY = event.values[1];
        axisZ = event.values[2];

        axisZ = (float) Math.toDegrees(Math.asin(axisZ)*2);
        axisY = (float) Math.toDegrees(Math.asin(axisY)*2);
        axisX = (float) Math.toDegrees(Math.asin(axisX)*2);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

}