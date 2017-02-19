package org.firstinspires.ftc.teamcode;

/**
 * Created by FTC2875 on 9/25/2016.
 */

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import ftc.vision.BeaconColorResult;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import ftc.vision.ImageProcessorResult;


import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Autonomous Beacon Red", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class AutoRed extends LinearOpMode implements SensorEventListener {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    /*-----------------------------------------------------------------------
    | Motor Declarations
    *-----------------------------------------------------------------------*/
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;


    /*-----------------------------------------------------------------------
    | Sensor Declarations
    *-----------------------------------------------------------------------*/
    private AnalogInput bottomLeftSensor = null;
    private AnalogInput bottomRightSensor = null;
    private AnalogInput bottomMiddleSensor = null;
    private DeviceInterfaceModule CDI = null;
    private ModernRoboticsI2cGyro gyro = null;
    private ModernRoboticsI2cGyro straightGyro = null;
    private ModernRoboticsI2cRangeSensor range = null;

    /*-----------------------------------------------------------------------
    | Global Variables
    *-----------------------------------------------------------------------*/
    private static final double COLOR_THRESHOLD = 3.6  ;
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

    private static final double LITTLE_VALUE = 0.7;
    private static final double BIGGER_VALUE = 1.25;

    private boolean checkLine = false;
    private boolean checkBeacon = false;
    private boolean realignStraight = false;

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
        range = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        straightGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro 2");

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
        straightGyro.calibrate();
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
            telemetry.addData("raw ultrasonic", range.rawUltrasonic());
            telemetry.addData("raw optical", range.rawOptical());
            telemetry.addData("cm optical", "%.2f cm", range.cmOptical());
            telemetry.addData("cm", "%.2f cm", range.getDistance(DistanceUnit.CM));
            telemetry.update();
            straightGyro.resetZAxisIntegrator();
            goToBeacon();
            beaconLineFollow();
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
        forwardFor(25, 0.5);

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
            telemetry.update();

            checkLine = true;
            strafeLeftFor(100, 0.5);
            checkLine = false;
            foundLine = true;
        }

        // we have found the line
        realign();
        checkBeacon = true;
        forwardFor(10, 0.3);
        checkBeacon = false;
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
            telemetry.addData("Left:", "BLUE");
            telemetry.addData("Right:,", "RED");
            telemetry.update();

            // go to the right and hit red button
            strafeRightFor(3, 0.5);
            stopMotors();
            beaconLoop = true;

        } else if (leftColor.toString().equals("BLUE") && !beaconLoop) {
            // telemetry data
            telemetry.addData("Left:", "RED");
            telemetry.addData("Right:", "BLUE");
            telemetry.update();

            // go to the left and hit red button
            strafeLeftFor(2, 0.5);
            stopMotors();
            beaconLoop = true;
        }

    }

    public void hitBeacon() throws InterruptedException{
        forwardFor(7, 0.4);
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

    public void realign() {
        int angle = straightGyro.getIntegratedZValue();
        realignStraight = true;

        //TODO fix left and right movement
        if (// too far right)
            leftFor(10, 0.3);
        else if (// too far left)
            rightFor(10, 0.3);

        realignStraight = false;
    }

    public void encoderMove(int leftFront,int leftBack,int rightFront,int rightBack, double speed, boolean isStrafeing, boolean isRight) throws InterruptedException{
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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
                    if (zIntegration < 0) {
                        speedFactor = LITTLE_VALUE;
                    } else {
                        speedFactor = BIGGER_VALUE;
                    }
                } else {
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

            if (rightBackDifference < 30
                    || leftBackDifference < 30
                    || rightFrontDifference < 30
                    || leftFrontDifference < 30) {
                stopMotors();
                break;
            }

            if (checkLine) {
                double middle = bottomMiddleSensor.getVoltage();
                telemetry.addData("Color: ", middle);
                telemetry.update();
                if (middle < COLOR_THRESHOLD)
                    break;
            }

            if (checkBeacon) {
                double distance = range.getDistance(DistanceUnit.CM);
                telemetry.addData("Distance: ", distance);
                telemetry.update();
                if (distance < 23)
                    break;
            }

            if (realignStraight) {
                int angle = straightGyro.getIntegratedZValue();
                telemetry.addData("Angle: ", angle);
                telemetry.update();
                if (angle == 0)
                    break;
            }

            telemetry.addData("Z Integration: ", zIntegration);
            telemetry.addData("Factor: ", speedFactor);
            telemetry.update();
        }
        telemetry.update();
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