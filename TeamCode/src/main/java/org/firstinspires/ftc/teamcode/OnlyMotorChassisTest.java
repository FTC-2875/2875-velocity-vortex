/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.List;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MECANUM WHEELS", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class OnlyMotorChassisTest extends LinearOpMode implements SensorEventListener
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // private DcMotor leftMotor = null;
    // private DcMotor rightMotor = null;
    /*-----------------------------------------------------------------------
        | Motor Declarations
        *-----------------------------------------------------------------------*/
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;

    private boolean spinnerOn = false;
    private boolean shooterOn;

    private boolean normalSpeed = true;

    private double motorSpeedFactor = 1;

    private double spinnerPower = 1.0;

    private static SensorManager mSensorManager;
    private static Sensor mSensor;

    private float axisX;
    private float axisY;
    private float axisZ;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
         /*-----------------------------------------------------------------------
        | Motor Hardware Declarations
        *-----------------------------------------------------------------------*/
        leftFrontMotor = hardwareMap.dcMotor.get("left front");
        rightFrontMotor = hardwareMap.dcMotor.get("right front");
        leftBackMotor = hardwareMap.dcMotor.get("left back");
        rightBackMotor = hardwareMap.dcMotor.get("right back");


        shooterOn = false;
        /*-------------------------------------------------------------------------------
        | Set the drive motor directions
        | "Reverse" the motor that runs backwards when connected directly to the battery
        *------------------------------------------------------------------------------*/
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);

        mSensorManager = (SensorManager) hardwareMap.appContext.getSystemService(Context.SENSOR_SERVICE);
        mSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        mSensorManager.registerListener(this, mSensor, SensorManager.SENSOR_DELAY_FASTEST);
        telemetry.addData("Program: ", Math.random() * 10);
        List<Sensor> testList = mSensorManager.getSensorList(Sensor.TYPE_ALL);
        telemetry.addData("Size of testList", testList.size());

        for (int i = 0; i < testList.size(); i++) {
            telemetry.addData(testList.get(i).toString(), "");
            telemetry.addLine();
        }
        telemetry.update();

        waitForStart();
        runtime.reset();

        while(opModeIsActive()){

            if (gamepad1.right_trigger > 0) {
                motorSpeedFactor = 0.1;
                telemetry.addData("Motor Mode: ", "Slow");
            }

            if (gamepad1.right_trigger == 0) {
                motorSpeedFactor = 1.0;
                telemetry.addData("Motor Mode: ", "Normal");
            }

            if (gamepad1.dpad_left){
                strafeLeftFor(0.5);
                telemetry.addData("Strafing: ", "Left");
            } else if(gamepad1.dpad_right) {
                strafeRightFor(0.5);
                telemetry.addData("Strafing: ", "Right");
            } else {
                leftFor(-gamepad1.left_stick_y * motorSpeedFactor);
                rightFor(-gamepad1.right_stick_y * motorSpeedFactor);

                telemetry.addData("Speed: L ", -gamepad1.left_stick_y * motorSpeedFactor);
                telemetry.addData("Speed: R ", -gamepad1.right_stick_y * motorSpeedFactor);
            }

            telemetry.addData("X: ", axisX);
            telemetry.addData("Y: ", axisY);
            telemetry.addData("Z: ", axisZ);
            telemetry.update();
        }


    }


    /*-------------------------------------------------------------------------------
    | Movement Code
    | Code for motor calls, etc.
    *------------------------------------------------------------------------------*/
    public void forwardFor(long time, double power) throws InterruptedException {
        motorPowerRight(rightFrontMotor, power);
        motorPowerLeft(leftFrontMotor, power);
        motorPowerRight(rightBackMotor, power);
        motorPowerLeft(leftBackMotor, power);

        sleep(time);
    }

    public void rightFor( double power) throws InterruptedException {
        motorPowerLeft(leftFrontMotor, power);
        motorPowerLeft(leftBackMotor, power);


    }

    public void leftFor( double power) throws InterruptedException {
        motorPowerRight(rightFrontMotor, power);
        motorPowerRight(rightBackMotor, power);


    }

    public void strafeLeftFor( double power) throws InterruptedException {
        motorPowerLeft(leftBackMotor, power*2);
        motorPowerLeft(leftFrontMotor, -power*0.5);
        motorPowerRight(rightBackMotor, -power);
        motorPowerRight(rightFrontMotor, power);


    }

    public void strafeRightFor( double power) throws InterruptedException {
        motorPowerLeft(leftBackMotor, -power);
        motorPowerLeft(leftFrontMotor, power);
        motorPowerRight(rightBackMotor, power*1.95);
        motorPowerRight(rightFrontMotor, -power*0.45);


    }

    public void motorPowerLeft(DcMotor motor, double power) {
        if (power < 0) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotor.Direction.FORWARD);
        }
        motor.setPower(Math.abs(power));
    }

    public void motorPowerRight(DcMotor motor, double power) {
        if (power < 0) {
            motor.setDirection(DcMotor.Direction.FORWARD);
        } else {
            motor.setDirection(DcMotor.Direction.REVERSE);
        }
        motor.setPower(Math.abs(power));
    }

    public void stopMotors(){
        leftBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }

    @Override
    public void onSensorChanged(SensorEvent event) {

        axisX = event.values[0];
        axisY = event.values[1];
        axisZ = event.values[2];

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    //TODO
    // Make sure name is TeamNumber followed DS / RC
    //
}