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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="ShootyAngle", group="Linear Opmode")  // @Autonomous(...) is the other common choice
public class ShootingAngle extends LinearOpMode
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
    private DcMotor shooter = null;
    private DcMotor spinner = null;

    DeviceInterfaceModule CDI = null;


    private boolean spinnerOn = false;
    private boolean shooterOn;

    private boolean normalSpeed = true;

    private double motorSpeedFactor = 1;

    private double spinnerPower = 1.0;

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
        shooter = hardwareMap.dcMotor.get("shooter");
        spinner = hardwareMap.dcMotor.get("spinner");

        CDI = hardwareMap.deviceInterfaceModule.get("Device Interface Module 1");

        shooterOn = false;
        /*-------------------------------------------------------------------------------
        | Set the drive motor directions
        | "Reverse" the motor that runs backwards when connected directly to the battery
        *------------------------------------------------------------------------------*/
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.FORWARD);
        spinner.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        runtime.reset();
        while(opModeIsActive()){
            fun();

            forwardFor(300, 0.5); /////////////////////////
            shooter.setPower(-0.60); //                     // used to be 88
            telemetry.addData("Shooter Power: ", shooter.getPower());
            telemetry.update();
            sleep(1500);           //       Shooting      //
            spinner.setPower(-0.50); //                     //
            sleep(18000);          /////////////////////////

            shooter.setPower(0.0);
            spinner.setPower(0.0);

            break;
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
        stopMotors();
    }

    public void rightFor(long time, double power) throws InterruptedException {
        motorPowerLeft(leftFrontMotor, power);
        motorPowerLeft(leftBackMotor, power);

        sleep(time);
        stopMotors();
    }

    public void leftFor( double power) throws InterruptedException {
        motorPowerRight(rightFrontMotor, power);
        motorPowerRight(rightBackMotor, power);


    }

    public void strafeLeftFor( double power) throws InterruptedException {
        motorPowerLeft(leftBackMotor, power);
        motorPowerLeft(leftFrontMotor, -power);
        motorPowerRight(rightBackMotor, -power);
        motorPowerRight(rightFrontMotor, power);


    }

    public void strafeRightFor( double power) throws InterruptedException {
        motorPowerLeft(leftBackMotor, -power);
        motorPowerLeft(leftFrontMotor, power);
        motorPowerRight(rightBackMotor, power);
        motorPowerRight(rightFrontMotor, -power);


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

}