/*
Copyright 2024 FIRST Tech Challenge Team 18039

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@TeleOp(name="drive program")

public class Drive extends LinearOpMode {

    HardwarePushbot robot       = new HardwarePushbot(); // use the class created to define a Pushbot's hardware
    @Override
    public void runOpMode() 
    {

        robot.rightFrontDriveWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rightBackDriveWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) 
        {
            telemetry.addData("Status", "Running");
            telemetry.update();
            double forward = - gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            
            //Controls the speed for going across the field; can be changed if going too slow
            //strafe is lateral movement!!! Looking forward while moving sideways
            if(gamepad1.right_bumper)
            {
                forward /= 4;  
                strafe / \= 4;
                turn / = 4;
            }
            
            double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
            
            rightFrontDriveWheel.setPower((forward - strafe - turn) / denominator);
            leftFrontDriveWheel.setPower((forward + strafe + turn) / denominator);
            leftBackDriveWheel.setPower((forward - strafe + turn) / denominator);
            rightBackDriveWheel.setPower((forward + strafe - turn) / denominator);

        }
    }
}
