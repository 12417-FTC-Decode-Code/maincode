/*
Copyright 2024 FIRST Tech Challenge

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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="MecanumHolonomicDrive", group="TeleOp")
public class MecanumDrive extends LinearOpMode {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    private DcMotor intakeMotor;
    private DcMotor transferMotorLeft;
    private DcMotor outakeMotor;

    @Override
    public void runOpMode() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        transferMotorLeft = hardwareMap.get(DcMotor.class, "transfer_motor_left");
        outakeMotor = hardwareMap.get(DcMotor.class, "outake_motor");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.  // Note: pushing stick forward gives negative value
            // Axial (forward/backward) and lateral (strafing) from right stick
            // Movement (forward/backward and strafing) from left joystick
            // Movement from left stick Y and X
            // Movement from left stick Y and X
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            
            // Rotation from right stick X
            double yaw = gamepad1.right_stick_x;
            
            // Right stick Y (no movement, only used for boost detection)
            double rightStickY = -gamepad1.right_stick_y;
            
            // Telemetry to debug joystick values and boost
            telemetry.addData("Axial (left stick Y)", axial);
            telemetry.addData("Right Stick Y", rightStickY);
            
            boolean boost = (axial > 0.5) && (rightStickY > 0.5);
            
            telemetry.addData("Boost active?", boost);
            telemetry.update();
            
            // Calculate motor powers without boost affecting rotation
            double frontLeftPower  = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower   = axial - lateral + yaw;
            double backRightPower  = axial + lateral - yaw;
            
            // Apply boost only to forward/strafing motion (axial+lateral), rotation stays normal
            if (boost) {
                frontLeftPower  = (axial + lateral) * 1.5 + yaw;
                frontRightPower = (axial - lateral) * 1.5 - yaw;
                backLeftPower   = (axial - lateral) * 1.5 + yaw;
                backRightPower  = (axial + lateral) * 1.5 - yaw;
            }
            
            // Normalize powers if above 1.0
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));
            
            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }
            
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);


            // Show the elapsed game time and wheel power.
            
        }
    }
}










