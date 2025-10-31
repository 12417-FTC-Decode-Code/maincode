package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Auto Example", group = "Autonomous")
public class AutoExample extends LinearOpMode {

    DcMotor leftFront, rightFront, leftBack, rightBack;

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFront = hardwareMap.get(DcMotor.class, "front_left_drive");
        rightFront = hardwareMap.get(DcMotor.class, "front_right_drive");
        leftBack = hardwareMap.get(DcMotor.class, "back_left_drive");
        rightBack = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Set motor directions (depends on robot build)
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for start
        waitForStart();

        if (opModeIsActive()) {
            // Drive forward for 3.9 seconds at half power
            setAllPower(0.5);
            sleep(3900);

            // Stop motors
            setAllPower(0);

            // Stop motors
            setAllPower(0);
        }
    }

    private void setAllPower(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }
}

