package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Autonomous(name = "RED HUMAN SIDE", group = "Autonomous")
public class redhuamnauto extends LinearOpMode {

    private DcMotorEx intakeMotor;
    private DcMotorEx transferMotor;
    private DcMotorEx outakeMotorLeft;
    private DcMotorEx outakeMotorRight;
    private CRServo servoMovement;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("====== INITIALIZING ======");
        telemetry.update();

        // Initialize motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer_motor_left");
        transferMotor.setDirection(DcMotor.Direction.FORWARD);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outakeMotorLeft = hardwareMap.get(DcMotorEx.class, "outake_motor_left");
        outakeMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        outakeMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outakeMotorRight = hardwareMap.get(DcMotorEx.class, "outake_motor_right");
        outakeMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servoMovement = hardwareMap.get(CRServo.class, "intake_servo");

        Pose2d startPose = new Pose2d(69, 20, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addData("Starting Pose", "X: 69, Y: 20, Heading: 180°");
        telemetry.addLine("✓ Initialized");
        telemetry.update();

        // ===== BUILD MOVEMENT =====
        Action movement = drive.actionBuilder(startPose)
                .splineTo(new Vector2d(-10, 10), Math.toRadians(-45))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addLine("\n========== RUNNING ==========");
        telemetry.update();

        try {
            // ===== PHASE 1: Servo + Intake (2 seconds) =====

            // ===== PHASE 2: Movement =====
            telemetry.addLine("[PHASE 2] Moving to location");
            telemetry.update();

            Actions.runBlocking(movement);

            telemetry.addLine("✓ Phase 2 Complete");
            telemetry.update();

            // ===== PHASE 3: Motors Phase 1 (5 seconds) =====
            telemetry.addLine("[PHASE 3] Running all motors (5s)");
            telemetry.update();

            // servoMovement.setPower(1.0);
            // intakeMotor.setPower(-1.0);
            transferMotor.setPower(-1.0);
            outakeMotorLeft.setPower(-0.7);
            outakeMotorRight.setPower(0.7);
            intakeMotor.setPower(-1.0);
            sleep(10000);  // 15 seconds

            telemetry.addLine("✓ Phase 3 Complete");
            telemetry.update();

            // ===== PHASE 4: Motors Phase 2 (10 seconds) =====
            telemetry.addLine("[PHASE 4] Running transfer + outtake (10s)");
            telemetry.update();

            /* intakeMotor.setPower(0);          // Stop intake
            transferMotor.setPower(-1.0);      // Continue transfer
            outakeMotorLeft.setPower(-0.7);
            outakeMotorRight.setPower(0.7);
            // Outtake already running
            sleep(10000);  // 10 more seconds */

            telemetry.addLine("✓ Phase 4 Complete");
            telemetry.update();

            telemetry.addData("Status", "✓ COMPLETE");

            Pose2d currentPose = drive.localizer.getPose();

            Actions.runBlocking(
                    drive.actionBuilder(currentPose)
                            .splineTo(new Vector2d(60,38), Math.toRadians(0))
                            .build()
            );
        Actions.runBlocking(
          drive.actionBuilder(currentpose
        intakemotor.setPower(-1.0);
        sleep
        transferMotor.setPower(-1.0)
        
        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
        }
        telemetry.update();

        // Stop all motors
        intakeMotor.setPower(0);
        transferMotor.setPower(0);
        outakeMotorLeft.setPower(0);
        outakeMotorRight.setPower(0);

        Pose2d finalPose = drive.localizer.getPose();
        telemetry.addData("Final X", String.format("%.2f", finalPose.position.x));
        telemetry.addData("Final Y", String.format("%.2f", finalPose.position.y));
        telemetry.addData("Final Heading (deg)", String.format("%.2f", Math.toDegrees(finalPose.heading.toDouble())));
        telemetry.addLine("\n========== AUTO COMPLETE ==========");
        telemetry.update();
    }
}
