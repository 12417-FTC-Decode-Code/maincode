package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Config
@Autonomous(name = "Vision Auto with Motor Control", group = "Autonomous")
public class rrmsoa extends LinearOpMode {

    @Config
    public static class VisionConfig {
        public static int CAMERA_TIMEOUT_MS = 3000;
        public static int TAG_SCAN_TIMEOUT_MS = 4000;
    }

    public class MotorSubsystem {
        private final DcMotor intakeMotor;
        private final DcMotor transferMotorLeft;
        private final DcMotor outakeMotorLeft;
        private final DcMotor outakeMotorRight;

        public MotorSubsystem(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
            transferMotorLeft = hardwareMap.get(DcMotor.class, "transfer_motor_left");
            outakeMotorLeft = hardwareMap.get(DcMotor.class, "outake_motor_left");
            outakeMotorRight = hardwareMap.get(DcMotor.class, "outake_motor_right");
        }

        public class RunIntakeAction implements Action {
            private final double power;
            private final double durationMillis;
            private long startTime = -1;

            public RunIntakeAction(double power, double durationSeconds) {
                this.power = power;
                this.durationMillis = durationSeconds * 1000;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime < 0) {
                    startTime = System.currentTimeMillis();
                    intakeMotor.setPower(power);
                    packet.put("Intake Motor", "RUNNING at " + power);
                }

                long elapsed = System.currentTimeMillis() - startTime;

                if (elapsed >= durationMillis) {
                    intakeMotor.setPower(0);
                    packet.put("Intake Motor", "STOPPED");
                    return false;
                }

                packet.put("Intake Time Remaining (s)",
                        String.format("%.1f", (durationMillis - elapsed) / 1000.0));
                return true;
            }
        }

        public class RunTransferAction implements Action {
            private final double power;
            private final double durationMillis;
            private long startTime = -1;

            public RunTransferAction(double power, double durationSeconds) {
                this.power = power;
                this.durationMillis = durationSeconds * 1000;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime < 0) {
                    startTime = System.currentTimeMillis();
                    transferMotorLeft.setPower(power);
                    packet.put("Transfer Motor", "RUNNING at " + power);
                }

                long elapsed = System.currentTimeMillis() - startTime;

                if (elapsed >= durationMillis) {
                    transferMotorLeft.setPower(0);
                    packet.put("Transfer Motor", "STOPPED");
                    return false;
                }

                packet.put("Transfer Time Remaining (s)",
                        String.format("%.1f", (durationMillis - elapsed) / 1000.0));
                return true;
            }
        }

        public class RunOuttakeAction implements Action {
            private final double durationMillis;
            private long startTime = -1;

            public RunOuttakeAction(double durationSeconds) {
                this.durationMillis = durationSeconds * 1000;
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime < 0) {
                    startTime = System.currentTimeMillis();
                    outakeMotorLeft.setPower(-1.0);
                    outakeMotorRight.setPower(1.0);
                    packet.put("Outtake Motors", "RUNNING");
                }

                long elapsed = System.currentTimeMillis() - startTime;

                if (elapsed >= durationMillis) {
                    outakeMotorLeft.setPower(0);
                    outakeMotorRight.setPower(0);
                    packet.put("Outtake Motors", "STOPPED");
                    return false;
                }

                packet.put("Outtake Time Remaining (s)",
                        String.format("%.1f", (durationMillis - elapsed) / 1000.0));
                return true;
            }
        }

        public void stopAll() {
            intakeMotor.setPower(0);
            transferMotorLeft.setPower(0);
            outakeMotorLeft.setPower(0);
            outakeMotorRight.setPower(0);
        }
    }

    public class VisionProcessor {
        private AprilTagProcessor aprilTag;
        private VisionPortal visionPortal;
        private int detectedTagId = -1;
        private boolean cameraReady = false;

        public VisionProcessor(HardwareMap hardwareMap) {
            try {
                aprilTag = new AprilTagProcessor.Builder()
                        .setDrawAxes(true)
                        .setDrawCubeProjection(true)
                        .setDrawTagID(true)
                        .setDrawTagOutline(true)
                        .build();

                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(aprilTag)
                        .enableLiveView(true)
                        .setLiveViewContainerId(0)
                        .build();

                telemetry.addData("Vision", "✓ Initialized");
                telemetry.update();

            } catch (Exception e) {
                telemetry.addData("Vision ERROR", e.getMessage());
                telemetry.update();
                sleep(2000);
            }
        }

        public void waitForCameraReady() {
            telemetry.addData("Camera Status", "Waiting for camera...");
            telemetry.update();

            long startTime = System.currentTimeMillis();

            while (!isStopRequested() &&
                    visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {

                long elapsed = System.currentTimeMillis() - startTime;

                if (elapsed > VisionConfig.CAMERA_TIMEOUT_MS) {
                    telemetry.addData("Camera Status", "✗ TIMEOUT");
                    telemetry.update();
                    break;
                }

                sleep(50);
            }

            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                cameraReady = true;
                telemetry.addData("Camera Status", "READY");
            } else {
                telemetry.addData("Camera Status", "FAILED");
            }
            telemetry.update();
            sleep(500);
        }

        public class ScanForAprilTag implements Action {
            private long startTime = 0;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime == 0) {
                    startTime = System.currentTimeMillis();
                }

                long elapsedTime = System.currentTimeMillis() - startTime;

                if (!cameraReady) {
                    packet.put("Status", "Camera not ready");
                    return (elapsedTime < VisionConfig.TAG_SCAN_TIMEOUT_MS);
                }

                List<AprilTagDetection> detections = aprilTag.getDetections();
                packet.put("Detections", detections.size());

                if (detections.size() > 0) {
                    AprilTagDetection tag = detections.get(0);

                    if (tag.ftcPose != null) {
                        detectedTagId = tag.id;
                        packet.put("TAG FOUND", tag.id);
                        packet.put("Range", String.format("%.2f", tag.ftcPose.range));
                        return false;
                    }
                }

                packet.put("Scan Time (ms)", elapsedTime);
                packet.put("Status", "SCANNING");

                if (elapsedTime >= VisionConfig.TAG_SCAN_TIMEOUT_MS) {
                    detectedTagId = -1;
                    packet.put("Status", "TIMEOUT");
                    return false;
                }

                return true;
            }
        }

        public Action scanForAprilTag() {
            return new ScanForAprilTag();
        }

        public int getDetectedTagId() {
            return detectedTagId;
        }

        public void shutdown() {
            if (visionPortal != null) {
                visionPortal.close();
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-55, 55, Math.toRadians(-45));

        MecanumDrive drive = null;
        try {
            drive = new MecanumDrive(hardwareMap, initialPose);
            telemetry.addData("MecanumDrive", "✓ Initialized");
        } catch (Exception e) {
            telemetry.addData("ERROR", e.getMessage());
            telemetry.update();
            sleep(3000);
            return;
        }

        MotorSubsystem motors = new MotorSubsystem(hardwareMap);


        VisionProcessor vision = new VisionProcessor(hardwareMap);
        vision.waitForCameraReady();

        Action moveToHalfway = drive.actionBuilder(initialPose)
                .lineToX(-27.5) 
                .build();

        Pose2d halfwayPose = new Pose2d(-27.5, 55, Math.toRadians(-45));

        Action runIntake = motors.new RunIntakeAction(1.0, 2.0);  
        Action runTransfer = motors.new RunTransferAction(1.0, 5.0);  
        Action runOuttake = motors.new RunOuttakeAction(5.0);  

        Action transferAndOuttake = new ParallelAction(
                runTransfer,
                runOuttake
        );

        Action allMotors = new SequentialAction(
                runIntake, 
                transferAndOuttake  
        );

        Action continueToFinal = drive.actionBuilder(halfwayPose)
                .lineToY(-20) 
                .build();

        Pose2d afterMoveForward = new Pose2d(-20, 4, Math.toRadians(0));

        Action pathForTag1 = drive.actionBuilder(afterMoveForward)
                .strafeTo(new Vector2d(-32, 48))
                .build();

        Action pathForTag2 = drive.actionBuilder(afterMoveForward)
                .lineToX(48)
                .build();

        Action pathForTag3 = drive.actionBuilder(afterMoveForward)
                .strafeTo(new Vector2d(-8, 48))
                .build();

        Action pathDefault = drive.actionBuilder(afterMoveForward)
                .turnTo(Math.toRadians(45))
                .lineToY(19)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addLine("Moving to halfway point");
        telemetry.update();

        try {
            Actions.runBlocking(moveToHalfway);
        } catch (Exception e) {
            telemetry.addData("Phase 1 ERROR", e.getMessage());
        }

        try {
            Actions.runBlocking(allMotors);
        } catch (Exception e) {
            telemetry.addData("Phase 2 ERROR", e.getMessage());
        }

        try {
            Actions.runBlocking(continueToFinal);
        } catch (Exception e) {
            telemetry.addData("Phase 3 ERROR", e.getMessage());
        }

        try {
            Actions.runBlocking(vision.scanForAprilTag());
        } catch (Exception e) {
            telemetry.addData("Phase 4 ERROR", e.getMessage());
        }

        int detectedTag = vision.getDetectedTagId();
        Action chosenPath;

        switch (detectedTag) {
            case 1:
                telemetry.addData("Detected Tag", "1 (LEFT)");
                chosenPath = pathForTag1;
                break;

            case 2:
                telemetry.addData("Detected Tag", "2 (CENTER)");
                chosenPath = pathForTag2;
                break;

            case 3:
                telemetry.addData("Detected Tag", "3 (RIGHT)");
                chosenPath = pathForTag3;
                break;

            default:
                telemetry.addData("Detected Tag", "NONE (using default)");
                chosenPath = pathDefault;
                break;
        }
        telemetry.update();
        sleep(500);


        try {
            Actions.runBlocking(chosenPath);
        } catch (Exception e) {
            telemetry.addData("Phase 6 ERROR", e.getMessage());
        }
        telemetry.update();

        motors.stopAll();
        vision.shutdown();

    }

}
