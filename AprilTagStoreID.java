package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.List;
import java.util.Map;

@TeleOp(name="AprilTagStoreID", group="Vision")
public class AprilTagStoreID extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private final Map<Integer,String> tagNames = Map.of(
        20, "BLUE",
        21, "GPP",
        22, "PGP",
        23, "PPG",
        24, "RED"
    );

    private Integer lastTagId = null;
    private boolean written = false;
    private boolean hasMoved = false;






    private static final double DRIVE_POWER = 0.5;
    private static final int TICKS_PER_INCH = 50; // Adjust based on your robot's encoder ratio

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                aprilTag
        );

        telemetry.addLine("AprilTag and Motors ready");
        telemetry.update();
        
        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            
            if (!detections.isEmpty()) {
                AprilTagDetection det = detections.get(0);
                lastTagId = det.id;

                String name = tagNames.getOrDefault(det.id, "UNKNOWN");
                telemetry.addData("Tag ID", det.id);
                telemetry.addData("Tag Name", name);
                
                if (det.ftcPose != null) {
                    telemetry.addData("Range (in)", "%.1f", det.ftcPose.range);
                    telemetry.addData("Bearing (deg)", "%.1f", det.ftcPose.bearing);
                    telemetry.addData("Yaw (deg)", "%.1f", det.ftcPose.yaw);
                }
                
                // Persist once per run
                if (!written) {
                    File f = AppUtil.getInstance().getSettingsFile("lastTagId.txt");
                    ReadWriteFile.writeFile(f, Integer.toString(det.id));
                    File f2 = AppUtil.getInstance().getSettingsFile("lastTagLabel.txt");
                    ReadWriteFile.writeFile(f2, name);
                    written = true;
                }


                if (!hasMoved) {
                    telemetry.addLine("Moving based on detected tag...");
                    telemetry.update();
                    
                    moveBasedOnTagId(det.id);
                    
                    hasMoved = true;
                    telemetry.addLine("Movement complete!");
                }
                
                telemetry.update();
                
            } else {
                telemetry.addLine("No tags visible");
                telemetry.update();
            }
            
            sleep(20);
        }
    }


    private void moveBasedOnTagId(int tagId) {
        switch (tagId) {
            case 20:
                moveForward(0.5);  
                strafeRight(0.5);   
                break;
            case 21: 
                moveForward(0.5);
                strafeRight(0.5);
                break;
            case 22: 
                moveForward(2);
                strafeRight(2);
                break;
            case 23: 
                moveForward(2);
                strafeRight(2);
                break;
            case 24:
                moveForward(2);
                strafeRight(2);
                break;
            default:
                telemetry.addLine("Unknown tag - no movement");
                break;
        }
    }


    private void moveForward(double inches) {
        int targetTicks = (int)(inches * TICKS_PER_INCH);
        
        int flTarget = frontLeft.getCurrentPosition() + targetTicks;
        int frTarget = frontRight.getCurrentPosition() + targetTicks;
        int blTarget = backLeft.getCurrentPosition() + targetTicks;
        int brTarget = backRight.getCurrentPosition() + targetTicks;

        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(DRIVE_POWER);
        frontRight.setPower(DRIVE_POWER);
        backLeft.setPower(DRIVE_POWER);
        backRight.setPower(DRIVE_POWER);

        while (opModeIsActive() && 
               (frontLeft.isBusy() || frontRight.isBusy() || 
                backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("Moving Forward", "%.1f inches", inches);
            telemetry.update();
            idle();
        }

        stopMotors();
        resetToRunMode();
    }

  
    private void strafeRight(double inches) {
        int targetTicks = (int)(inches * TICKS_PER_INCH);
        
        int flTarget = frontLeft.getCurrentPosition() + targetTicks;
        int frTarget = frontRight.getCurrentPosition() - targetTicks;
        int blTarget = backLeft.getCurrentPosition() - targetTicks;
        int brTarget = backRight.getCurrentPosition() + targetTicks;

        frontLeft.setTargetPosition(flTarget);
        frontRight.setTargetPosition(frTarget);
        backLeft.setTargetPosition(blTarget);
        backRight.setTargetPosition(brTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(DRIVE_POWER);
        frontRight.setPower(DRIVE_POWER);
        backLeft.setPower(DRIVE_POWER);
        backRight.setPower(DRIVE_POWER);

        while (opModeIsActive() && 
               (frontLeft.isBusy() || frontRight.isBusy() || 
                backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("Strafing Right", "%.1f inches", inches);
            telemetry.update();
            idle();
        }

        stopMotors();
        resetToRunMode();
    }


    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }


    private void resetToRunMode() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}



