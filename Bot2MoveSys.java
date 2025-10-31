package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Bot2MoveSys {


    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    private DcMotor Arm;
    private Servo trapdoor;

    Orientation angles;

    //numbers
    float armSpeed = 0.4f;
    static final float     COUNTS_PER_MOTOR_REV    = 300f;
    static final float     DRIVE_GEAR_REDUCTION    = 1.0f;
    static final float     WHEEL_DIAMETER_INCHES   = 3.54f;
    static final float     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415f);
    static final float     DRIVE_SPEED             = 0.3f;
    static final float     TURN_SPEED              = 0.9f;
    static final float     ARM_SPEED                = 0.4f;

    //placeholder
    Orientation lastAngles = new Orientation();
    float globalAngle;

    public Bot2MoveSys(HardwareMap hardwareMap) {
        //openCv = new OpticSysOpenCV(hardwareMap);
        //aprilTag = new OpticSysAprilTag(hardwareMap);

//        parameters.loggingEnabled = true;
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//

        FLMotor = hardwareMap.dcMotor.get ("FL_Motor"); //check with driver hub
        FRMotor = hardwareMap.dcMotor.get("FR_Motor"); //check with driver hub
        BLMotor = hardwareMap.dcMotor.get ("BL_Motor"); //check with driver hub
        BRMotor = hardwareMap.dcMotor.get ("BR_Motor"); //check with driver hub


        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);

        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    public int getCurrentTicks()
    {
        return FLMotor.getCurrentPosition();
    }
    public int forward(float inches) {
        int i = (int) (inches*COUNTS_PER_INCH);
        int newFL;
        int newFR;
        int newBL;
        int newBR;

        // Determine new target position, and pass to motor controller
        newFL = FLMotor.getCurrentPosition() + i;
        newFR = FRMotor.getCurrentPosition() + i;
        newBL = BLMotor.getCurrentPosition() + i;
        newBR = BRMotor.getCurrentPosition() + i;

        FLMotor.setTargetPosition(newFL);
        FRMotor.setTargetPosition(newFR);
        BLMotor.setTargetPosition(newBL);
        BRMotor.setTargetPosition(newBR);

        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FLMotor.setPower(DRIVE_SPEED);
        FRMotor.setPower(DRIVE_SPEED);
        BLMotor.setPower(DRIVE_SPEED);
        BRMotor.setPower(DRIVE_SPEED);


            return newFL;
    }
    public int right(float inches) {
        int i = (int) (inches*COUNTS_PER_INCH); //convert inches to ticks
        int newFL;
        int newFR;
        int newBL;
        int newBR;

        // Determine new target position, and pass to motor controller
        newFL = FLMotor.getCurrentPosition() - i;
        newFR = FRMotor.getCurrentPosition() + i;
        newBL = BLMotor.getCurrentPosition() + i;
        newBR = BRMotor.getCurrentPosition() - i;

        //Set values
        FLMotor.setTargetPosition(newFL);
        FRMotor.setTargetPosition(newFR);
        BLMotor.setTargetPosition(newBL);
        BRMotor.setTargetPosition(newBR);


        //Turn on RUN_TO_POSITION
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Set power
        FLMotor.setPower(DRIVE_SPEED);
        FRMotor.setPower(DRIVE_SPEED);
        BLMotor.setPower(DRIVE_SPEED);
        BRMotor.setPower(DRIVE_SPEED);

        return newFL;
    }
    public void right_old(int inches)
    {
        int newFL;
        int newBR;
        int newFR;
        int newBL;


        // Determine new target position, and pass to motor controller
        newFL = FLMotor.getCurrentPosition() -(int)(inches * COUNTS_PER_INCH);
        newBR = BRMotor.getCurrentPosition() -(int)(inches * COUNTS_PER_INCH);

        newFR = FRMotor.getCurrentPosition() +(int)(inches * COUNTS_PER_INCH);
        newBL = BLMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        FLMotor.setTargetPosition(newFL);
        BRMotor.setTargetPosition(newBR);

        FRMotor.setTargetPosition(newFR);
        BLMotor.setTargetPosition(newBL);



        // Turn On RUN_TO_POSITION
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // LMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // RMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // reset the timeout time and start motion.
        //runtime.reset();
        FLMotor.setPower(DRIVE_SPEED);
        FRMotor.setPower(DRIVE_SPEED);
        BLMotor.setPower(DRIVE_SPEED);
        BRMotor.setPower(DRIVE_SPEED);

        // Turn off RUN_TO_POSITION
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    float getAngle()
    {

        float deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;


    }
    public void rotate(int degrees)
    {
//        telemetry.addData("I am working" ,"totally working");
//        telemetry.update();
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        float  leftPower, rightPower;

        resetAngle();

        //if the degrees are less than 0, the robot will turn right
        if (degrees < 0)
        {
            leftPower = -TURN_SPEED;
            rightPower = TURN_SPEED;
        }
        else if (degrees > 0)//if greater than 0, turn left
        {
            leftPower = TURN_SPEED;
            rightPower = -TURN_SPEED;
        }
        else return;

        //sets power o motors with negative signs properly assigned to make the robot go in the correct direction
//
        FLMotor.setPower(leftPower);
        FRMotor.setPower(rightPower);
        BLMotor.setPower(leftPower);
        BRMotor.setPower(rightPower);

//        while(getAngle()!=degrees)
//        {
//            telemetry.addData("hello", getAngle());
//            telemetry.addData("hello", degrees);
//            telemetry.update();
//
//
//        }
//        while(getAngle()<degrees)
//        {
//
//        }


//        while( getAngle()<degrees)
//        {
//            telemetry.addData("hello", getAngle());
//            telemetry.addData("hello", degrees);
//              telemetry.update();
//        }

        if (degrees < 0)
        {

            while ( getAngle() > degrees) {
//                telemetry.addData("angle", getAngle());
//                telemetry.addData("degrees",degrees );
//                telemetry.update();
            }
        }
        else
            while (getAngle() < degrees) {}


        //stop the motors after the angle has been found.

        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);

        //sleep for a bit to make sure the robot doesn't over shoot
        //sleep(3000);
        resetAngle();
    }

    public void resetAngle()
    {

        globalAngle = 0;
    }
//    String formatAngle(AngleUnit angleUnit, double angle) {
//        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
//    }
    //Go to correct tag

    public void placePixel()
    {
        int target = 1500;
        Arm.setTargetPosition(1500);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(Math.abs(armSpeed));
        trapdoor.setPosition(1.0);
        Arm.setPower(0);

    }
    public void returnTrapdoor()
    {
        trapdoor.setPosition(0.0);
    }
    public void getCalibrationStatus()
    {
        return;
    }

    public boolean isMoving()
    {
        return FLMotor.getPowerFloat()&& FRMotor.getPowerFloat();

    }
}
