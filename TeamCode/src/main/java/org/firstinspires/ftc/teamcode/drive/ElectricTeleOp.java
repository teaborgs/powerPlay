package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "Driving")
public class ElectricTeleOp extends LinearOpMode {

    SampleMecanumDrive mecanumDrive;

    DcMotorEx liftMotor, plateMotor;
    Servo catcher;
    ColorSensor sensor;
    double suppress1;
    boolean lr = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            run();
        }
    }

    private void initialization(){
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        catcher = hardwareMap.get(Servo.class, "catcherServo");
        sensor = hardwareMap.get(ColorSensor.class, "sensor");

        catcher.setPosition(0);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        plateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanumDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    private void controlWheels(){
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
        if (gamepad1.start) lr = !lr;
        if (lr) {
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y*suppress1,
                    -gamepad1.left_stick_x*suppress1
            ).rotated(-poseEstimate.getHeading());
            mecanumDrive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -(gamepad1.right_trigger-gamepad1.left_trigger)*suppress1
                    )
            );
        }
        else {
            Vector2d input = new Vector2d(
                    -gamepad1.right_stick_y*suppress1,
                    -gamepad1.right_stick_x*suppress1
            ).rotated(-poseEstimate.getHeading());
            mecanumDrive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -(gamepad1.right_trigger-gamepad1.left_trigger)*suppress1
                    )
            );
        }
    }

    int cp = 0;
    int pp = 0;
    /*private void controlRotations(){
        if(gamepad2.y || gamepad2.b) {
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            cp = liftMotor.getCurrentPosition();
        } else{
            liftMotor.setTargetPosition(cp);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1f);
        }
        if(gamepad2.y)
            liftMotor.setPower(1f);
        if(gamepad2.a)
            plateMotor.setPower(1f);
        else if(gamepad2.x)
            plateMotor.setPower(-1f);
        else
            plateMotor.setPower(0);
    }*/

    boolean stop = false;

    private void controlArm(){
        if(gamepad2.left_stick_y!=0){
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            cp = liftMotor.getCurrentPosition();
            liftMotor.setPower( 1f * gamepad2.left_stick_y);
        }
        else{
            liftMotor.setTargetPosition(cp);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1f);
        }
        if (gamepad2.right_trigger!=0 || gamepad2.left_trigger!=0){
            plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pp = plateMotor.getCurrentPosition();
            plateMotor.setPower ( 1f * (gamepad2.right_trigger-gamepad2.left_trigger));
        }
        else {
            plateMotor.setTargetPosition(pp);
            plateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            plateMotor.setPower(1f);
        }
    }
    private void controlCatcher(){
        if(gamepad1.dpad_left)
            catcher.setPosition(0);
        else if(gamepad1.dpad_right)
            catcher.setPosition(.4f);
    }
    private void setPlateLevel(){
        if(gamepad2.left_bumper){
            pp = - 1000;
            plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            plateMotor.setTargetPosition(-1000);
        }
        else if (gamepad2.right_bumper){
            pp = 1000;
            plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            plateMotor.setTargetPosition(1000);
        }
        else if(gamepad2.dpad_up) {
            pp = 0;
            plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            plateMotor.setTargetPosition(0);
        }
    }
    private void setLiftLevel(){
        if (gamepad2.a){
            cp = 0;
            liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setTargetPosition(0);
            liftMotor.setPower(-1f);
        }
        else if (gamepad2.b){
            cp = -1000;
            liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setTargetPosition(-1000);
            liftMotor.setPower(1f);
        }
        else if (gamepad2.x){
            cp = -1750;
            liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setTargetPosition(-1750);
            liftMotor.setPower(1f);
        }
        else if (gamepad2.y){
            cp = -2600;
            liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setTargetPosition(-2650);
            liftMotor.setPower(1f);
        }

        if(gamepad2.dpad_down){
            cp = -2000;
            liftMotor.setTargetPosition(cp);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1f);
        }
    }

    private void suppressWheels() {
        if(gamepad1.right_bumper)
            suppress1 = 0.5f;
        else
            suppress1 = 1f;
    }

    private void colorDet() {
        if(sensor.red() >= 200 && sensor.blue() <= 10 && sensor.green() <= 10)
            catcher.setPosition(.4f);
    }

    private void run(){
        suppressWheels();
        setPlateLevel();
        controlWheels();
        setLiftLevel();
        controlArm();
        controlCatcher();
        debugTelemetry();
        colorDet();
    }

    private void debugTelemetry(){
        telemetry.addData("lift", liftMotor.getCurrentPosition());
        telemetry.addData("plate", plateMotor.getCurrentPosition());
        telemetry.addData("claw",catcher.getPosition());
        telemetry.addData("red", sensor.red());
        telemetry.addData("blue", sensor.blue());
        telemetry.addData("green", sensor.green());
        telemetry.addData("green", sensor.argb());

        telemetry.update();
    }

}
