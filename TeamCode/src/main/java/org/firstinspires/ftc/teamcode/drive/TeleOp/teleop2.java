package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(group = "Driving")
public class teleop2 extends LinearOpMode {

    SampleMecanumDrive mecanumDrive;
    DcMotorEx liftMotor1, liftMotor2, plateMotor;
    Servo catcher;
    double suppress1;
    RevColorSensorV3 sensor;
    int pp = 0, cp1 = 0, cp2 = 0;
    double suppressRotate;

    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        initialization();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            run();
        }
    }

    private void initialization() {
        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        catcher = hardwareMap.get(Servo.class, "catcherServo");
        sensor = hardwareMap.get(RevColorSensorV3.class, "sensor");
        sensor.enableLed(true);
        catcher.setPosition(0);
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        plateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanumDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void controlWheels() {
        Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y * suppress1,
                -gamepad1.left_stick_x * suppress1
        ).rotated(-poseEstimate.getHeading());
        mecanumDrive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -(gamepad1.right_trigger - gamepad1.left_trigger) * suppressRotate
                )
        );
    }

    boolean standing = true;

    private void resetArmLocalization() {
        if (gamepad2.right_stick_button) {
            liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }


    boolean lastPressedCatch = false;









































































    private void controlCatcher() {
        executeCurrentMoveTarget();
        MoveTarget currentTarget;
        boolean left_bumper1_pressed = gamepad1.left_bumper;
        if (left_bumper1_pressed && !lastPressedCatch) {
            if (catcher.getPosition() == 0) {
                catcher.setPosition(.4f);
            } else {
                catcher.setPosition(0);
                resetTargets();
                if (down) {
                    currentTarget = new MoveTarget(liftMotor1, -300);
                    moveTargets.add(currentTarget);
                    down = false;
                }
            }
        }
        lastPressedCatch = left_bumper1_pressed;
    }


    private void suppressWheels() {
        if (gamepad1.right_bumper) {
            suppress1 = 0.5f;
            suppressRotate = 0.5f;
        } else {
            suppress1 = 1f;
            suppressRotate = 1f;
        }
    }

    private void autonomousArm() {
        if (gamepad2.back) {
            pp = 1423;
            plateMotor.setTargetPosition(1423);
            plateMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
    }

    private void colorDetect() {
        if (catcher.getPosition() == 0f)
            if (sensor.red() > sensor.blue() && sensor.red() > sensor.green() && sensor.getDistance(DistanceUnit.METER) < 0.045)
                catcher.setPosition(.4f);
    }

    private void run() {
        resetArmLocalization();
        suppressWheels();
        controlWheels();
        controlArm();
        controlCatcher();
        debugTelemetry();
        //autonomousArm();
        relax();
        ///colorDetect();
    }

    private void debugTelemetry() {
        telemetry.addData("lift1", liftMotor1.getCurrentPosition());
        telemetry.addData("lift2", liftMotor2.getCurrentPosition());
        telemetry.addData("plate", plateMotor.getCurrentPosition());
        telemetry.addData("claw", catcher.getPosition());
        telemetry.addLine("Colors ").addData("red", sensor.red()).addData("green", sensor.green()).addData("blue", sensor.blue());
        telemetry.addData("colorDistance", sensor.getDistance(DistanceUnit.METER));
        telemetry.addData("motorPower1", liftMotor1.getPower());
        telemetry.addData("motorBusy1", liftMotor1.isBusy());
        telemetry.addData("motorPower2", liftMotor2.getPower());
        telemetry.addData("motorBusy2", liftMotor2.isBusy());
        telemetry.addData("plateBusy1", plateMotor.isBusy());
        telemetry.addData("platePower", plateMotor.getPower());
        telemetry.update();
    }

    private class MoveTarget {
        private DcMotorEx motor;
        private int position;

        public MoveTarget(DcMotorEx motor, int position) {
            this.motor = motor;
            this.position = position;
        }

        public DcMotorEx getMotor() {
            return motor;
        }

        public int getPosition() {
            return position;
        }
    }

    boolean down = false;

    private void executeCurrentMoveTarget() {
        if (moveTargets.isEmpty()) return;

        MoveTarget moveTarget = moveTargets.peek();
        DcMotorEx motor = moveTarget.getMotor();

        if (motor == liftMotor1) {
            DcMotorEx motor2 = liftMotor2;
            motor.setTargetPosition(moveTarget.getPosition());
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
            motor2.setTargetPosition(moveTarget.getPosition());
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setPower(1);
        } else {
            motor.setTargetPosition(moveTarget.getPosition());
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);
        }

        if (runtime.milliseconds() > 200) {//800
            moveTargets.remove();
            runtime.reset();
        }
    }

    Queue<MoveTarget> moveTargets = new LinkedList<>();

    private void resetTargets() {
        moveTargets.clear();
        runtime.reset();
    }

    private void controlArm() {
        executeCurrentMoveTarget();
        MoveTarget currentTarget;
        if (gamepad2.a) {
            resetTargets();
            currentTarget = new MoveTarget(plateMotor, 0);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(liftMotor1, 0);
            moveTargets.add(currentTarget);
        } else if (gamepad2.dpad_left) {
            resetTargets();
            currentTarget = new MoveTarget(liftMotor1, -1272);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, -1000);
            moveTargets.add(currentTarget);
        } else if (gamepad2.dpad_right) {
            resetTargets();
            currentTarget = new MoveTarget(liftMotor1, -770);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, -1000);
            moveTargets.add(currentTarget);
        } else if (gamepad2.dpad_up) {
            resetTargets();
            currentTarget = new MoveTarget(liftMotor1, -1757);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, -1000);
            moveTargets.add(currentTarget);
        } else if (gamepad2.left_bumper) {
            resetTargets();
            down = true;
            currentTarget = new MoveTarget(liftMotor1, -80);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, -1000);
            moveTargets.add(currentTarget);
        } else if (gamepad2.x) {
            resetTargets();
            currentTarget = new MoveTarget(liftMotor1, -1272);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, 1000);
            moveTargets.add(currentTarget);
        } else if (gamepad2.dpad_down) {
            resetTargets();
            currentTarget = new MoveTarget(plateMotor, 0);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(liftMotor1, 0);
            moveTargets.add(currentTarget);
        } else if (gamepad2.b) {
            resetTargets();
            currentTarget = new MoveTarget(liftMotor1, -770);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, 1000);
            moveTargets.add(currentTarget);
        } else if (gamepad2.y) {
            resetTargets();
            currentTarget = new MoveTarget(liftMotor1, -1757);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, 1000);
            moveTargets.add(currentTarget);
        } else if (gamepad2.right_bumper) {
            resetTargets();
            down = true;
            currentTarget = new MoveTarget(liftMotor1, -80);
            moveTargets.add(currentTarget);
            currentTarget = new MoveTarget(plateMotor, 1000);
            moveTargets.add(currentTarget);
        }
    }

    private void relax() {
            if (moveTargets.isEmpty() && runtime.milliseconds() > 1000) {
                plateMotor.setPower(0);
                liftMotor1.setPower(0.1f);
                liftMotor2.setPower(0.1f);
            }
            if ((liftMotor2.getCurrentPosition() >= -500 || liftMotor1.getCurrentPosition() >= -500) && moveTargets.isEmpty() && runtime.milliseconds() > 1000) {
                liftMotor1.setPower(0);
                liftMotor2.setPower(0);
            } else if ((liftMotor2.getCurrentPosition() >= -800 || liftMotor1.getCurrentPosition() >= -800) && moveTargets.isEmpty() && runtime.milliseconds() > 1000) {
                liftMotor1.setPower(0.05f);
                liftMotor2.setPower(0.05f);
            }
    }
}
