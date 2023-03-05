package org.firstinspires.ftc.teamcode.autonom.Traiectorii;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.AutoPosition;
import org.firstinspires.ftc.teamcode.autonom.AutoUtil;
import org.firstinspires.ftc.teamcode.autonom.AutonomStangaHigh;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TraiectoriiStangaHigh {
    AutonomStangaHigh auto;
    TrajectorySequence deliverPreload,catchCone1,deliverCone1, deliverCone2, catchCone2,catchCone3,catchCone4,catchCone5,deliverCone3,deliverCone4,deliverCone5,park1,park2,park3;

    public TraiectoriiStangaHigh(AutonomStangaHigh auto){
        this.auto = auto;
        initializeTrajectories();
    }
    double MAX_ANG_VEL = Math.toRadians(308.7320082135523);
    double TRACK_WIDTH = 13.2; // in

    public void initializeTrajectories(){
        deliverPreload = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .strafeRight(55)
                .back(6)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-120))
                .addTemporalMarker(.7, () -> auto.adjuster.setPosition(0.8))
                .build();
        catchCone1 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .forward(26.3)
                .addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.plateMotor,-1045))
                .addTemporalMarker(.4, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE5))
                .build();
        deliverCone1 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .back(26.8)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGHother))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-120))
                .addTemporalMarker(.3, () -> auto.adjuster.setPosition(0.8))
                .build();
        catchCone2 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .forward(27.3)
                .addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.plateMotor,-1045))
                .addTemporalMarker(.4, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE4))
                .build();
        deliverCone2 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .back(26.8)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGHaf))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-120))
                .addTemporalMarker(.3, () -> auto.adjuster.setPosition(0.8))
                .build();
        catchCone3 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .turn(Math.toRadians(1.4))
                .forward(27.1)
                .addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.plateMotor,-1045))
                .addTemporalMarker(.4, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE3))
                .build();
        deliverCone3 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .back(26.8)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGHaf))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-120))
                .addTemporalMarker(.3, () -> auto.adjuster.setPosition(0.8))
                .build();
        catchCone4 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .forward(26.9)
                .addTemporalMarker( .1, () -> AutoUtil.platePosition(auto.plateMotor,-1045))
                .addTemporalMarker(.4, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE2))
                .build();
        deliverCone4 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .back(28)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGHaf))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-120))
                .addTemporalMarker(.3, () -> auto.adjuster.setPosition(0.8))
                .build();
        catchCone5 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .forward(28.1)
                .addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.plateMotor,-1045))
                .addTemporalMarker(.4, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
        deliverCone5 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .back(28) /// neagra
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGHaf))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-120))
                .addTemporalMarker(.3, () -> auto.adjuster.setPosition(0.8))
                .build();
        park3 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .back(11)
                .turn(Math.toRadians(90))
                .addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.plateMotor,-1045))
                .addTemporalMarker(2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
        park2 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .forward(12.3)
                .turn(Math.toRadians(90))
                .addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.plateMotor,-1045))
                .addTemporalMarker(1.7, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
        park1 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .forward(35.3)
                .addTemporalMarker(.1, () -> AutoUtil.platePosition(auto.plateMotor,-1745))
                .turn(Math.toRadians(90))
                .addTemporalMarker(1.9, () -> AutoUtil.platePosition(auto.plateMotor,-1045))
                .addTemporalMarker(2.5, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
    }

    public void runAuto(int detected){
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.mecanumDrive.followTrajectorySequence(deliverPreload);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(300);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.adjuster.setPosition(0f);
        auto.mecanumDrive.followTrajectorySequence(catchCone1);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(300);
        auto.mecanumDrive.followTrajectorySequence(deliverCone1);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(250);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.adjuster.setPosition(0f);
        auto.mecanumDrive.followTrajectorySequence(catchCone2);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(300);
        auto.mecanumDrive.followTrajectorySequence(deliverCone2);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(250);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.adjuster.setPosition(0f);
        auto.mecanumDrive.followTrajectorySequence(catchCone3);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(300);
        auto.mecanumDrive.followTrajectorySequence(deliverCone3);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(250);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.adjuster.setPosition(0f);
        auto.mecanumDrive.followTrajectorySequence(catchCone4);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(300);
        auto.mecanumDrive.followTrajectorySequence(deliverCone4);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(250);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.adjuster.setPosition(0f);
        auto.mecanumDrive.followTrajectorySequence(catchCone5);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(150);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(300);
        auto.mecanumDrive.followTrajectorySequence(deliverCone5);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(250);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        auto.adjuster.setPosition(0f);
        if(detected == 2) auto.mecanumDrive.followTrajectorySequence(park2);
        else if(detected == 1) auto.mecanumDrive.followTrajectorySequence(park1);
        else auto.mecanumDrive.followTrajectorySequence(park3);
    }
}
