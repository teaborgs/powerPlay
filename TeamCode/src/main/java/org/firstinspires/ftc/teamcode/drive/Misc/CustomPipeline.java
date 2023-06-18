package org.firstinspires.ftc.teamcode.drive.Misc;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class CustomPipeline extends OpenCvPipeline
{
    enum Detection
    {
        //
    }

    Detection lastDetection;

    @Override
    public Mat processFrame(Mat input)
    {

        // TODO: detectie

        for (int i = 0; i < input.cols(); i++)
            for (int j = 0; j < input.rows(); j++)
                x = input.get(i, j);


        Imgproc.rectangle(
                input,
                new Point(
                        input.cols()/4,
                        input.rows()/4),
                new Point(
                        input.cols()*(3f/4f),
                        input.rows()*(3f/4f)),
                new Scalar(0, 255, 0), 4);

        return input;
    }

    public Detection GetLastDetection()
    {
        return lastDetection;
    }
}
