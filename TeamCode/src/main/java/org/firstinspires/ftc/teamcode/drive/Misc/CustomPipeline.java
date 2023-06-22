package org.firstinspires.ftc.teamcode.drive.Misc;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.math.BigInteger;

public class CustomPipeline extends OpenCvPipeline
{
    enum Detection
    {
        None,
        Red,
        Blue,
        Unknown
    }

    Detection lastDetection = Detection.None;

    private final Object decimationSync = new Object();

    @Override
    public Mat processFrame(Mat input)
    {
        long rAvg = 0, gAvg = 0, bAvg = 0;


        for (int i = 0; i < input.height(); i++)
        {
            for (int j = 0; j < input.width(); j++)
            {
                double[] pixel = input.get(i, j);

                rAvg += pixel[0];
                gAvg += pixel[1];
                bAvg += pixel[2];
            }
        }

        rAvg /= ((long) input.width() * input.height());
        gAvg /= ((long) input.width() * input.height());
        bAvg /= ((long) input.width() * input.height());

        if (rAvg > 170 && rAvg > bAvg)
            lastDetection = Detection.Red;
        else if (bAvg > 170 && bAvg > rAvg)
            lastDetection = Detection.Blue;


        Imgproc.rectangle(
                input,
                new Point(
                        input.cols()/4f,
                        input.rows()/4f),
                new Point(
                        input.cols()*(3f/4f),
                        input.rows()*(3f/4f)),
                new Scalar((lastDetection == Detection.Red ? 255 : 0), 0, (lastDetection == Detection.Blue ? 255 : 0)), 4);


        return input;
    }

    public Detection GetLastDetection()
    {
        return lastDetection;
    }
}
