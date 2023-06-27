package org.firstinspires.ftc.teamcode.drive.Misc;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;
import java.util.Objects;

public final class CustomPipeline extends OpenCvPipeline
{
	private static class Pixel
	{
		Pixel()
		{
			r = 0;
			g = 0;
			b = 0;
			a = 0;
		}
		Pixel(int _r, int _g, int _b, int _a)
		{
			r = _r;
			g = _g;
			b = _b;
			a = _a;
		}
		Pixel(double[] data)
		{
			r = data[0];
			g = data[1];
			b = data[2];
			a = data[3];
		}

		public final double[] GetData()
		{
			return new double[]{ r, g, b, a };
		}

		public double r, g, b, a;
	}

	public final static class Detection
	{
		public enum Color
		{
			None, Red, Blue, Unknown
		}

		public enum Shape
		{
			None, Cube, Sphere, Unknown
		}

		Detection()
		{
			Reset();
		}

		Detection(Color _color, Shape _shape)
		{
			color = _color;
			shape = _shape;
		}

		public void Reset()
		{
			color = Color.None;
			shape = Shape.None;
		}

		Color color;
		Shape shape;
	}

	private Detection _lastDetection = new Detection();
	private boolean _debugMode = true;
	private boolean _pauseDetection = false;
	private int _approximation = 4;

	@Override
	public Mat processFrame(Mat image)
	{
		if (_pauseDetection)
			return image;

		Detection detection = new Detection();

		Mat processed = new Mat(image.rows() / _approximation, image.cols() / _approximation, image.type());

		long redArea = 0, blueArea = 0;
		for (int i = 0; i < image.height(); i += _approximation)
		{
			for (int j = 0; j < image.width(); j += _approximation)
			{
				Pixel pixel = new Pixel(image.get(i, j));

				if (pixel.g > 150 || pixel.g > pixel.r && pixel.g > pixel.b) // ignore too much green or green dominant
				{
					processed.put(i / _approximation, j / _approximation, 0, 0, 0, 255);
					continue;
				}

				if (IsGray(pixel)) // ignore shades of gray
				{
					processed.put(i / _approximation, j / _approximation, 0, 0, 0, 255);
					//CullPixels(image, i, j, _approximation, 0, 0, 0, 255);
					continue;
				}

				if (pixel.r > pixel.g && pixel.r > pixel.b) // red is dominant
				{
					processed.put(i / _approximation, j / _approximation, 255, 0, 0, 255);
					//CullPixels(image, i, j, _approximation, 255, 0, 0, 255);
					redArea++;
				}
				else if (pixel.b > pixel.r && pixel.b > pixel.g) // blue is dominant
				{
					processed.put(i / _approximation, j / _approximation, 0, 0, 255, 255);
					//CullPixels(image, i, j, _approximation, 0, 0, 255, 255);
					blueArea++;
				}
			}
		}

		if (redArea > blueArea)
			detection.color = Detection.Color.Red;
		else
			detection.color = Detection.Color.Blue;


		double[] bounds = FindBounds(processed);
		double boundingBoxArea = (bounds[1] - bounds[0]) * (bounds[3] - bounds[2]);
		double areaRatio = (detection.color == Detection.Color.Red ? redArea : blueArea) / boundingBoxArea;
		if (areaRatio < 0.85) // a bit more than pi/4
			detection.shape = Detection.Shape.Sphere;
		else
			detection.shape = Detection.Shape.Cube;

		if (_debugMode)
			Imgproc.rectangle(processed, new Point(bounds[0], bounds[2]), new Point(bounds[1], bounds[3]), new Scalar((detection.shape == Detection.Shape.Cube ? 255 : 0), 255, (detection.shape == Detection.Shape.Sphere ? 255 : 0), 255), 1);

		_lastDetection = detection;

		return processed;
	}

	public Detection GetLastDetection() { return _lastDetection; }

	public int GetApproximation() { return _approximation; }
	public void SetApproximation(int approx) { _approximation = approx; }

	public boolean GetDebug() { return _debugMode; }
	public void SetDebug(boolean debug) { _debugMode = debug; }

	public void Pause() { _pauseDetection = true; }
	public void Unpause() { _pauseDetection = false; }

	private boolean IsGray(final Pixel pixel)
	{
		double min = Math.min(Math.min(pixel.r, pixel.g), pixel.b);
		double max = Math.max(Math.max(pixel.r, pixel.g), pixel.b);

		return (max - min) < 40;
	}

	private double[] FindBounds(Mat image)
	{
		double[] bounds = new double[4];
		bounds[0] = image.width();
		bounds[2] = image.height();

		Pixel pixel = new Pixel((_lastDetection.color == Detection.Color.Red ? 255 : 0), 0, (_lastDetection.color == Detection.Color.Blue ? 255 : 0), 255);

		for (int i = 0; i < image.height(); i++)
		{
			for (int j = 0; j < image.width(); j++)
			{
				if (!Arrays.equals(image.get(i, j), pixel.GetData()))
					continue;

				boolean alone = true;
				for (int ii = -1; ii <= 1 && alone; ii++)
					for (int jj = -1; jj <= 1 && alone; jj++)
						if (!Arrays.equals(image.get(i, j), image.get((jj == 0 ? ii : 0), (ii == 0 ? jj : 0))))
							alone = false;
				if (alone)
					continue;

				if (j < bounds[0])
					bounds[0] = j;
				else if (j > bounds[1])
					bounds[1] = j;

				if (i < bounds[2])
					bounds[2] = i;
				else if (i > bounds[3])
					bounds[3] = i;
			}
		}

		return bounds;
	}

	private void CullPixels(Mat image, int i, int j, int noPixels, double... data)
	{
		if (!_debugMode)
			return;

		for (int ii = i; ii < i + noPixels; ii++)
			for (int jj = j; jj < j + noPixels; jj++)
				image.put(ii, jj, data);
	}
}