package org.firstinspires.ftc.teamcode.vision.operators;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robotlib.vision.MatOperator;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

@Config
public class MorphologyOperator implements MatOperator {
    public static Size openSize = new Size(15,15);
    public static Size closeSize = new Size(30,30);

    public MorphologyOperator() {

    }
    public MorphologyOperator(Size openSize, Size closeSize) {
        MorphologyOperator.openSize = openSize;
        MorphologyOperator.closeSize = closeSize;
    }

    @Override
    public Mat process(Mat mask) {
        // Denoising //
        //Imgproc.GaussianBlur(mask, finalMask, new Size(101,101), 10);

        Mat finalMask = new Mat();

        // Remove isolated pixels using morphological operations
        // Preparing the kernel matrix object
        //Mat kernel = Mat.ones(openSize, CvType.CV_32F);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, openSize);
        // Applying dilate on the Image
        Imgproc.morphologyEx(mask, finalMask, Imgproc.MORPH_OPEN, kernel);
        // Closes gaps //
        Mat se = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, closeSize);
        Imgproc.morphologyEx(finalMask, finalMask, Imgproc.MORPH_CLOSE, se);
        return finalMask;
    }
}
