package org.firstinspires.ftc.teamcode.vision.operators;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotlib.vision.MatOperator;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

@Config
public class MorphologyOperator implements MatOperator {
    public static Size openSize = new Size(3,3);
    public static Size closeSize = new Size(3,3);
    public static boolean close = true;

    public MorphologyOperator() {

    }
    public MorphologyOperator(Size openSize, Size closeSize) {
        MorphologyOperator.openSize = openSize;
        MorphologyOperator.closeSize = closeSize;
    }

    Mat finalMask = new Mat();
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, openSize);//Mat kernel = Mat.ones(openSize, CvType.CV_32F);
    Mat se = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, closeSize);
    @Override
    public Mat process(Mat mask) {
        // Remove isolated pixels using morphological operations
        // Applying dilate on the Image
        Imgproc.morphologyEx(mask, finalMask, Imgproc.MORPH_OPEN, kernel);

        // Closes gaps //
        if (close) {
            Imgproc.morphologyEx(finalMask, finalMask, Imgproc.MORPH_CLOSE, se);
        }
        return finalMask;
    }

    public void setClose(boolean close) {
        this.close = close;
    }
}
