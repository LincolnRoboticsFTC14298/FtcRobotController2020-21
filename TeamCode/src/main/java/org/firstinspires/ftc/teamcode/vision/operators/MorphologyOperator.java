package org.firstinspires.ftc.teamcode.vision.operators;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;

public class MorphologyOperator implements MatOperator {
    private Size openSize = new Size(15,15);
    private Size closeSize = new Size(30,30);

    public MorphologyOperator() {

    }
    public MorphologyOperator(Size openSize, Size closeSize) {
        this.openSize = openSize;
        this.closeSize = closeSize;
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
        // Don't use close since it makes it harder for segmenter
        // Closes gaps //
        Mat se = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, closeSize);
        Imgproc.morphologyEx(finalMask, finalMask, Imgproc.MORPH_CLOSE, se);
        return finalMask;
    }
}
