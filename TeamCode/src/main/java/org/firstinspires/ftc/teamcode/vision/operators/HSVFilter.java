package org.firstinspires.ftc.teamcode.vision.operators;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class HSVFilter implements MatOperator {
    private Scalar lowerThreshold;
    private Scalar upperThershold;
    private Size blur = new Size(30,30);
    private final Size closeSize = new Size(5,5), openSize = new Size(5,5);

    public HSVFilter(Scalar lowerThreshold, Scalar upperThreshold) {
        this.lowerThreshold = lowerThreshold;
        this.upperThershold = upperThreshold;
    }
    public HSVFilter(Scalar lowerThreshold, Scalar upperThreshold, Size blur) {
        this.lowerThreshold = lowerThreshold;
        this.upperThershold = upperThreshold;
        this.blur = blur;
    }

    @Override
    public Mat process(Mat img) {
        Mat mask = new Mat();
        Imgproc.cvtColor(img, mask, Imgproc.COLOR_BGR2HSV);

        Imgproc.GaussianBlur(mask, mask, blur, 0);

        Core.inRange(mask, lowerThreshold, upperThershold, mask);


        // Denoising //
        //Imgproc.GaussianBlur(mask, mask, new Size(101,101), 10);

        // Closes gaps //
        Mat se = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, closeSize);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, se);

        // Remove isolated pixels using morphological operations
        // Preparing the kernel matrix object
        Mat kernel = Mat.ones(openSize, CvType.CV_32F);
        // Applying dilate on the Image
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        return mask;
    }
}
