package org.boofcv.android.recognition;

import android.support.annotation.NonNull;

import boofcv.abst.fiducial.*;
import boofcv.factory.fiducial.ConfigFiducialBinary;
import boofcv.factory.fiducial.FactoryFiducial;
import boofcv.factory.filter.binary.ConfigThreshold;
import boofcv.factory.filter.binary.ThresholdType;
import boofcv.struct.image.GrayU8;

/**
 * Created by will on 4/04/17.
 */

public class FiducialDetectionManager {


    public static final double TARGET_WIDTH_OF_FIDUCIAL_MARKER_SIZE = 0.135; //0.257;

    @NonNull
    static boofcv.abst.fiducial.FiducialDetector<GrayU8> createGrayU8FiducialDetector(int binaryThreshold, Object lock, boolean robust) {
        boofcv.abst.fiducial.FiducialDetector<GrayU8> detector;
        ConfigFiducialBinary config = new ConfigFiducialBinary(TARGET_WIDTH_OF_FIDUCIAL_MARKER_SIZE);

        synchronized (lock) {
            ConfigThreshold configThreshold;
            if (robust) {
                configThreshold = ConfigThreshold.local(ThresholdType.LOCAL_SQUARE, 6);
            } else {
                configThreshold = ConfigThreshold.fixed(binaryThreshold);
            }
            detector = FactoryFiducial.squareBinary(config, configThreshold, GrayU8.class);
        }

        return detector;
    }


}
