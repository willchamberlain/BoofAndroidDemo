package org.boofcv.android.recognition;

import boofcv.abst.fiducial.FiducialDetector;
import boofcv.struct.image.GrayU8;

/**
 * Detects and shows square binary fiducials
 *
 * @author Peter Abeles
 */
public class FiducialSquareBinaryActivity extends FiducialSquareActivity
{

	public FiducialSquareBinaryActivity() {
		super(FiducialSquareBinaryHelpActivity.class);
		System.out.println("FiducialSquareBinaryActivity()");
	}

	@Override
	protected FiducialDetector<GrayU8> createDetector() {
		System.out.println("FiducialSquareBinaryActivity: createDetector()");
		return FiducialDetectionManager.createGrayU8FiducialDetector(binaryThreshold, lock, robust);
	}

}
