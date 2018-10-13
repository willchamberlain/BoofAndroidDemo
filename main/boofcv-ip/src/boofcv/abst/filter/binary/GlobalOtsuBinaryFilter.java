/*
 * Copyright (c) 2011-2017, Peter Abeles. All Rights Reserved.
 *
 * This file is part of BoofCV (http://boofcv.org).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package boofcv.abst.filter.binary;

import boofcv.alg.filter.binary.GThresholdImageOps;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.ImageGray;
import boofcv.struct.image.ImageType;

/**
 * Computes a threshold using Otsu's equation.
 *
 * @see GThresholdImageOps#computeOtsu(ImageGray, int, int)
 *
 * @author Peter Abeles
 */
public class GlobalOtsuBinaryFilter<T extends ImageGray<T>> implements InputToBinary<T> {

	ImageType<T> inputType;

	boolean down;
	int minValue;
	int maxValue;

	/**
	 * @see GThresholdImageOps#computeOtsu
	 */
	public GlobalOtsuBinaryFilter(int minValue, int maxValue,
								  boolean down, ImageType<T> inputType) {
		this.minValue = minValue;
		this.maxValue = maxValue;
		this.down = down;
		this.inputType = inputType;
	}

	@Override
	public void process(T input, GrayU8 output) {
		double threshold = GThresholdImageOps.computeOtsu(input,minValue,maxValue);
		GThresholdImageOps.threshold(input,output,threshold,down);
	}

	@Override
	public int getHorizontalBorder() {
		return 0;
	}

	@Override
	public int getVerticalBorder() {
		return 0;
	}

	@Override
	public ImageType<T> getInputType() {
		return inputType;
	}

	@Override
	public ImageType<GrayU8> getOutputType() {
		return ImageType.single(GrayU8.class);
	}
}
