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

package boofcv.alg.filter.derivative.impl;

import boofcv.alg.filter.derivative.CompareDerivativeToConvolution;
import boofcv.alg.filter.derivative.GradientPrewitt;
import boofcv.alg.misc.ImageMiscOps;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.GrayS16;
import boofcv.struct.image.GrayU8;
import org.junit.Test;

import java.util.Random;


/**
 * @author Peter Abeles
 */
public class TestGradientPrewitt_Shared {
	Random rand = new Random(234);

	int width = 20;
	int height = 25;


	@Test
	public void compareToConvolve_I8() throws NoSuchMethodException {
		CompareDerivativeToConvolution validator = new CompareDerivativeToConvolution();
		validator.setTarget(GradientPrewitt_Shared.class.getMethod("process",
				GrayU8.class, GrayS16.class, GrayS16.class ));

		validator.setKernel(0, GradientPrewitt.kernelDerivX_I32);
		validator.setKernel(1,GradientPrewitt.kernelDerivY_I32);

		GrayU8 input = new GrayU8(width,height);
		ImageMiscOps.fillUniform(input, rand, 0, 10);
		GrayS16 derivX = new GrayS16(width,height);
		GrayS16 derivY = new GrayS16(width,height);

		validator.compare(false,input,derivX,derivY);
	}

	@Test
	public void compareToConvolve_I16() throws NoSuchMethodException {
		CompareDerivativeToConvolution validator = new CompareDerivativeToConvolution();
		validator.setTarget(GradientPrewitt_Shared.class.getMethod("process",
				GrayS16.class, GrayS16.class, GrayS16.class ));

		validator.setKernel(0, GradientPrewitt.kernelDerivX_I32);
		validator.setKernel(1,GradientPrewitt.kernelDerivY_I32);

		GrayS16 input = new GrayS16(width,height);
		ImageMiscOps.fillUniform(input, rand, 0, 10);
		GrayS16 derivX = new GrayS16(width,height);
		GrayS16 derivY = new GrayS16(width,height);

		validator.compare(false,input,derivX,derivY);
	}

	@Test
	public void compareToConvolve_F32() throws NoSuchMethodException {
		CompareDerivativeToConvolution validator = new CompareDerivativeToConvolution();
		validator.setTarget(GradientPrewitt_Shared.class.getMethod("process",
				GrayF32.class, GrayF32.class, GrayF32.class ));

		validator.setKernel(0, GradientPrewitt.kernelDerivX_F32);
		validator.setKernel(1,GradientPrewitt.kernelDerivY_F32);

		GrayF32 input = new GrayF32(width,height);
		ImageMiscOps.fillUniform(input, rand, 0, 10);
		GrayF32 derivX = new GrayF32(width,height);
		GrayF32 derivY = new GrayF32(width,height);

		validator.compare(false,input,derivX,derivY);
	}
}
