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
import boofcv.alg.misc.GImageMiscOps;
import boofcv.core.image.GeneralizedImageOps;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.ImageGray;
import boofcv.struct.image.ImageType;
import boofcv.testing.BoofTesting;
import org.junit.Test;

import java.util.Random;

/**
 * @author Peter Abeles
 */
public class TestLocalSauvolaBinaryFilter {
	Random rand = new Random(234);

	@Test
	public void compare() {
		Class imageTypes[] = new Class[]{GrayU8.class,GrayF32.class};

		for( Class type : imageTypes ) {

			ImageGray input = GeneralizedImageOps.createSingleBand(type, 30, 40);
			GrayU8 found = new GrayU8(30,40);
			GrayU8 expected = new GrayU8(30,40);

			GImageMiscOps.fillUniform(input, rand, 0, 200);

			LocalSauvolaBinaryFilter alg = new LocalSauvolaBinaryFilter(4,0.3f,true, ImageType.single(type));

			alg.process(input,found);
			GThresholdImageOps.localSauvola(input, expected, 4, 0.3f, true);

			BoofTesting.assertEquals(found, expected, 0);
		}
	}
}