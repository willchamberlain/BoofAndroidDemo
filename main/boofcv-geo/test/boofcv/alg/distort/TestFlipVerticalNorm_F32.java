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

package boofcv.alg.distort;

import boofcv.struct.distort.Point2Transform2_F32;
import georegression.struct.point.Point2D_F32;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * @author Peter Abeles
 */
public class TestFlipVerticalNorm_F32 {

	@Test
	public void theSuperDuperTest() {
		
		FlipVerticalNorm2_F32 alg = new FlipVerticalNorm2_F32(new Dummy(),1);
		
		Point2D_F32 out = new Point2D_F32();
		alg.compute(2,3,out);
		
		assertEquals(2,out.x,1e-8);
		assertEquals(3,-out.y,1e-8);
	}
	
	private class Dummy implements Point2Transform2_F32 {

		@Override
		public void compute(float x, float y, Point2D_F32 out) {
			out.x = x;
			out.y = y;
		}
	}
}
