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

import boofcv.alg.geo.PerspectiveOps;
import boofcv.struct.calib.CameraPinhole;
import boofcv.struct.calib.CameraPinholeRadial;
import boofcv.struct.distort.Point2Transform2_F32;
import boofcv.struct.distort.Point2Transform2_F64;
import georegression.struct.affine.Affine2D_F32;
import georegression.struct.affine.Affine2D_F64;
import georegression.struct.point.Point2D_F32;
import georegression.struct.point.Point2D_F64;
import georegression.struct.shapes.RectangleLength2D_F32;
import georegression.struct.shapes.RectangleLength2D_F64;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * @author Peter Abeles
 */
public class TestLensDistortionOps {

	Point2D_F32 pf = new Point2D_F32();
	Point2D_F64 pd = new Point2D_F64();
	int width = 300;
	int height = 350;

	@Test
	public void transformChangeModel_F32_NONE_modified() {
		// distorted pixel in original image
		float pixelX = 12.5f,pixelY = height-3;

		CameraPinholeRadial orig = new CameraPinholeRadial().
				fsetK(300, 320, 0, 150, 130, width, height).fsetRadial(0.1, 0.05);
		CameraPinhole desired = new CameraPinhole(orig);

		Point2Transform2_F32 distToNorm = LensDistortionOps.narrow(orig).undistort_F32(true, false);

		Point2D_F32 norm = new Point2D_F32();
		distToNorm.compute(pixelX, pixelY, norm);

		CameraPinhole adjusted = new CameraPinhole();
		Point2Transform2_F32 distToAdj = LensDistortionOps.
				transformChangeModel_F32(AdjustmentType.NONE, orig, desired, false, adjusted);

		Point2D_F32 adjPixel = new Point2D_F32();
		Point2D_F32 normFound = new Point2D_F32();
		distToAdj.compute(pixelX,pixelY,adjPixel);

		PerspectiveOps.convertPixelToNorm(adjusted, adjPixel, normFound);

		// see if the normalized image coordinates are the same
		assertEquals(norm.x, normFound.x,1e-3);
		assertEquals(norm.y, normFound.y, 1e-3);
	}

	/**
	 * Checks the border of the returned transform.  Makes sure that the entire original image is visible.
	 * Also makes sure that the requested inverse transform is actually the inverse.
	 */
	@Test
	public void transformChangeModel_F32_FULLVIEW() {
		CameraPinholeRadial param = new CameraPinholeRadial().
				fsetK(300, 320, 0, 150, 130, width, height).fsetRadial(0.1, 0.05);
		CameraPinhole desired = new CameraPinhole(param);

		Point2Transform2_F32 adjToDist = LensDistortionOps.transformChangeModel_F32(
				AdjustmentType.FULL_VIEW, param, desired, true, null);
		Point2Transform2_F32 distToAdj = LensDistortionOps.transformChangeModel_F32(
				AdjustmentType.FULL_VIEW, param, desired, false, null);

		checkBorderOutside(adjToDist,distToAdj);

		param = new CameraPinholeRadial().
				fsetK(300, 320, 0, 150, 130, width, height).fsetRadial(-0.1,-0.05);
		desired = new CameraPinhole(param);
		adjToDist = LensDistortionOps.transformChangeModel_F32(AdjustmentType.FULL_VIEW, param,desired, true, null);
		distToAdj = LensDistortionOps.transformChangeModel_F32(AdjustmentType.FULL_VIEW, param,desired, false, null);
		checkBorderOutside(adjToDist,distToAdj);
	}

	/**
	 * Checks to see if the returned modified model is correct
	 */
	@Test
	public void transformChangeModel_F32_FULLVIEW_modified() {
		// distorted pixel in original image
		float pixelX = 12.5f,pixelY = height-3;

		CameraPinholeRadial orig = new CameraPinholeRadial().
				fsetK(300, 320, 0, 150, 130, width, height).fsetRadial(0.1, 0.05);
		CameraPinhole desired = new CameraPinhole(orig);

		Point2Transform2_F32 distToNorm = LensDistortionOps.narrow(orig).undistort_F32(true, false);

		Point2D_F32 norm = new Point2D_F32();
		distToNorm.compute(pixelX, pixelY, norm);

		CameraPinholeRadial adjusted = new CameraPinholeRadial();
		Point2Transform2_F32 distToAdj = LensDistortionOps.
				transformChangeModel_F32(AdjustmentType.FULL_VIEW, orig, desired, false, adjusted);

		Point2D_F32 adjPixel = new Point2D_F32();
		Point2D_F32 normFound = new Point2D_F32();
		distToAdj.compute(pixelX,pixelY,adjPixel);

		PerspectiveOps.convertPixelToNorm(adjusted, adjPixel, normFound);

		// see if the normalized image coordinates are the same
		assertEquals(norm.x, normFound.x,1e-3);
		assertEquals(norm.y, normFound.y, 1e-3);
	}

	/**
	 * Checks the border of the returned transform.  Makes sure that no none-visible portion is visible.
	 * Also makes sure that the requested inverse transform is actually the inverse.
	 */
	@Test
	public void transformChangeModel_F32_EXPAND() {
		CameraPinholeRadial param =
				new CameraPinholeRadial().fsetK(300, 320, 0, 150, 130, width, height).fsetRadial(0.1, 1e-4);
		CameraPinhole desired = new CameraPinhole(param);

		Point2Transform2_F32 adjToDist = LensDistortionOps.transformChangeModel_F32(AdjustmentType.EXPAND, param,desired, true, null);
		Point2Transform2_F32 distToAdj = LensDistortionOps.transformChangeModel_F32(AdjustmentType.EXPAND, param,desired, false, null);
		checkInside(adjToDist, distToAdj);

		// distort it in the other direction
		param = new CameraPinholeRadial().fsetK(300, 320, 0, 150, 130, width, height).fsetRadial(-0.1,-1e-4);

		adjToDist = LensDistortionOps.transformChangeModel_F32(AdjustmentType.EXPAND, param, desired, true, null);
		distToAdj = LensDistortionOps.transformChangeModel_F32(AdjustmentType.EXPAND, param, desired, false, null);

		checkInside(adjToDist, distToAdj);
	}

	/**
	 * Sees if the adjusted intrinsic parameters is correct but computing normalized image coordinates first
	 * with the original distorted image and then with the adjusted undistorted image.
	 */
	@Test
	public void transformChangeModel_F32_EXPAND_modified() {

		// distorted pixel in original image
		float pixelX = 12.5f,pixelY = height-3;

		CameraPinholeRadial orig = new CameraPinholeRadial().
				fsetK(300, 320, 0, 150, 130, width, height).fsetRadial(0.1, 0.05);
		CameraPinhole desired = new CameraPinhole(orig);

		Point2Transform2_F32 distToNorm = LensDistortionOps.narrow(orig).undistort_F32(true, false);

		Point2D_F32 norm = new Point2D_F32();
		distToNorm.compute(pixelX, pixelY, norm);

		CameraPinholeRadial adjusted = new CameraPinholeRadial();
		Point2Transform2_F32 distToAdj = LensDistortionOps.transformChangeModel_F32(AdjustmentType.EXPAND, orig, desired, false, adjusted);

		Point2D_F32 adjPixel = new Point2D_F32();
		Point2D_F32 normFound = new Point2D_F32();
		distToAdj.compute(pixelX,pixelY,adjPixel);

		PerspectiveOps.convertPixelToNorm(adjusted, adjPixel, normFound);

		// see if the normalized image coordinates are the same
		assertEquals(norm.x, normFound.x, 1e-3);
		assertEquals(norm.y, normFound.y, 1e-3);
	}

	private void checkBorderOutside(Point2Transform2_F32 tran, Point2Transform2_F32 tranInv) {
		for( int y = 0; y < height; y++ ) {
			checkBorderOutside(0, y, tran, tranInv);
			checkBorderOutside(width - 1, y, tran, tranInv);
		}

		for( int x = 0; x < width; x++ ) {
			checkBorderOutside(x, 0, tran, tranInv);
			checkBorderOutside(x, height - 1, tran, tranInv);
		}
	}

	private void checkBorderOutside(Point2Transform2_F64 distToUndist, Point2Transform2_F64 undistToDist) {
		for( int y = 0; y < height; y++ ) {
			checkBorderOutside(0, y, distToUndist, undistToDist);
			checkBorderOutside(width - 1, y, distToUndist, undistToDist);
		}

		for( int x = 0; x < width; x++ ) {
			checkBorderOutside(x, 0, distToUndist, undistToDist);
			checkBorderOutside(x, height - 1, distToUndist, undistToDist);
		}
	}

	private void checkBorderOutside(int x, int y, Point2Transform2_F32 tran, Point2Transform2_F32 tranInv) {
		tran.compute(x, y, pf);

		float tol = 0.1f;

		String s = x+" "+y+" -> "+ pf.x+" "+ pf.y;
		assertTrue(s,
				pf.x <= 1 + tol || pf.x >= width - 1 - tol ||
						pf.y <= 1 + tol || pf.y >= height - 1 - tol);

		// check the inverse
		tranInv.compute(pf.x, pf.y, pf);

		assertEquals(pf.x,x, 0.01f);
		assertEquals(pf.y,y, 0.01f);
	}

	private void checkBorderOutside(int x, int y, Point2Transform2_F64 tran, Point2Transform2_F64 tranInv) {
		tran.compute(x, y, pd);

		double tol = 0.1;

		String s = x+" "+y+" -> "+ pd.x+" "+ pd.y;
		assertTrue(s, pd.x <= 1 + tol || pd.x >= width - 1 - tol || pd.y <= 1 + tol || pd.y >= height - 1 - tol);

		// check the inverse
		tranInv.compute(pd.x, pd.y, pd);

		assertEquals(pd.x,x, 0.001);
		assertEquals(pd.y,y, 0.001);
	}

	private void checkInside(Point2Transform2_F32 tran, Point2Transform2_F32 tranInv ) {
		double closestT = Double.MAX_VALUE;
		double closestB = Double.MAX_VALUE;

		for( int y = 0; y < height; y++ ) {
			checkInside(0,y,tran,tranInv);
			checkInside(width-1,y,tran,tranInv);

			closestT = Math.min(closestT,distanceEdge(0,y,tran));
			closestB = Math.min(closestB,distanceEdge(width-1,y,tran));
		}

		// should be close to the edge at some point
		assertTrue( closestT < 1 );
		assertTrue( closestB < 1 );

		closestT = closestB = Double.MAX_VALUE;
		for( int x = 0; x < width; x++ ) {
			checkInside(x,0,tran,tranInv);
			checkInside(x,height-1,tran,tranInv);

			closestT = Math.min(closestT,distanceEdge(x,0,tran));
			closestB = Math.min(closestB,distanceEdge(x,height-1,tran));
		}

		// should be close to the edge at some point
		assertTrue(closestT < 1);
		assertTrue(closestB < 1);
	}

	private void checkInside(Point2Transform2_F64 tran, Point2Transform2_F64 tranInv ) {
		double closestT = Double.MAX_VALUE;
		double closestB = Double.MAX_VALUE;

		for( int y = 0; y < height; y++ ) {
			checkInside(0,y,tran,tranInv);
			checkInside(width-1,y,tran,tranInv);

			closestT = Math.min(closestT,distanceEdge(0,y,tran));
			closestB = Math.min(closestB,distanceEdge(width-1,y,tran));
		}

		// should be close to the edge at some point
		assertTrue( closestT < 1 );
		assertTrue( closestB < 1 );

		closestT = closestB = Double.MAX_VALUE;
		for( int x = 0; x < width; x++ ) {
			checkInside(x,0,tran,tranInv);
			checkInside(x,height-1,tran,tranInv);

			closestT = Math.min(closestT,distanceEdge(x,0,tran));
			closestB = Math.min(closestB,distanceEdge(x,height-1,tran));
		}

		// should be close to the edge at some point
		assertTrue(closestT < 1);
		assertTrue(closestB < 1);
	}

	private void checkInside(int x , int y , Point2Transform2_F32 tran , Point2Transform2_F32 tranInv ) {
		tran.compute(x, y, pf);

		float tol = 0.1f;

		String s = x+" "+y+" -> "+ pf.x+" "+ pf.y;
		assertTrue(s, pf.x >= -tol && pf.x < width + tol);
		assertTrue(s, pf.y >= -tol && pf.y < height + tol);

		// check the inverse
		tranInv.compute(pf.x, pf.y, pf);

		assertEquals(pf.x, x, 0.01f);
		assertEquals(pf.x, x, 0.01f);
	}

	private void checkInside(int x , int y , Point2Transform2_F64 tran , Point2Transform2_F64 tranInv ) {
		tran.compute(x, y, pd);

		double tol = 0.1f;

		String s = x+" "+y+" -> "+ pd.x+" "+ pd.y;
		assertTrue(s, pd.x >= -tol && pd.x < width + tol);
		assertTrue(s, pd.y >= -tol && pd.y < height + tol);

		// check the inverse
		tranInv.compute(pd.x, pd.y, pd);

		assertEquals(pd.x, x, 0.01);
		assertEquals(pd.x, x, 0.01);
	}

	private double distanceEdge( int x , int y ,  Point2Transform2_F32 tran ) {
		tran.compute(x, y, pf);

		double min = Double.MAX_VALUE;

		if( x < min ) min = x;
		if( y < min ) min = y;
		if( width-x-1 < min ) min = width-x-1;
		if( height-y-1 < min ) min = height-y-1;

		return min;
	}

	private double distanceEdge( int x , int y ,  Point2Transform2_F64 tran ) {
		tran.compute(x, y, pd);

		double min = Double.MAX_VALUE;

		if( x < min ) min = x;
		if( y < min ) min = y;
		if( width-x-1 < min ) min = width-x-1;
		if( height-y-1 < min ) min = height-y-1;

		return min;
	}

	@Test
	public void boundBoxInside_F32() {
		// basic sanity check
		Affine2D_F32 affine = new Affine2D_F32(1,1,0,1,1,2);
		PixelTransformAffine_F32 transform = new PixelTransformAffine_F32(affine);
		RectangleLength2D_F32 found = LensDistortionOps.boundBoxInside(20, 10, transform);

		assertEquals(10,found.x0,1e-4);
		assertEquals(2 ,found.y0,1e-4);
		assertEquals(20-9,found.width,1e-4);
		assertEquals(10, found.height,1e-4);
	}

	@Test
	public void boundBoxInside_F64() {
		// basic sanity check
		Affine2D_F64 affine = new Affine2D_F64(1,1,0,1,1,2);
		PixelTransformAffine_F64 transform = new PixelTransformAffine_F64(affine);
		RectangleLength2D_F64 found = LensDistortionOps.boundBoxInside(20, 10, transform);

		assertEquals(10,found.x0,1e-8);
		assertEquals(2 ,found.y0,1e-8);
		assertEquals(20-9,found.width,1e-8);
		assertEquals(10, found.height,1e-8);
	}
}
