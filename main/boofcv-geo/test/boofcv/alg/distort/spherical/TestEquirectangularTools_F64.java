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

package boofcv.alg.distort.spherical;

import georegression.misc.GrlConstants;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * @author Peter Abeles
 */
public class TestEquirectangularTools_F64 {

	Random rand = new Random(234);
	int width = 300;
	int height = 250;

	/**
	 * Test converting back and forth between equirectangular coordinates and lat-lon using different
	 * centers
	 */
	@Test
	public void equiToLonlat_reverse() {

		EquirectangularTools_F64 tools = new EquirectangularTools_F64();
		tools.configure(width,height);

		equiToLonlat_reverse(tools, width/2, height/2);
		equiToLonlat_reverse(tools, 0, height/2);
		equiToLonlat_reverse(tools, width-1, height/2);
		equiToLonlat_reverse(tools, width/2, 0);
		equiToLonlat_reverse(tools, width/2, height-1);

	}

	private void equiToLonlat_reverse(EquirectangularTools_F64 tools, double x , double y) {
		Point2D_F64 ll = new Point2D_F64();
		Point2D_F64 r = new Point2D_F64();

		tools.equiToLonlat(x,y,ll);
		tools.lonlatToEqui(ll.x,ll.y,r);

		assertEquals(x,r.x, GrlConstants.TEST_F64);
		assertEquals(y,r.y, GrlConstants.TEST_F64);
	}

	@Test
	public void equiToLonlatFV_reverse() {

		EquirectangularTools_F64 tools = new EquirectangularTools_F64();
		tools.configure(width,height);

		equiToLonlatFV_reverse(tools, width/2, height/2);
		equiToLonlatFV_reverse(tools, 0, height/2);
		equiToLonlatFV_reverse(tools, width-1, height/2);
		equiToLonlatFV_reverse(tools, width/2, 0);
		equiToLonlatFV_reverse(tools, width/2, height-1);
	}

	private void equiToLonlatFV_reverse(EquirectangularTools_F64 tools, double x , double y) {
		Point2D_F64 ll = new Point2D_F64();
		Point2D_F64 r = new Point2D_F64();

		tools.equiToLonlatFV(x,y,ll);
		tools.lonlatToEquiFV(ll.x,ll.y,r);

		assertEquals(x,r.x, GrlConstants.TEST_F64);
		assertEquals(y,r.y, GrlConstants.TEST_F64);
	}

	/**
	 * Test one very simple case with a known answer
	 */
	@Test
	public void equiToNorm() {
		EquirectangularTools_F64 tools = new EquirectangularTools_F64();

		tools.configure(300,250);

		Point3D_F64 found = new Point3D_F64();
		tools.equiToNorm(300.0/2.0, 249.0/2.0, found);

		assertEquals(1.0,found.x, GrlConstants.TEST_F64);
		assertEquals(0.0,found.y, GrlConstants.TEST_F64);
		assertEquals(0.0,found.z, GrlConstants.TEST_F64);

		tools.equiToNorm(0, 249/2.0, found);
		assertTrue(found.distance(new Point3D_F64(-1,0,0)) <= GrlConstants.TEST_F64);
		tools.equiToNorm(300, 249/2.0, found);
		assertTrue(found.distance(new Point3D_F64(-1,0,0)) <= GrlConstants.TEST_F64);

		tools.equiToNorm(300/4, 249/2.0, found);
		assertTrue(found.distance(new Point3D_F64(0,-1,0)) <= GrlConstants.TEST_F64);
		tools.equiToNorm(3*300/4, 249/2.0, found);
		assertTrue(found.distance(new Point3D_F64(0, 1,0)) <= GrlConstants.TEST_F64);

		tools.equiToNorm(300/2, 0, found);
		assertTrue(found.distance(new Point3D_F64(0,0, 1)) <= GrlConstants.TEST_F64);
		tools.equiToNorm(300/2, 249, found);
		assertTrue(found.distance(new Point3D_F64(0,0, -1)) <= GrlConstants.TEST_F64);
	}

	@Test
	public void equiToNormFV() {
		EquirectangularTools_F64 tools = new EquirectangularTools_F64();

		tools.configure(300,250);

		Point3D_F64 found = new Point3D_F64();
		tools.equiToNormFV(300/2, 249/2.0, found);

		assertEquals(1.0,found.x, GrlConstants.TEST_F64);
		assertEquals(0.0,found.y, GrlConstants.TEST_F64);
		assertEquals(0.0,found.z, GrlConstants.TEST_F64);

		tools.equiToNormFV(0, 249/2.0, found);
		assertTrue(found.distance(new Point3D_F64(-1,0,0)) <= GrlConstants.TEST_F64);
		tools.equiToNormFV(300, 249/2.0, found);
		assertTrue(found.distance(new Point3D_F64(-1,0,0)) <= GrlConstants.TEST_F64);

		tools.equiToNormFV(300/4, 249/2.0, found);
		assertTrue(found.distance(new Point3D_F64(0,-1,0)) <= GrlConstants.TEST_F64);
		tools.equiToNormFV(3*300/4, 249/2.0, found);
		assertTrue(found.distance(new Point3D_F64(0, 1,0)) <= GrlConstants.TEST_F64);

		tools.equiToNormFV(300/2, 0, found);
		assertTrue(found.distance(new Point3D_F64(0,0, -1)) <= GrlConstants.TEST_F64);
		tools.equiToNormFV(300/2, 249, found);
		assertTrue(found.distance(new Point3D_F64(0,0, 1)) <= GrlConstants.TEST_F64);
	}

	@Test
	public void equiToNorm_reverse() {

		EquirectangularTools_F64 tools = new EquirectangularTools_F64();
		tools.configure(width,height);

		equiToNorm_reverse(tools, width/2, (height-1)/2);
		equiToNorm_reverse(tools, 0, height/2);
		equiToNorm_reverse(tools, width-1, (height-1)/2);
		equiToNorm_reverse(tools, width/2, 1); // pathological cases at extreme.  many to one mapping
		equiToNorm_reverse(tools, width/2, height-2);

		for (int i = 0; i < 100; i++) {
			int x = rand.nextInt(width);
			int y = rand.nextInt(height-1)+1; // avoid pathological case

			equiToNorm_reverse(tools,x,y);
		}

	}

	private void equiToNorm_reverse(EquirectangularTools_F64 tools, double x , double y) {
		Point3D_F64 n = new Point3D_F64();
		Point2D_F64 r = new Point2D_F64();

		tools.equiToNorm(x,y,n);
		tools.normToEqui(n.x,n.y,n.z,r);

		assertEquals(x,r.x, GrlConstants.TEST_F64);
		assertEquals(y,r.y, GrlConstants.TEST_F64);
	}

	@Test
	public void equiToNorm_reverseFV() {

		EquirectangularTools_F64 tools = new EquirectangularTools_F64();
		tools.configure(width,height);

		equiToNorm_reverseFV(tools, width/2, height/2);
		equiToNorm_reverseFV(tools, 0, height/2);
		equiToNorm_reverseFV(tools, width-1, height/2);
		equiToNorm_reverseFV(tools, width/2, 1);  // pathological cases at extreme.  many to one mapping
		equiToNorm_reverseFV(tools, width/2, height-2);

		for (int i = 0; i < 100; i++) {
			int x = rand.nextInt(width);
			int y = rand.nextInt(height-1)+1; // avoid pathological case

			equiToNorm_reverse(tools,x,y);
		}

	}

	private void equiToNorm_reverseFV(EquirectangularTools_F64 tools, double x , double y) {
		Point3D_F64 n = new Point3D_F64();
		Point2D_F64 r = new Point2D_F64();

		tools.equiToNormFV(x,y,n);
		tools.normToEquiFV(n.x,n.y,n.z,r);

		assertEquals(x,r.x, GrlConstants.TEST_F64);
		assertEquals(y,r.y, GrlConstants.TEST_F64);
	}

	@Test
	public void lonlatToEqui() {
		EquirectangularTools_F64 tools = new EquirectangularTools_F64();
		tools.configure(width,height);

		Point2D_F64 found = new Point2D_F64();
		tools.lonlatToEqui(0,GrlConstants.PId2, found);
		assertEquals(width/2,found.x, GrlConstants.TEST_F64);
		assertEquals(height-1,found.y, GrlConstants.TEST_F64);

		tools.lonlatToEqui(0,-GrlConstants.PId2, found);
		assertEquals(width/2,found.x, GrlConstants.TEST_F64);
		assertEquals(0,found.y, GrlConstants.TEST_F64);
	}

	@Test
	public void lonlatToEquiFV() {
		EquirectangularTools_F64 tools = new EquirectangularTools_F64();
		tools.configure(width,height);

		Point2D_F64 found = new Point2D_F64();
		tools.lonlatToEquiFV(0,-GrlConstants.PId2, found);
		assertEquals(width/2,found.x, GrlConstants.TEST_F64);
		assertEquals(height-1,found.y, GrlConstants.TEST_F64);

		tools.lonlatToEquiFV(0,GrlConstants.PId2, found);
		assertEquals(width/2,found.x, GrlConstants.TEST_F64);
		assertEquals(0,found.y, GrlConstants.TEST_F64);
	}
}
