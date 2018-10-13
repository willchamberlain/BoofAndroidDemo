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

package boofcv.alg.enhance.impl;

import boofcv.struct.convolve.Kernel2D_F32;
import boofcv.struct.convolve.Kernel2D_S32;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.GrayI;
import boofcv.struct.image.GrayU8;

/**
 * <p>
 * Filter based functions for image enhancement.
 * </p>
 *
 * <p>
 * NOTE: Do not modify.  Automatically generated by {@link GenerateImplEnhanceFilter}.
 * </p>
 *
 * @author Peter Abeles
 */
public class ImplEnhanceFilter {

	public static Kernel2D_S32 kernelEnhance4_I32 = new Kernel2D_S32(3, new int[]{0,-1,0,-1,5,-1,0,-1,0});
	public static Kernel2D_F32 kernelEnhance4_F32 = new Kernel2D_F32(3, new float[]{0,-1,0,-1,5,-1,0,-1,0});
	public static Kernel2D_S32 kernelEnhance8_I32 = new Kernel2D_S32(3, new int[]{-1,-1,-1,-1,9,-1,-1,-1,-1});
	public static Kernel2D_F32 kernelEnhance8_F32 = new Kernel2D_F32(3, new float[]{-1,-1,-1,-1,9,-1,-1,-1,-1});

	public static void sharpenInner4(GrayU8 input , GrayU8 output , int minValue , int maxValue ) {
		for( int y = 1; y < input.height-1; y++ ) {
			int indexIn = input.startIndex + y*input.stride + 1;
			int indexOut = output.startIndex + y*output.stride + 1;

			for( int x = 1; x < input.width-1; x++ , indexIn++,indexOut++) {

				int a = 5*(input.data[indexIn] & 0xFF) - (
						(input.data[indexIn-1] & 0xFF)+(input.data[indexIn+1] & 0xFF) +
								(input.data[indexIn-input.stride] & 0xFF) + (input.data[indexIn+input.stride] & 0xFF));

				if( a > maxValue )
					a = maxValue;
				else if( a < minValue )
					a = minValue;

				output.data[indexOut] = (byte)a;
			}
		}
	}

	public static void sharpenBorder4(GrayU8 input , GrayU8 output , int minValue , int maxValue ) {
		int value;

		int b = input.height-1;

		int indexTop = input.startIndex;
		int indexBottom = input.startIndex + b*input.stride;
		
		for( int x = 0; x < input.width; x++ ) {
			value = 4*safeGet(input,x,0) - (safeGet(input,x-1,0) + safeGet(input,x+1,0) + safeGet(input,x,1));

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexTop++] = (byte)value;

			value = 4*safeGet(input,x,b) - (safeGet(input,x-1,b) + safeGet(input,x+1,b) + safeGet(input,x,b-1));

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexBottom++] = (byte)value;
		}

		b = input.width-1;
		int indexLeft = input.startIndex + input.stride;
		int indexRight = input.startIndex + input.stride + b;

		for( int y = 1; y < input.height-1; y++ ) {
			value = 4*safeGet(input,0,y) - (safeGet(input,1,y) + safeGet(input,0,y-1) + safeGet(input,0,y+1));

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexLeft] = (byte)value;

			value = 4*safeGet(input,b,y) - (safeGet(input,b-1,y) + safeGet(input,b,y-1) + safeGet(input,b,y+1));

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexRight] = (byte)value;
			
			indexLeft += input.stride;
			indexRight += input.stride;
		}
	}

	public static void sharpenInner4(GrayF32 input , GrayF32 output , float minValue , float maxValue ) {
		for( int y = 1; y < input.height-1; y++ ) {
			int indexIn = input.startIndex + y*input.stride + 1;
			int indexOut = output.startIndex + y*output.stride + 1;

			for( int x = 1; x < input.width-1; x++ , indexIn++,indexOut++) {

				float a = 5*(input.data[indexIn] ) - (
						(input.data[indexIn-1] )+(input.data[indexIn+1] ) +
								(input.data[indexIn-input.stride] ) + (input.data[indexIn+input.stride] ));

				if( a > maxValue )
					a = maxValue;
				else if( a < minValue )
					a = minValue;

				output.data[indexOut] = a;
			}
		}
	}

	public static void sharpenBorder4(GrayF32 input , GrayF32 output , float minValue , float maxValue ) {
		float value;

		int b = input.height-1;

		int indexTop = input.startIndex;
		int indexBottom = input.startIndex + b*input.stride;
		
		for( int x = 0; x < input.width; x++ ) {
			value = 4*safeGet(input,x,0) - (safeGet(input,x-1,0) + safeGet(input,x+1,0) + safeGet(input,x,1));

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexTop++] = value;

			value = 4*safeGet(input,x,b) - (safeGet(input,x-1,b) + safeGet(input,x+1,b) + safeGet(input,x,b-1));

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexBottom++] = value;
		}

		b = input.width-1;
		int indexLeft = input.startIndex + input.stride;
		int indexRight = input.startIndex + input.stride + b;

		for( int y = 1; y < input.height-1; y++ ) {
			value = 4*safeGet(input,0,y) - (safeGet(input,1,y) + safeGet(input,0,y-1) + safeGet(input,0,y+1));

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexLeft] = value;

			value = 4*safeGet(input,b,y) - (safeGet(input,b-1,y) + safeGet(input,b,y-1) + safeGet(input,b,y+1));

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexRight] = value;
			
			indexLeft += input.stride;
			indexRight += input.stride;
		}
	}

	public static void sharpenInner8(GrayU8 input , GrayU8 output , int minValue , int maxValue ) {
		for( int y = 1; y < input.height-1; y++ ) {
			int indexIn = input.startIndex + y*input.stride + 1;
			int indexOut = output.startIndex + y*output.stride + 1;

			for( int x = 1; x < input.width-1; x++ , indexIn++,indexOut++) {

				int a11 = input.data[indexIn-input.stride-1] & 0xFF;
				int a12 = input.data[indexIn-input.stride] & 0xFF;
				int a13 = input.data[indexIn-input.stride+1] & 0xFF;
				int a21 = input.data[indexIn-1] & 0xFF;
				int a22 = input.data[indexIn] & 0xFF;
				int a23 = input.data[indexIn+1] & 0xFF;
				int a31 = input.data[indexIn+input.stride-1] & 0xFF;
				int a32 = input.data[indexIn+input.stride] & 0xFF;
				int a33 = input.data[indexIn+input.stride+1] & 0xFF;
				
				int result = 9*a22 - (a11+a12+a13+a21+a23+a31+a32+a33);

				if( result > maxValue )
					result = maxValue;
				else if( result < minValue )
					result = minValue;

				output.data[indexOut] = (byte)result;
			}
		}
	}

	public static void sharpenBorder8(GrayU8 input , GrayU8 output , int minValue , int maxValue ) {
		int value;

		int b = input.height-1;
		int a11,a12,a13,a21,a22,a23,a31,a32,a33;

		int indexTop = input.startIndex;
		int indexBottom = input.startIndex + b*input.stride;

		for( int x = 0; x < input.width; x++ ) {

			a11 = safeGet(input,x-1,-1);
			a12 = safeGet(input,x  ,-1);
			a13 = safeGet(input,x+1,-1);
			a21 = safeGet(input,x-1, 0);
			a22 = safeGet(input,x  , 0);
			a23 = safeGet(input,x+1, 0);
			a31 = safeGet(input,x-1, 1);
			a32 = safeGet(input,x  , 1);
			a33 = safeGet(input,x+1, 1);

			value = 9*a22 - (a11+a12+a13+a21+a23+a31+a32+a33);

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexTop++] = (byte)value;

			a11 = safeGet(input,x-1,b-1);
			a12 = safeGet(input,x  ,b-1);
			a13 = safeGet(input,x+1,b-1);
			a21 = safeGet(input,x-1, b);
			a22 = safeGet(input,x  , b);
			a23 = safeGet(input,x+1, b);
			a31 = safeGet(input,x-1,b+1);
			a32 = safeGet(input,x  ,b+1);
			a33 = safeGet(input,x+1,b+1);

			value = 9*a22 - (a11+a12+a13+a21+a23+a31+a32+a33);

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexBottom++] = (byte)value;
		}

		b = input.width-1;
		int indexLeft = input.startIndex + input.stride;
		int indexRight = input.startIndex + input.stride + b;

		for( int y = 1; y < input.height-1; y++ ) {
			a11 = safeGet(input,-1,y-1);
			a12 = safeGet(input, 0,y-1);
			a13 = safeGet(input,+1,y-1);
			a21 = safeGet(input,-1, y );
			a22 = safeGet(input, 0, y );
			a23 = safeGet(input,+1, y );
			a31 = safeGet(input,-1,y+1);
			a32 = safeGet(input, 0,y+1);
			a33 = safeGet(input,+1,y+1);

			value = 9*a22 - (a11+a12+a13+a21+a23+a31+a32+a33);

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexLeft] = (byte)value;

			a11 = safeGet(input,b-1,y-1);
			a12 = safeGet(input, b ,y-1);
			a13 = safeGet(input,b+1,y-1);
			a21 = safeGet(input,b-1, y );
			a22 = safeGet(input, b , y );
			a23 = safeGet(input,b+1, y );
			a31 = safeGet(input,b-1,y+1);
			a32 = safeGet(input, b ,y+1);
			a33 = safeGet(input,b+1,y+1);

			value = 9*a22 - (a11+a12+a13+a21+a23+a31+a32+a33);

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexRight] = (byte)value;

			indexLeft += input.stride;
			indexRight += input.stride;
		}
	}

	public static void sharpenInner8(GrayF32 input , GrayF32 output , float minValue , float maxValue ) {
		for( int y = 1; y < input.height-1; y++ ) {
			int indexIn = input.startIndex + y*input.stride + 1;
			int indexOut = output.startIndex + y*output.stride + 1;

			for( int x = 1; x < input.width-1; x++ , indexIn++,indexOut++) {

				float a11 = input.data[indexIn-input.stride-1] ;
				float a12 = input.data[indexIn-input.stride] ;
				float a13 = input.data[indexIn-input.stride+1] ;
				float a21 = input.data[indexIn-1] ;
				float a22 = input.data[indexIn] ;
				float a23 = input.data[indexIn+1] ;
				float a31 = input.data[indexIn+input.stride-1] ;
				float a32 = input.data[indexIn+input.stride] ;
				float a33 = input.data[indexIn+input.stride+1] ;
				
				float result = 9*a22 - (a11+a12+a13+a21+a23+a31+a32+a33);

				if( result > maxValue )
					result = maxValue;
				else if( result < minValue )
					result = minValue;

				output.data[indexOut] = result;
			}
		}
	}

	public static void sharpenBorder8(GrayF32 input , GrayF32 output , float minValue , float maxValue ) {
		float value;

		int b = input.height-1;
		float a11,a12,a13,a21,a22,a23,a31,a32,a33;

		int indexTop = input.startIndex;
		int indexBottom = input.startIndex + b*input.stride;

		for( int x = 0; x < input.width; x++ ) {

			a11 = safeGet(input,x-1,-1);
			a12 = safeGet(input,x  ,-1);
			a13 = safeGet(input,x+1,-1);
			a21 = safeGet(input,x-1, 0);
			a22 = safeGet(input,x  , 0);
			a23 = safeGet(input,x+1, 0);
			a31 = safeGet(input,x-1, 1);
			a32 = safeGet(input,x  , 1);
			a33 = safeGet(input,x+1, 1);

			value = 9*a22 - (a11+a12+a13+a21+a23+a31+a32+a33);

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexTop++] = value;

			a11 = safeGet(input,x-1,b-1);
			a12 = safeGet(input,x  ,b-1);
			a13 = safeGet(input,x+1,b-1);
			a21 = safeGet(input,x-1, b);
			a22 = safeGet(input,x  , b);
			a23 = safeGet(input,x+1, b);
			a31 = safeGet(input,x-1,b+1);
			a32 = safeGet(input,x  ,b+1);
			a33 = safeGet(input,x+1,b+1);

			value = 9*a22 - (a11+a12+a13+a21+a23+a31+a32+a33);

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexBottom++] = value;
		}

		b = input.width-1;
		int indexLeft = input.startIndex + input.stride;
		int indexRight = input.startIndex + input.stride + b;

		for( int y = 1; y < input.height-1; y++ ) {
			a11 = safeGet(input,-1,y-1);
			a12 = safeGet(input, 0,y-1);
			a13 = safeGet(input,+1,y-1);
			a21 = safeGet(input,-1, y );
			a22 = safeGet(input, 0, y );
			a23 = safeGet(input,+1, y );
			a31 = safeGet(input,-1,y+1);
			a32 = safeGet(input, 0,y+1);
			a33 = safeGet(input,+1,y+1);

			value = 9*a22 - (a11+a12+a13+a21+a23+a31+a32+a33);

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexLeft] = value;

			a11 = safeGet(input,b-1,y-1);
			a12 = safeGet(input, b ,y-1);
			a13 = safeGet(input,b+1,y-1);
			a21 = safeGet(input,b-1, y );
			a22 = safeGet(input, b , y );
			a23 = safeGet(input,b+1, y );
			a31 = safeGet(input,b-1,y+1);
			a32 = safeGet(input, b ,y+1);
			a33 = safeGet(input,b+1,y+1);

			value = 9*a22 - (a11+a12+a13+a21+a23+a31+a32+a33);

			if( value > maxValue )
				value = maxValue;
			else if( value < minValue )
				value = minValue;

			output.data[indexRight] = value;

			indexLeft += input.stride;
			indexRight += input.stride;
		}
	}

	/**
	 * Handle outside image pixels by extending the image.
	 */
	public static int safeGet(GrayI input , int x , int y ) {
		if( x < 0 )
			x = 0;
		else if( x >= input.width )
			x = input.width-1;
		if( y < 0 )
			y = 0;
		else if( y >= input.height )
			y = input.height-1;

		return input.unsafe_get(x,y);
	}

	/**
	 * Handle outside image pixels by extending the image.
	 */
	public static float safeGet(GrayF32 input , int x , int y ) {
		if( x < 0 )
			x = 0;
		else if( x >= input.width )
			x = input.width-1;
		if( y < 0 )
			y = 0;
		else if( y >= input.height )
			y = input.height-1;

		return input.unsafe_get(x,y);
	}


}
