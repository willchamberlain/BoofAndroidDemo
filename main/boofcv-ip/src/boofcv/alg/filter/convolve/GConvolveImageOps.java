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

package boofcv.alg.filter.convolve;

import boofcv.core.image.border.*;
import boofcv.struct.convolve.*;
import boofcv.struct.image.*;

/**
 * Image type agnostic convolution functions
 */
public class GConvolveImageOps {

	/**
	 * Performs a horizontal 1D convolution across the image.  Borders are handled as specified by the 'border'
	 * parameter.
	 *
	 * @param input The original image. Not modified.
	 * @param output Where the resulting image is written to. Modified.
	 * @param kernel The kernel that is being convolved. Not modified.
	 * @param border How the image borders are handled.
	 */
	public static <In extends ImageBase<In>, Out extends ImageBase<Out>, K extends Kernel1D, B extends ImageBorder<In>>
	void horizontal(K kernel, In input, Out output , B border ) {
		switch( input.getImageType().getFamily() ) {
			case GRAY: {
				if( input instanceof GrayF32) {
					ConvolveWithBorder.horizontal((Kernel1D_F32)kernel,(GrayF32)input,(GrayF32)output,(ImageBorder_F32)border);
				} else if( input instanceof GrayU8) {
					if( GrayI16.class.isAssignableFrom(output.getClass()) )
						ConvolveWithBorder.horizontal((Kernel1D_S32)kernel,(GrayU8)input,(GrayI16)output,(ImageBorder_S32)border);
					else
						ConvolveWithBorder.horizontal((Kernel1D_S32)kernel,(GrayU8)input,(GrayS32)output,(ImageBorder_S32)border);
				} else if( input instanceof GrayS16) {
					ConvolveWithBorder.horizontal((Kernel1D_S32)kernel,(GrayS16)input,(GrayI16)output,(ImageBorder_S32)border);
				} else {
					throw new IllegalArgumentException("Unknown image type: "+input.getClass().getName());
				}
			}break;

			case INTERLEAVED: {
				if( input instanceof InterleavedF32) {
					ConvolveWithBorder.horizontal((Kernel1D_F32)kernel,(InterleavedF32)input,(InterleavedF32)output,(ImageBorder_IL_F32)border);
				} else if( input instanceof InterleavedU8) {
					if( InterleavedI16.class.isAssignableFrom(output.getClass()) )
						ConvolveWithBorder.horizontal((Kernel1D_S32)kernel,(InterleavedU8)input,(InterleavedI16)output,(ImageBorder_IL_S32)border);
					else
						ConvolveWithBorder.horizontal((Kernel1D_S32)kernel,(InterleavedU8)input,(InterleavedS32)output,(ImageBorder_IL_S32)border);
				} else if( input instanceof InterleavedS16) {
					ConvolveWithBorder.horizontal((Kernel1D_S32)kernel,(InterleavedS16)input,(InterleavedU16)output,(ImageBorder_IL_S32)border);
				} else {
					throw new IllegalArgumentException("Unknown image type: "+input.getClass().getName());
				}
			}break;

			case PLANAR: {
				Planar inp = (Planar)input;
				Planar outp = (Planar)output;

				for (int i = 0; i < inp.getNumBands(); i++) {
					horizontal(kernel, inp.getBand(i), outp.getBand(i), (ImageBorder)border);
				}
			}break;
		}
	}

	/**
	 * Performs a vertical 1D convolution across the image.  Borders are handled as specified by the 'border'
	 * parameter.
	 *
	 * @param input The original image. Not modified.
	 * @param output Where the resulting image is written to. Modified.
	 * @param kernel The kernel that is being convolved. Not modified.
	 * @param border How the image borders are handled.
	 */
	public static <In extends ImageBase<In>, Out extends ImageBase<Out>, K extends Kernel1D, B extends ImageBorder<In>>
	void vertical(K kernel, In input, Out output , B border ) {
		switch( input.getImageType().getFamily() ) {
			case GRAY: {
				if (input instanceof GrayF32) {
					ConvolveWithBorder.vertical((Kernel1D_F32) kernel, (GrayF32) input, (GrayF32) output, (ImageBorder_F32) border);
				} else if (input instanceof GrayU8) {
					if (GrayI16.class.isAssignableFrom(output.getClass()))
						ConvolveWithBorder.vertical((Kernel1D_S32) kernel, (GrayU8) input, (GrayI16) output, (ImageBorder_S32) border);
					else
						ConvolveWithBorder.vertical((Kernel1D_S32) kernel, (GrayU8) input, (GrayS32) output, (ImageBorder_S32) border);
				} else if (input instanceof GrayS16) {
					ConvolveWithBorder.vertical((Kernel1D_S32) kernel, (GrayS16) input, (GrayI16) output, (ImageBorder_S32) border);
				} else {
					throw new IllegalArgumentException("Unknown image type: " + input.getClass().getName());
				}
			} break;

			case INTERLEAVED: {
				if( input instanceof InterleavedF32) {
					ConvolveWithBorder.vertical((Kernel1D_F32)kernel,(InterleavedF32)input,(InterleavedF32)output,(ImageBorder_IL_F32)border);
				} else if( input instanceof InterleavedU8) {
					if( InterleavedI16.class.isAssignableFrom(output.getClass()) )
						ConvolveWithBorder.vertical((Kernel1D_S32)kernel,(InterleavedU8)input,(InterleavedI16)output,(ImageBorder_IL_S32)border);
					else
						ConvolveWithBorder.vertical((Kernel1D_S32)kernel,(InterleavedU8)input,(InterleavedS32)output,(ImageBorder_IL_S32)border);
				} else if( input instanceof InterleavedS16) {
					ConvolveWithBorder.vertical((Kernel1D_S32)kernel,(InterleavedS16)input,(InterleavedU16)output,(ImageBorder_IL_S32)border);
				} else {
					throw new IllegalArgumentException("Unknown image type: "+input.getClass().getName());
				}
			}break;

			case PLANAR: {
				Planar inp = (Planar)input;
				Planar outp = (Planar)output;

				for (int i = 0; i < inp.getNumBands(); i++) {
					vertical(kernel, inp.getBand(i), outp.getBand(i), (ImageBorder)border);
				}
			}break;
		}
	}

	/**
	 * Performs a 2D convolution across the image.  Borders are handled as specified by the 'border'
	 * parameter.
	 *
	 * @param input The original image. Not modified.
	 * @param output Where the resulting image is written to. Modified.
	 * @param kernel The kernel that is being convolved. Not modified.
	 * @param border How the image borders are handled.
	 */
	public static <In extends ImageBase<In>, Out extends ImageBase<Out>, K extends Kernel2D, B extends ImageBorder<In>>
	void convolve(K kernel, In input, Out output , B border ) {
		switch( input.getImageType().getFamily() ) {
			case GRAY: {
				if (input instanceof GrayF32) {
					ConvolveWithBorder.convolve((Kernel2D_F32) kernel, (GrayF32) input, (GrayF32) output, (ImageBorder_F32) border);
				} else if (input instanceof GrayU8) {
					if (GrayI16.class.isAssignableFrom(output.getClass()))
						ConvolveWithBorder.convolve((Kernel2D_S32) kernel, (GrayU8) input, (GrayI16) output, (ImageBorder_S32) border);
					else
						ConvolveWithBorder.convolve((Kernel2D_S32) kernel, (GrayU8) input, (GrayS32) output, (ImageBorder_S32) border);
				} else if (input instanceof GrayS16) {
					ConvolveWithBorder.convolve((Kernel2D_S32) kernel, (GrayS16) input, (GrayI16) output, (ImageBorder_S32) border);
				} else {
					throw new IllegalArgumentException("Unknown image type: " + input.getClass().getName());
				}
			} break;

			case INTERLEAVED: {
				if( input instanceof InterleavedF32) {
					ConvolveWithBorder.convolve((Kernel2D_F32)kernel,(InterleavedF32)input,(InterleavedF32)output,(ImageBorder_IL_F32)border);
				} else if( input instanceof InterleavedU8) {
					if( InterleavedI16.class.isAssignableFrom(output.getClass()) )
						ConvolveWithBorder.convolve((Kernel2D_S32)kernel,(InterleavedU8)input,(InterleavedI16)output,(ImageBorder_IL_S32)border);
					else
						ConvolveWithBorder.convolve((Kernel2D_S32)kernel,(InterleavedU8)input,(InterleavedS32)output,(ImageBorder_IL_S32)border);
				} else if( input instanceof InterleavedS16) {
					ConvolveWithBorder.convolve((Kernel2D_S32)kernel,(InterleavedS16)input,(InterleavedU16)output,(ImageBorder_IL_S32)border);
				} else {
					throw new IllegalArgumentException("Unknown image type: "+input.getClass().getName());
				}
			}break;

			case PLANAR: {
				Planar inp = (Planar)input;
				Planar outp = (Planar)output;

				for (int i = 0; i < inp.getNumBands(); i++) {
					convolve(kernel, inp.getBand(i), outp.getBand(i), (ImageBorder)border);
				}
			}break;
		}
	}

	/**
	 * Performs a horizontal 1D convolution across the image.  The horizontal border is not processed.
	 *
	 * @param input	 The original image. Not modified.
	 * @param output Where the resulting image is written to. Modified.
	 * @param kernel The kernel that is being convolved. Not modified.
	 */
	public static <In extends ImageBase<In>, Out extends ImageBase<Out>, K extends Kernel1D>
	void horizontal(K kernel, In input, Out output ) {
		switch (input.getImageType().getFamily()) {
			case GRAY: {
				if (input instanceof GrayF32) {
					ConvolveImageNoBorder.horizontal((Kernel1D_F32) kernel, (GrayF32) input, (GrayF32) output);
				} else if (input instanceof GrayU8) {
					if (GrayI16.class.isAssignableFrom(output.getClass()))
						ConvolveImageNoBorder.horizontal((Kernel1D_S32) kernel, (GrayU8) input, (GrayI16) output);
					else
						ConvolveImageNoBorder.horizontal((Kernel1D_S32) kernel, (GrayU8) input, (GrayS32) output);
				} else if (input instanceof GrayS16) {
					ConvolveImageNoBorder.horizontal((Kernel1D_S32) kernel, (GrayS16) input, (GrayI16) output);
				} else {
					throw new IllegalArgumentException("Unknown image type: " + input.getClass().getName());
				}
			}break;

			case INTERLEAVED: {
				if (output instanceof InterleavedF32) {
					ConvolveImageNoBorder.horizontal((Kernel1D_F32) kernel, (InterleavedF32) input, (InterleavedF32) output);
				} else if (input instanceof InterleavedU8) {
					if (InterleavedI16.class.isAssignableFrom(output.getClass()))
						ConvolveImageNoBorder.horizontal((Kernel1D_S32) kernel, (InterleavedU8) input, (InterleavedI16) output);
					else
						ConvolveImageNoBorder.horizontal((Kernel1D_S32) kernel, (InterleavedU8) input, (InterleavedS32) output);
				} else if (input instanceof InterleavedS16) {
					ConvolveImageNoBorder.horizontal((Kernel1D_S32) kernel, (InterleavedS16) input, (InterleavedI16) output);
				} else {
					throw new IllegalArgumentException("Unknown image type: " + input.getClass().getName());
				}
			}break;

			case PLANAR:{
				Planar inp = (Planar)input;
				Planar outp = (Planar)output;

				for (int i = 0; i < inp.getNumBands(); i++) {
					horizontal(kernel, inp.getBand(i), outp.getBand(i));
				}
			}break;

			default: throw new IllegalArgumentException("Unknown image family");
		}
	}
	/**
	 * Performs a horizontal 1D convolution across the image.  The horizontal border is not processed.
	 *
	 * @param input	 The original image. Not modified.
	 * @param output Where the resulting image is written to. Modified.
	 * @param kernel The kernel that is being convolved. Not modified.
	 */
	public static <In extends ImageBase<In>, Out extends ImageBase<Out>, K extends Kernel1D>
	void vertical(K kernel, In input, Out output ) {
		switch (input.getImageType().getFamily()) {
			case GRAY: {
				if (input instanceof GrayF32) {
					ConvolveImageNoBorder.vertical((Kernel1D_F32) kernel, (GrayF32) input, (GrayF32) output);
				} else if (input instanceof GrayU8) {
					if (GrayI16.class.isAssignableFrom(output.getClass()))
						ConvolveImageNoBorder.vertical((Kernel1D_S32) kernel, (GrayU8) input, (GrayI16) output);
					else
						ConvolveImageNoBorder.vertical((Kernel1D_S32) kernel, (GrayU8) input, (GrayS32) output);
				} else if (input instanceof GrayS16) {
					ConvolveImageNoBorder.vertical((Kernel1D_S32) kernel, (GrayS16) input, (GrayI16) output);
				} else {
					throw new IllegalArgumentException("Unknown image type: " + input.getClass().getName());
				}
			} break;

			case INTERLEAVED: {
				if (output instanceof InterleavedF32) {
					ConvolveImageNoBorder.vertical((Kernel1D_F32) kernel, (InterleavedF32) input, (InterleavedF32) output);
				} else if (input instanceof InterleavedU8) {
					if (InterleavedI16.class.isAssignableFrom(output.getClass()))
						ConvolveImageNoBorder.vertical((Kernel1D_S32) kernel, (InterleavedU8) input, (InterleavedI16) output);
					else
						ConvolveImageNoBorder.vertical((Kernel1D_S32) kernel, (InterleavedU8) input, (InterleavedS32) output);
				} else if (input instanceof InterleavedS16) {
					ConvolveImageNoBorder.vertical((Kernel1D_S32) kernel, (InterleavedS16) input, (InterleavedI16) output);
				} else {
					throw new IllegalArgumentException("Unknown image type: " + input.getClass().getName());
				}
			}break;

			case PLANAR:{
				Planar inp = (Planar)input;
				Planar outp = (Planar)output;

				for (int i = 0; i < inp.getNumBands(); i++) {
					vertical(kernel, inp.getBand(i), outp.getBand(i));
				}
			}break;

			default: throw new IllegalArgumentException("Unknown image family");
		}
	}

	/**
	 * Performs a 2D convolution across the image.  The image's borders are not processed.
	 *
	 * @param kernel A square kernel that will be convolved across the source image
	 * @param input  The source image that is to be convolved
	 * @param output   The results of the convolution
	 */
	public static <In extends ImageBase<In>, Out extends ImageBase<Out>, K extends Kernel2D>
	void convolve(K kernel, In input, Out output ) {
		switch (input.getImageType().getFamily()) {
			case GRAY: {
				if (input instanceof GrayF32) {
					ConvolveImageNoBorder.convolve((Kernel2D_F32) kernel, (GrayF32) input, (GrayF32) output);
				} else if (input instanceof GrayU8) {
					if (GrayI16.class.isAssignableFrom(output.getClass()))
						ConvolveImageNoBorder.convolve((Kernel2D_S32) kernel, (GrayU8) input, (GrayI16) output);
					else
						ConvolveImageNoBorder.convolve((Kernel2D_S32) kernel, (GrayU8) input, (GrayS32) output);
				} else if (input instanceof GrayS16) {
					ConvolveImageNoBorder.convolve((Kernel2D_S32) kernel, (GrayS16) input, (GrayI16) output);
				} else {
					throw new IllegalArgumentException("Unknown image type: " + input.getClass().getName());
				}
			} break;

			case INTERLEAVED: {
				if (output instanceof InterleavedF32) {
					ConvolveImageNoBorder.convolve((Kernel2D_F32) kernel, (InterleavedF32) input, (InterleavedF32) output);
				} else if (input instanceof InterleavedU8) {
					if (InterleavedI16.class.isAssignableFrom(output.getClass()))
						ConvolveImageNoBorder.convolve((Kernel2D_S32) kernel, (InterleavedU8) input, (InterleavedI16) output);
					else
						ConvolveImageNoBorder.convolve((Kernel2D_S32) kernel, (InterleavedU8) input, (InterleavedS32) output);
				} else if (input instanceof InterleavedS16) {
					ConvolveImageNoBorder.convolve((Kernel2D_S32) kernel, (InterleavedS16) input, (InterleavedI16) output);
				} else {
					throw new IllegalArgumentException("Unknown image type: " + input.getClass().getName());
				}
			}break;

			case PLANAR:{
				Planar inp = (Planar)input;
				Planar outp = (Planar)output;

				for (int i = 0; i < inp.getNumBands(); i++) {
					convolve(kernel, inp.getBand(i), outp.getBand(i));
				}
			}break;

			default: throw new IllegalArgumentException("Unknown image family");
		}
	}

	/**
	 * Performs a horizontal 1D convolution across the image while re-normalizing the kernel depending on its
	 * overlap with the image.
	 *
	 * @param input	 The original image. Not modified.
	 * @param output	 Where the resulting image is written to. Modified.
	 * @param kernel The kernel that is being convolved. Not modified.
	 */
	public static <In extends ImageBase, Out extends ImageBase, K extends Kernel1D>
	void horizontalNormalized(K kernel, In input, Out output ) {
		switch (input.getImageType().getFamily()) {
			case GRAY: {
				if (input instanceof GrayF32) {
					ConvolveNormalized.horizontal((Kernel1D_F32) kernel, (GrayF32) input, (GrayF32) output);
				} else if (input instanceof GrayF64) {
					ConvolveNormalized.horizontal((Kernel1D_F64) kernel, (GrayF64) input, (GrayF64) output);
				} else if (input instanceof GrayU8) {
					ConvolveNormalized.horizontal((Kernel1D_S32) kernel, (GrayU8) input, (GrayI8) output);
				} else if (input instanceof GrayS16) {
					ConvolveNormalized.horizontal((Kernel1D_S32) kernel, (GrayS16) input, (GrayI16) output);
				} else {
					throw new IllegalArgumentException("Unknown image type: " + input.getClass().getName());
				}
			}
			break;

			case INTERLEAVED: {
				if (input instanceof InterleavedF32) {
					ConvolveNormalized.horizontal((Kernel1D_F32) kernel, (InterleavedF32) input, (InterleavedF32) output);
				} else if (input instanceof InterleavedF64) {
					ConvolveNormalized.horizontal((Kernel1D_F64) kernel, (InterleavedF64) input, (InterleavedF64) output);
				} else if (input instanceof InterleavedU8) {
					ConvolveNormalized.horizontal((Kernel1D_S32) kernel, (InterleavedU8) input, (InterleavedI8) output);
				} else if (input instanceof InterleavedS16) {
					ConvolveNormalized.horizontal((Kernel1D_S32) kernel, (InterleavedS16) input, (InterleavedI16) output);
				} else {
					throw new IllegalArgumentException("Unknown image type: " + input.getClass().getName());
				}
			}
			break;

			case PLANAR: {
				Planar inp = (Planar)input;
				Planar outp = (Planar)output;

				for (int i = 0; i < inp.getNumBands(); i++) {
					horizontalNormalized(kernel, inp.getBand(i), outp.getBand(i));
				}
			}
			break;

			default:
				throw new IllegalArgumentException("Unknown image family");
		}
	}

	/**
	 * Performs a vertical 1D convolution across the image while re-normalizing the kernel depending on its
	 * overlap with the image.
	 *
	 * @param input	 The original image. Not modified.
	 * @param output	 Where the resulting image is written to. Modified.
	 * @param kernel The kernel that is being convolved. Not modified.
	 */
	public static <T extends ImageBase<T>, K extends Kernel1D>
	void verticalNormalized(K kernel, T input, T output ) {
		switch (input.getImageType().getFamily()) {
			case GRAY: {
				if (input instanceof GrayF32) {
					ConvolveNormalized.vertical((Kernel1D_F32) kernel, (GrayF32) input, (GrayF32) output);
				} else if (input instanceof GrayF64) {
					ConvolveNormalized.vertical((Kernel1D_F64) kernel, (GrayF64) input, (GrayF64) output);
				} else if (input instanceof GrayU8) {
					ConvolveNormalized.vertical((Kernel1D_S32) kernel, (GrayU8) input, (GrayI8) output);
				} else if (input instanceof GrayS16) {
					ConvolveNormalized.vertical((Kernel1D_S32) kernel, (GrayS16) input, (GrayI16) output);
				} else {
					throw new IllegalArgumentException("Unknown image type: " + input.getClass().getName());
				}
			} break;

			case INTERLEAVED: {
				if (input instanceof InterleavedF32) {
					ConvolveNormalized.vertical((Kernel1D_F32) kernel, (InterleavedF32) input, (InterleavedF32) output);
				} else if (input instanceof InterleavedF64) {
					ConvolveNormalized.vertical((Kernel1D_F64) kernel, (InterleavedF64) input, (InterleavedF64) output);
				} else if (input instanceof InterleavedU8) {
					ConvolveNormalized.vertical((Kernel1D_S32) kernel, (InterleavedU8) input, (InterleavedI8) output);
				} else if (input instanceof InterleavedS16) {
					ConvolveNormalized.vertical((Kernel1D_S32) kernel, (InterleavedS16) input, (InterleavedI16) output);
				} else {
					throw new IllegalArgumentException("Unknown image type: " + input.getClass().getName());
				}
			} break;

			case PLANAR: {
				Planar inp = (Planar)input;
				Planar outp = (Planar)output;

				for (int i = 0; i < inp.getNumBands(); i++) {
					verticalNormalized(kernel, inp.getBand(i), outp.getBand(i));
				}
			}
			break;

			default:
				throw new IllegalArgumentException("Unknown image family");
		}
	}

	/**
	 * Performs a 2D convolution across the image while re-normalizing the kernel depending on its
	 * overlap with the image.
	 *
	 * @param input	 The original image. Not modified.
	 * @param output	 Where the resulting image is written to. Modified.
	 * @param kernel The kernel that is being convolved. Not modified.
	 */
	public static <T extends ImageBase<T>, K extends Kernel2D>
	void convolveNormalized(K kernel, T input, T output ) {
		switch (input.getImageType().getFamily()) {
			case GRAY: {
				if (input instanceof GrayF32) {
					ConvolveNormalized.convolve((Kernel2D_F32) kernel, (GrayF32) input, (GrayF32) output);
				} else if (input instanceof GrayF64) {
					ConvolveNormalized.convolve((Kernel2D_F64) kernel, (GrayF64) input, (GrayF64) output);
				} else if (input instanceof GrayU8) {
					ConvolveNormalized.convolve((Kernel2D_S32) kernel, (GrayU8) input, (GrayI8) output);
				} else if (input instanceof GrayS16) {
					ConvolveNormalized.convolve((Kernel2D_S32) kernel, (GrayS16) input, (GrayI16) output);
				} else {
					throw new IllegalArgumentException("Unknown image type: " + input.getClass().getName());
				}
			} break;

			case INTERLEAVED: {
				if (input instanceof InterleavedF32) {
					ConvolveNormalized.convolve((Kernel2D_F32) kernel, (InterleavedF32) input, (InterleavedF32) output);
				} else if (input instanceof InterleavedF64) {
					ConvolveNormalized.convolve((Kernel2D_F64) kernel, (InterleavedF64) input, (InterleavedF64) output);
				} else if (input instanceof InterleavedU8) {
					ConvolveNormalized.convolve((Kernel2D_S32) kernel, (InterleavedU8) input, (InterleavedI8) output);
				} else if (input instanceof InterleavedS16) {
					ConvolveNormalized.convolve((Kernel2D_S32) kernel, (InterleavedS16) input, (InterleavedI16) output);
				} else {
					throw new IllegalArgumentException("Unknown image type: " + input.getClass().getName());
				}
			} break;

			case PLANAR: {
				Planar inp = (Planar) input;
				Planar outp = (Planar) output;

				for (int i = 0; i < inp.getNumBands(); i++) {
					convolveNormalized(kernel, inp.getBand(i), outp.getBand(i));
				}
			} break;

			default:
				throw new IllegalArgumentException("Unknown image family");
		}
	}
}
