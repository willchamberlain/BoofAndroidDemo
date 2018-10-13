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

package boofcv.alg.filter.binary;

import boofcv.misc.AutoTypeImage;
import boofcv.misc.CodeGeneratorBase;

import java.io.FileNotFoundException;


/**
 * @author Peter Abeles
 */
public class GenerateThresholdImageOps extends CodeGeneratorBase {
	String className = "ThresholdImageOps";

	@Override
	public void generate() throws FileNotFoundException {
		printPreamble();

		printAll(AutoTypeImage.F32);
		printAll(AutoTypeImage.F64);
		printAll(AutoTypeImage.U8);
		printAll(AutoTypeImage.S16);
		printAll(AutoTypeImage.U16);
		printAll(AutoTypeImage.S32);

		printLocal(AutoTypeImage.U8);
		printLocal(AutoTypeImage.F32);

		out.print("\n" +
				"}\n");
	}

	private void printPreamble() throws FileNotFoundException {
		setOutputFile(className);
		out.print("import boofcv.alg.InputSanityCheck;\n" +
				"import boofcv.struct.image.*;\n" +
				"import boofcv.alg.filter.blur.BlurImageOps;\n" +
				"\n" +
				"/**\n" +
				" * <p>\n" +
				" * Operations for thresholding images and converting them into a binary image.\n" +
				" * </p>\n" +
				" *\n" +
				" * <p>\n" +
				" * WARNING: Do not modify.  Automatically generated by "+getClass().getSimpleName()+".\n" +
				" * </p>\n" +
				" *\n" +
				" * @author Peter Abeles\n" +
				" */\n" +
				"public class "+className+" {\n\n");

	}

	public void printAll( AutoTypeImage imageIn ) {
		printThreshold(imageIn);
	}

	public void printLocal(AutoTypeImage imageIn) {
		printLocalSquare(imageIn);
		printLocalGaussian(imageIn);
	}

	public void printThreshold( AutoTypeImage imageIn ) {
		out.print("\t/**\n" +
				"\t * Applies a global threshold across the whole image.  If 'down' is true, then pixels with values <=\n" +
				"\t * to 'threshold' are set to 1 and the others set to 0.  If 'down' is false, then pixels with values >\n" +
				"\t * to 'threshold' are set to 1 and the others set to 0.\n" +
				"\t *\n" +
				"\t * @param input Input image. Not modified.\n" +
				"\t * @param output (Optional) Binary output image. If null a new image will be declared. Modified.\n" +
				"\t * @param threshold threshold value.\n" +
				"\t * @param down If true then the inequality <= is used, otherwise if false then &gt; is used.\n" +
				"\t * @return Output image.\n" +
				"\t */\n" +
				"\tpublic static GrayU8 threshold( "+imageIn.getSingleBandName()+" input , GrayU8 output ,\n" +
				"\t\t\t\t\t\t\t\t\t\t"+imageIn.getSumType()+" threshold , boolean down )\n" +
				"\t{\n" +
				"\t\toutput = InputSanityCheck.checkDeclare(input,output,GrayU8.class);\n" +
				"\n" +
				"\t\tif( down ) {\n" +
				"\t\t\tfor( int y = 0; y < input.height; y++ ) {\n" +
				"\t\t\t\tint indexIn = input.startIndex + y*input.stride;\n" +
				"\t\t\t\tint indexOut = output.startIndex + y*output.stride;\n" +
				"\n" +
				"\t\t\t\tint end = indexIn + input.width;\n" +
				"\n" +
				"\t\t\t\tfor( ; indexIn < end; indexIn++ , indexOut++ ) {\n" +
				"\t\t\t\t\tif( (input.data[indexIn]"+imageIn.getBitWise()+") <= threshold )\n" +
				"\t\t\t\t\t\toutput.data[indexOut] = 1;\n" +
				"\t\t\t\t\telse\n" +
				"\t\t\t\t\t\toutput.data[indexOut] = 0;\n" +
				"\t\t\t\t}\n" +
				"\t\t\t}\n" +
				"\t\t} else {\n" +
				"\t\t\tfor( int y = 0; y < input.height; y++ ) {\n" +
				"\t\t\t\tint indexIn = input.startIndex + y*input.stride;\n" +
				"\t\t\t\tint indexOut = output.startIndex + y*output.stride;\n" +
				"\n" +
				"\t\t\t\tint end = indexIn + input.width;\n" +
				"\n" +
				"\t\t\t\tfor( ; indexIn < end; indexIn++ , indexOut++ ) {\n" +
				"\t\t\t\t\tif( (input.data[indexIn]"+imageIn.getBitWise()+") > threshold )\n" +
				"\t\t\t\t\t\toutput.data[indexOut] = 1;\n" +
				"\t\t\t\t\telse\n" +
				"\t\t\t\t\t\toutput.data[indexOut] = 0;\n" +
				"\t\t\t\t}\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\n" +
				"\t\treturn output;\n" +
				"\t}\n\n");
	}

	public void printLocalSquare(AutoTypeImage imageIn) {

		String imageName = imageIn.getSingleBandName();
		String sumType = imageIn.getSumType();
		String bitwise = imageIn.getBitWise();

		out.print("\t/**\n" +
				"\t * Thresholds the image using a locally adaptive threshold that is computed using a local square region centered\n" +
				"\t * on each pixel.  The threshold is equal to the average value of the surrounding pixels times the scale.\n" +
				"\t * If down is true then b(x,y) = I(x,y) &le; T(x,y) * scale ? 1 : 0.  Otherwise\n" +
				"\t * b(x,y) = I(x,y) * scale &gt; T(x,y) ? 0 : 1\n" +
				"\t *\n" +
				"\t * @param input Input image.\n" +
				"\t * @param output (optional) Output binary image.  If null it will be declared internally.\n" +
				"\t * @param radius Radius of square region.\n" +
				"\t * @param scale Scale factor used to adjust threshold.  Try 0.95\n" +
				"\t * @param down Should it threshold up or down.\n" +
				"\t * @param storage1 (Optional) Storage for intermediate step. If null will be declared internally.\n" +
				"\t * @param storage2 (Optional) Storage for intermediate step. If null will be declared internally.\n" +
				"\t * @return Thresholded image.\n" +
				"\t */\n" +
				"\tpublic static GrayU8 localSquare( "+imageName+" input , GrayU8 output ,\n" +
				"\t\t\t\t\t\t\t\t\t\t\t int radius , float scale , boolean down ,\n" +
				"\t\t\t\t\t\t\t\t\t\t\t "+imageName+" storage1 , "+imageName+" storage2 ) {\n" +
				"\n" +
				"\t\toutput = InputSanityCheck.checkDeclare(input,output,GrayU8.class);\n" +
				"\t\tstorage1 = InputSanityCheck.checkDeclare(input,storage1,"+imageName+".class);\n" +
				"\t\tstorage2 = InputSanityCheck.checkDeclare(input,storage2,"+imageName+".class);\n" +
				"\n" +
				"\t\t"+imageName+" mean = storage1;\n" +
				"\n" +
				"\t\tBlurImageOps.mean(input,mean,radius,storage2);\n" +
				"\n" +
				"\t\tif( down ) {\n" +
				"\t\t\tfor( int y = 0; y < input.height; y++ ) {\n" +
				"\t\t\t\tint indexIn = input.startIndex + y*input.stride;\n" +
				"\t\t\t\tint indexOut = output.startIndex + y*output.stride;\n" +
				"\t\t\t\tint indexMean = mean.startIndex + y*mean.stride;\n" +
				"\n" +
				"\t\t\t\tint end = indexIn + input.width;\n" +
				"\n" +
				"\t\t\t\tfor( ; indexIn < end; indexIn++ , indexOut++, indexMean++ ) {\n" +
				"\t\t\t\t\tfloat threshold = (mean.data[indexMean]"+bitwise+") * scale;\n" +
				"\n" +
				"\t\t\t\t\tif( (input.data[indexIn]"+bitwise+") <= threshold )\n" +
				"\t\t\t\t\t\toutput.data[indexOut] = 1;\n" +
				"\t\t\t\t\telse\n" +
				"\t\t\t\t\t\toutput.data[indexOut] = 0;\n" +
				"\t\t\t\t}\n" +
				"\t\t\t}\n" +
				"\t\t} else {\n" +
				"\t\t\tfor( int y = 0; y < input.height; y++ ) {\n" +
				"\t\t\t\tint indexIn = input.startIndex + y*input.stride;\n" +
				"\t\t\t\tint indexOut = output.startIndex + y*output.stride;\n" +
				"\t\t\t\tint indexMean = mean.startIndex + y*mean.stride;\n" +
				"\n" +
				"\t\t\t\tint end = indexIn + input.width;\n" +
				"\n" +
				"\t\t\t\tfor( ; indexIn < end; indexIn++ , indexOut++, indexMean++ ) {\n" +
				"\t\t\t\t\t"+sumType+" threshold = (mean.data[indexMean]"+bitwise+");\n" +
				"\n" +
				"\t\t\t\t\tif( (input.data[indexIn]"+bitwise+") * scale > threshold )\n" +
				"\t\t\t\t\t\toutput.data[indexOut] = 1;\n" +
				"\t\t\t\t\telse\n" +
				"\t\t\t\t\t\toutput.data[indexOut] = 0;\n" +
				"\t\t\t\t}\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\n" +
				"\t\treturn output;\n" +
				"\t}\n\n");
	}

	public void printLocalGaussian(AutoTypeImage imageIn) {

		String imageName = imageIn.getSingleBandName();
		String sumType = imageIn.getSumType();
		String bitwise = imageIn.getBitWise();

		out.print("\t/**\n" +
				"\t * Thresholds the image using a locally adaptive threshold that is computed using a local square region centered\n" +
				"\t * on each pixel.  The threshold is equal to the gaussian weighted sum of the surrounding pixels times the scale.\n" +
				"\t * If down is true then b(x,y) = I(x,y) &le; T(x,y) * scale ? 1 : 0.  Otherwise\n" +
				"\t * b(x,y) = I(x,y) * scale &gt; T(x,y) ? 0 : 1\n" +
				"\t *\n" +
				"\t * @param input Input image.\n" +
				"\t * @param output (optional) Output binary image.  If null it will be declared internally.\n" +
				"\t * @param radius Radius of square region.\n" +
				"\t * @param scale Scale factor used to adjust threshold.  Try 0.95\n" +
				"\t * @param down Should it threshold up or down.\n" +
				"\t * @param storage1 (Optional) Storage for intermediate step. If null will be declared internally.\n" +
				"\t * @param storage2 (Optional) Storage for intermediate step. If null will be declared internally.\n" +
				"\t * @return Thresholded image.\n" +
				"\t */\n" +
				"\tpublic static GrayU8 localGaussian( "+imageName+" input , GrayU8 output ,\n" +
				"\t\t\t\t\t\t\t\t\t\t\t   int radius , float scale , boolean down ,\n" +
				"\t\t\t\t\t\t\t\t\t\t\t   "+imageName+" storage1 , "+imageName+" storage2 ) {\n" +
				"\n" +
				"\t\toutput = InputSanityCheck.checkDeclare(input,output,GrayU8.class);\n" +
				"\t\tstorage1 = InputSanityCheck.checkDeclare(input,storage1,"+imageName+".class);\n" +
				"\t\tstorage2 = InputSanityCheck.checkDeclare(input,storage2,"+imageName+".class);\n" +
				"\n" +
				"\t\t"+imageName+" blur = storage1;\n" +
				"\n" +
				"\t\tBlurImageOps.gaussian(input,blur,-1,radius,storage2);\n" +
				"\n" +
				"\t\tif( down ) {\n" +
				"\t\t\tfor( int y = 0; y < input.height; y++ ) {\n" +
				"\t\t\t\tint indexIn = input.startIndex + y*input.stride;\n" +
				"\t\t\t\tint indexOut = output.startIndex + y*output.stride;\n" +
				"\t\t\t\tint indexMean = blur.startIndex + y*blur.stride;\n" +
				"\n" +
				"\t\t\t\tint end = indexIn + input.width;\n" +
				"\n" +
				"\t\t\t\tfor( ; indexIn < end; indexIn++ , indexOut++, indexMean++ ) {\n" +
				"\t\t\t\t\tfloat threshold = (blur.data[indexMean]"+bitwise+") * scale;\n" +
				"\n" +
				"\t\t\t\t\tif( (input.data[indexIn]"+bitwise+") <= threshold )\n" +
				"\t\t\t\t\t\toutput.data[indexOut] = 1;\n" +
				"\t\t\t\t\telse\n" +
				"\t\t\t\t\t\toutput.data[indexOut] = 0;\n" +
				"\t\t\t\t}\n" +
				"\t\t\t}\n" +
				"\t\t} else {\n" +
				"\t\t\tfor( int y = 0; y < input.height; y++ ) {\n" +
				"\t\t\t\tint indexIn = input.startIndex + y*input.stride;\n" +
				"\t\t\t\tint indexOut = output.startIndex + y*output.stride;\n" +
				"\t\t\t\tint indexMean = blur.startIndex + y*blur.stride;\n" +
				"\n" +
				"\t\t\t\tint end = indexIn + input.width;\n" +
				"\n" +
				"\t\t\t\tfor( ; indexIn < end; indexIn++ , indexOut++, indexMean++ ) {\n" +
				"\t\t\t\t\t"+sumType+" threshold = (blur.data[indexMean]"+bitwise+");\n" +
				"\n" +
				"\t\t\t\t\tif( (input.data[indexIn]"+bitwise+") * scale > threshold )\n" +
				"\t\t\t\t\t\toutput.data[indexOut] = 1;\n" +
				"\t\t\t\t\telse\n" +
				"\t\t\t\t\t\toutput.data[indexOut] = 0;\n" +
				"\t\t\t\t}\n" +
				"\t\t\t}\n" +
				"\t\t}\n" +
				"\n" +
				"\t\treturn output;\n" +
				"\t}\n\n");

	}

	public static void main( String args[] ) throws FileNotFoundException {
		GenerateThresholdImageOps app = new GenerateThresholdImageOps();
		app.generate();
	}
}
