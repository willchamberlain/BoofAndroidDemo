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

package boofcv;

import com.peterabeles.auto64fto32f.ConvertFile32From64;
import com.peterabeles.auto64fto32f.RecursiveConvert;

import java.io.File;

/**
 * Auto generates 32bit code from 64bit code.
 *
 * @author Peter Abeles
 */
public class Autocode64to32App extends RecursiveConvert {


    public Autocode64to32App(ConvertFile32From64 converter) {
        super(converter);
    }

    public static void main(String args[] ) {
        String directories[] = new String[]{
                "main/boofcv-geo/src/boofcv/alg",
                "main/boofcv-geo/test/boofcv/alg",
                "main/boofcv-ip/src/boofcv/alg/distort",
                "main/boofcv-ip/src/boofcv/struct/distort",
                };

        ConvertFile32From64 converter = new ConvertFile32From64(false);

        converter.replacePattern("/\\*\\*/double", "FIXED_DOUBLE");
        converter.replacePattern("double", "float");
        converter.replacePattern("Double", "Float");
        converter.replacePattern("/\\*\\*/Affine2D_F64", "ASASDASDA_AFFINE_2D");
        converter.replacePattern("_F64", "_F32");
        converter.replacePattern("64-bit", "32-bit");
        converter.replacePattern("64F", "32F");
        converter.replacePattern("_DD", "_FD");
        converter.replacePattern("DMatrix", "FMatrix");
        converter.replacePattern("DCONV_TOL_", "FCONV_TOL_");
        converter.replacePattern("GrlConstants.PI", "GrlConstants.F_PI");
        converter.replacePattern("GrlConstants.EPS", "GrlConstants.F_EPS");

        converter.replaceStartsWith("Math.", "(float)Math.");
        converter.replaceStartsWith("-Math.", "(float)-Math.");

        converter.replacePatternAfter("ASASDASDA_AFFINE_2D", "/**/Affine2D_F64");
        converter.replacePatternAfter("FIXED_DOUBLE", "/\\*\\*/double");


        Autocode64to32App app = new Autocode64to32App(converter);
        for( String dir : directories ) {
            app.process(new File(dir) );
        }
    }
}
