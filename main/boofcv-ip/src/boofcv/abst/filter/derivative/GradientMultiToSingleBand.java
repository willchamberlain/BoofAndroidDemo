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

package boofcv.abst.filter.derivative;

import boofcv.struct.image.ImageGray;
import boofcv.struct.image.ImageMultiBand;
import boofcv.struct.image.ImageType;

/**
 * Interface for converting a multi-band gradient into a single band gradient.
 *
 * @author Peter Abeles
 */
public interface GradientMultiToSingleBand<Input extends ImageMultiBand<Input>, Output extends ImageGray<Output>>
{
	void process( Input inDerivX , Input inDerivY , Output  outDerivX , Output outDerivY );

	ImageType<Input> getInputType();

	Class<Output> getOutputType();
}
