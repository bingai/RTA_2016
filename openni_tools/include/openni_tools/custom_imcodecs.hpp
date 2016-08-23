//  ================================================================
// Created by Gregory Kramida on 7/19/16.
//  Copyright (c) 2016 Gregory Kramida
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//  http://www.apache.org/licenses/LICENSE-2.0

//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//  ================================================================


#pragma once
#define HAVE_PNG
#define HAVE_LIBPNG_PNG_H
//#define _LFS64_LARGEFILE
#include <opencv2/core/core.hpp>

namespace custom_cv {
//! Imwrite flags
	enum ImwriteFlags {
		IMWRITE_PNG_COMPRESSION = 16, //!< For PNG, it can be the compression level from 0 to 9. A higher value means a smaller size and longer compression time. Default value is 3. Also strategy is changed to IMWRITE_PNG_STRATEGY_DEFAULT (Z_DEFAULT_STRATEGY).
		IMWRITE_PNG_STRATEGY = 17, //!< One of cv::ImwritePNGFlags, default is IMWRITE_PNG_STRATEGY_DEFAULT.
		IMWRITE_PNG_BILEVEL = 18, //!< Binary level PNG, 0 or 1, default is 0.
	};

	enum ImwritePNGFlags {
		IMWRITE_PNG_STRATEGY_DEFAULT      = 0, //!< Use this value for normal data.
		IMWRITE_PNG_STRATEGY_FILTERED     = 1, //!< Use this value for data produced by a filter (or predictor).Filtered data consists mostly of small values with a somewhat random distribution. In this case, the compression algorithm is tuned to compress them better.
		IMWRITE_PNG_STRATEGY_HUFFMAN_ONLY = 2, //!< Use this value to force Huffman encoding only (no string match).
		IMWRITE_PNG_STRATEGY_RLE          = 3, //!< Use this value to limit match distances to one (run-length encoding).
		IMWRITE_PNG_STRATEGY_FIXED        = 4  //!< Using this value prevents the use of dynamic Huffman codes, allowing for a simpler decoder for special applications.
	};

	enum ImreadModes {
		IMREAD_UNCHANGED            = -1, //!< If set, return the loaded image as is (with alpha channel, otherwise it gets cropped).
		IMREAD_GRAYSCALE            = 0,  //!< If set, always convert image to the single channel grayscale image.
		IMREAD_COLOR                = 1,  //!< If set, always convert image to the 3 channel BGR color image.
		IMREAD_ANYDEPTH             = 2,  //!< If set, return 16-bit/32-bit image when the input has the corresponding depth, otherwise convert it to 8-bit.
		IMREAD_ANYCOLOR             = 4,  //!< If set, the image is read in any possible color format.
		IMREAD_LOAD_GDAL            = 8,  //!< If set, use the gdal driver for loading the image.
		IMREAD_REDUCED_GRAYSCALE_2  = 16, //!< If set, always convert image to the single channel grayscale image and the image size reduced 1/2.
		IMREAD_REDUCED_COLOR_2      = 17, //!< If set, always convert image to the 3 channel BGR color image and the image size reduced 1/2.
		IMREAD_REDUCED_GRAYSCALE_4  = 32, //!< If set, always convert image to the single channel grayscale image and the image size reduced 1/4.
		IMREAD_REDUCED_COLOR_4      = 33, //!< If set, always convert image to the 3 channel BGR color image and the image size reduced 1/4.
		IMREAD_REDUCED_GRAYSCALE_8  = 64, //!< If set, always convert image to the single channel grayscale image and the image size reduced 1/8.
		IMREAD_REDUCED_COLOR_8      = 65  //!< If set, always convert image to the 3 channel BGR color image and the image size reduced 1/8.
	};

	enum
	{
/* 8bit, color or not */
				CV_LOAD_IMAGE_UNCHANGED  =-1,
/* 8bit, gray */
				CV_LOAD_IMAGE_GRAYSCALE  =0,
/* ?, color */
				CV_LOAD_IMAGE_COLOR      =1,
/* any depth, ? */
				CV_LOAD_IMAGE_ANYDEPTH   =2,
/* ?, any color */
				CV_LOAD_IMAGE_ANYCOLOR   =4
	};

/** @brief Loads an image from a file.

@anchor imread

The function imread loads an image from the specified file and returns it. If the image cannot be
read (because of missing file, improper permissions, unsupported or invalid format), the function
returns an empty matrix ( cv::Mat::data==NULL ).

Currently, the following file formats are supported:

-   Windows bitmaps - \*.bmp, \*.dib (always supported)
-   JPEG files - \*.jpeg, \*.jpg, \*.jpe (see the *Notes* section)
-   JPEG 2000 files - \*.jp2 (see the *Notes* section)
-   Portable Network Graphics - \*.png (see the *Notes* section)
-   WebP - \*.webp (see the *Notes* section)
-   Portable image format - \*.pbm, \*.pgm, \*.ppm \*.pxm, \*.pnm (always supported)
-   Sun rasters - \*.sr, \*.ras (always supported)
-   TIFF files - \*.tiff, \*.tif (see the *Notes* section)
-   OpenEXR Image files - \*.exr (see the *Notes* section)
-   Radiance HDR - \*.hdr, \*.pic (always supported)
-   Raster and Vector geospatial data supported by Gdal (see the *Notes* section)

@note

-   The function determines the type of an image by the content, not by the file extension.
-   In the case of color images, the decoded images will have the channels stored in **B G R** order.
-   On Microsoft Windows\* OS and MacOSX\*, the codecs shipped with an OpenCV image (libjpeg,
    libpng, libtiff, and libjasper) are used by default. So, OpenCV can always read JPEGs, PNGs,
    and TIFFs. On MacOSX, there is also an option to use native MacOSX image readers. But beware
    that currently these native image loaders give images with different pixel values because of
    the color management embedded into MacOSX.
-   On Linux\*, BSD flavors and other Unix-like open-source operating systems, OpenCV looks for
    codecs supplied with an OS image. Install the relevant packages (do not forget the development
    files, for example, "libjpeg-dev", in Debian\* and Ubuntu\*) to get the codec support or turn
    on the OPENCV_BUILD_3RDPARTY_LIBS flag in CMake.
-   In the case you set *WITH_GDAL* flag to true in CMake and @ref IMREAD_LOAD_GDAL to load the image,
    then [GDAL](http://www.gdal.org) driver will be used in order to decode the image by supporting
    the following formats: [Raster](http://www.gdal.org/formats_list.html),
    [Vector](http://www.gdal.org/ogr_formats.html).
@param filename Name of file to be loaded.
@param flags Flag that can take values of cv::ImreadModes
*/
	CV_EXPORTS_W cv::Mat imread( const cv::String& filename, int flags = IMREAD_COLOR );

/** @brief Loads a multi-page image from a file.

The function imreadmulti loads a multi-page image from the specified file into a vector of cv::Mat objects.
@param filename Name of file to be loaded.
@param flags Flag that can take values of cv::ImreadModes, default with cv::IMREAD_ANYCOLOR.
@param mats A vector of cv::Mat objects holding each page, if more than one.
@sa cv::imread
*/
	CV_EXPORTS_W bool imreadmulti(const cv::String& filename, std::vector<cv::Mat>& mats, int flags = IMREAD_ANYCOLOR);

/** @brief Saves an image to a specified file.

The function imwrite saves the image to the specified file. The image format is chosen based on the
filename extension (see cv::imread for the list of extensions). Only 8-bit (or 16-bit unsigned (CV_16U)
in case of PNG, JPEG 2000, and TIFF) single-channel or 3-channel (with 'BGR' channel order) images
can be saved using this function. If the format, depth or channel order is different, use
cv::Mat::convertTo , and cv::cvtColor to convert it before saving. Or, use the universal FileStorage I/O
functions to save the image to XML or YAML format.

It is possible to store PNG images with an alpha channel using this function. To do this, create
8-bit (or 16-bit) 4-channel image BGRA, where the alpha channel goes last. Fully transparent pixels
should have alpha set to 0, fully opaque pixels should have alpha set to 255/65535.

The sample below shows how to create such a BGRA image and store to PNG file. It also demonstrates how to set custom
compression parameters :
@code
    #include <opencv2/opencv.hpp>

    using namespace cv;
    using namespace std;

    void createAlphaMat(cv::Mat &mat)
    {
        CV_Assert(mat.channels() == 4);
        for (int i = 0; i < mat.rows; ++i) {
            for (int j = 0; j < mat.cols; ++j) {
                Vec4b& bgra = mat.at<Vec4b>(i, j);
                bgra[0] = UCHAR_MAX; // Blue
                bgra[1] = saturate_cast<uchar>((float (mat.cols - j)) / ((float)mat.cols) * UCHAR_MAX); // Green
                bgra[2] = saturate_cast<uchar>((float (mat.rows - i)) / ((float)mat.rows) * UCHAR_MAX); // Red
                bgra[3] = saturate_cast<uchar>(0.5 * (bgra[1] + bgra[2])); // Alpha
            }
        }
    }

    int main(int argv, char **argc)
    {
        // Create mat with alpha channel
        cv::Mat mat(480, 640, CV_8UC4);
        createAlphaMat(mat);

        vector<int> compression_params;
        compression_params.push_back(IMWRITE_PNG_COMPRESSION);
        compression_params.push_back(9);

        try {
            imwrite("alpha.png", mat, compression_params);
        }
        catch (cv::Exception& ex) {
            fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
            return 1;
        }

        fprintf(stdout, "Saved PNG file with alpha data.\n");
        return 0;
    }
@endcode
@param filename Name of the file.
@param img Image to be saved.
@param params Format-specific parameters encoded as pairs (paramId_1, paramValue_1, paramId_2, paramValue_2, ... .) see cv::ImwriteFlags
*/
	CV_EXPORTS_W bool imwrite( const cv::String& filename, cv::InputArray img,
	                           const std::vector<int>& params = std::vector<int>());

/** @brief Reads an image from a buffer in memory.

The function imdecode reads an image from the specified buffer in the memory. If the buffer is too short or
contains invalid data, the function returns an empty matrix ( cv::Mat::data==NULL ).

See cv::imread for the list of supported formats and flags description.

@note In the case of color images, the decoded images will have the channels stored in **B G R** order.
@param buf Input array or vector of bytes.
@param flags The same flags as in cv::imread, see cv::ImreadModes.
*/
	CV_EXPORTS_W cv::Mat imdecode( cv::InputArray buf, int flags );

/** @overload
@param buf
@param flags
@param dst The optional output placeholder for the decoded matrix. It can save the image
reallocations when the function is called repeatedly for images of the same size.
*/
	CV_EXPORTS cv::Mat imdecode( cv::InputArray buf, int flags, cv::Mat* dst);

/** @brief Encodes an image into a memory buffer.

The function imencode compresses the image and stores it in the memory buffer that is resized to fit the
result. See cv::imwrite for the list of supported formats and flags description.

@param ext File extension that defines the output format.
@param img Image to be written.
@param buf Output buffer resized to fit the compressed image.
@param params Format-specific parameters. See cv::imwrite and cv::ImwriteFlags.
*/
	CV_EXPORTS_W bool imencode( const cv::String& ext, cv::InputArray img,
	                            CV_OUT std::vector<uchar>& buf,
	                            const std::vector<int>& params = std::vector<int>());

}