/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

//
//  Loading and saving IPL images.
//

#include <openni_tools/custom_imcodecs.hpp>
#include "grfmt_png.hpp"

#undef min
#undef max

#include <iostream>
#include <opencv2/core/core_c.h>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/imgproc.hpp>

/****************************************************************************************\
*                                      Image Codecs                                      *
\****************************************************************************************/
namespace custom_cv {

/**
 * @struct ImageCodecInitializer
 *
 * Container which stores the registered codecs to be used by OpenCV
*/
	struct ImageCodecInitializer {
		/**
		 * Default Constructor for the ImageCodeInitializer
		*/
		ImageCodecInitializer() {
#ifdef HAVE_PNG
			decoders.push_back(cv::Ptr<PngDecoder>(new PngDecoder()));
			encoders.push_back(cv::Ptr<PngEncoder>(new PngEncoder()));
#endif
		}

		std::vector<ImageDecoder> decoders;
		std::vector<ImageEncoder> encoders;
	};

	static ImageCodecInitializer codecs;

/**
 * Find the decoders
 *
 * @param[in] filename File to search
 *
 * @return Image decoder to parse image file.
*/
	static ImageDecoder findDecoder(const cv::String& filename) {

		size_t i, maxlen = 0;

		/// iterate through list of registered codecs
		for (i = 0; i < codecs.decoders.size(); i++) {
			size_t len = codecs.decoders[i]->signatureLength();
			maxlen = std::max(maxlen, len);
		}

		/// Open the file
		FILE* f = fopen(filename.c_str(), "rb");

		/// in the event of a failure, return an empty image decoder
		if (!f)
			return ImageDecoder();

		// read the file signature
		cv::String signature(maxlen, ' ');
		maxlen = fread((void*) signature.c_str(), 1, maxlen, f);
		fclose(f);
		signature = signature.substr(0, maxlen);

		/// compare signature against all decoders
		for (i = 0; i < codecs.decoders.size(); i++) {
			if (codecs.decoders[i]->checkSignature(signature))
				return codecs.decoders[i]->newDecoder();
		}

		/// If no decoder was found, return base type
		return ImageDecoder();
	}

	static ImageDecoder findDecoder(const cv::Mat& buf) {
		size_t i, maxlen = 0;

		if (buf.rows * buf.cols < 1 || !buf.isContinuous())
			return ImageDecoder();

		for (i = 0; i < codecs.decoders.size(); i++) {
			size_t len = codecs.decoders[i]->signatureLength();
			maxlen = std::max(maxlen, len);
		}

		cv::String signature(maxlen, ' ');
		size_t bufSize = buf.rows * buf.cols * buf.elemSize();
		maxlen = std::min(maxlen, bufSize);
		memcpy((void*) signature.c_str(), buf.data, maxlen);

		for (i = 0; i < codecs.decoders.size(); i++) {
			if (codecs.decoders[i]->checkSignature(signature))
				return codecs.decoders[i]->newDecoder();
		}

		return ImageDecoder();
	}

	static ImageEncoder findEncoder(const cv::String& _ext) {
		if (_ext.size() <= 1)
			return ImageEncoder();

		const char* ext = strrchr(_ext.c_str(), '.');
		if (!ext)
			return ImageEncoder();
		int len = 0;
		for (ext++; len < 128 && isalnum(ext[len]); len++);

		for (size_t i = 0; i < codecs.encoders.size(); i++) {
			cv::String description = codecs.encoders[i]->getDescription();
			const char* descr = strchr(description.c_str(), '(');

			while (descr) {
				descr = strchr(descr + 1, '.');
				if (!descr)
					break;
				int j = 0;
				for (descr++; j < len && isalnum(descr[j]); j++) {
					int c1 = tolower(ext[j]);
					int c2 = tolower(descr[j]);
					if (c1 != c2)
						break;
				}
				if (j == len && !isalnum(descr[j]))
					return codecs.encoders[i]->newEncoder();
				descr += j;
			}
		}

		return ImageEncoder();
	}

	enum {
		LOAD_CVMAT = 0, LOAD_IMAGE = 1, LOAD_MAT = 2
	};

/**
 * Read an image into memory and return the information
 *
 * @param[in] filename File to load
 * @param[in] flags Flags
 * @param[in] hdrtype { LOAD_CVMAT=0,
 *                      LOAD_IMAGE=1,
 *                      LOAD_MAT=2
 *                    }
 * @param[in] mat Reference to C++ Mat object (If LOAD_MAT)
 * @param[in] scale_denom Scale value
 *
*/
	static void*
	imread_(const cv::String& filename, int flags, int hdrtype, cv::Mat* mat = 0) {
		IplImage* image = 0;
		CvMat* matrix = 0;
		cv::Mat temp, * data = &temp;

		/// Search for the relevant decoder to handle the imagery
		ImageDecoder decoder;

#ifdef HAVE_GDAL
		if(flags != IMREAD_UNCHANGED && (flags & IMREAD_LOAD_GDAL) == IMREAD_LOAD_GDAL ){
			decoder = GdalDecoder().newDecoder();
		}else{
#endif
		decoder = findDecoder(filename);
#ifdef HAVE_GDAL
		}
#endif

		/// if no decoder was found, return nothing.
		if (!decoder) {
			return 0;
		}

		int scale_denom = 1;
		if (flags > IMREAD_LOAD_GDAL) {
			if (flags & IMREAD_REDUCED_GRAYSCALE_2)
				scale_denom = 2;
			else if (flags & IMREAD_REDUCED_GRAYSCALE_4)
				scale_denom = 4;
			else if (flags & IMREAD_REDUCED_GRAYSCALE_8)
				scale_denom = 8;
		}

		/// set the scale_denom in the driver
		decoder->setScale(scale_denom);

		/// set the filename in the driver
		decoder->setSource(filename);

		// read the header to make sure it succeeds
		if (!decoder->readHeader())
			return 0;

		// established the required input image size
		CvSize size;
		size.width = decoder->width();
		size.height = decoder->height();

		// grab the decoded type
		int type = decoder->type();
		if ((flags & IMREAD_LOAD_GDAL) != IMREAD_LOAD_GDAL && flags != IMREAD_UNCHANGED) {
			if ((flags & CV_LOAD_IMAGE_ANYDEPTH) == 0)
				type = CV_MAKETYPE(CV_8U, CV_MAT_CN(type));

			if ((flags & CV_LOAD_IMAGE_COLOR) != 0 ||
			    ((flags & CV_LOAD_IMAGE_ANYCOLOR) != 0 && CV_MAT_CN(type) > 1))
				type = CV_MAKETYPE(CV_MAT_DEPTH(type), 3);
			else
				type = CV_MAKETYPE(CV_MAT_DEPTH(type), 1);
		}

		if (hdrtype == LOAD_CVMAT || hdrtype == LOAD_MAT) {
			if (hdrtype == LOAD_CVMAT) {
				matrix = cvCreateMat(size.height, size.width, type);
				temp = cv::cvarrToMat(matrix);
			}
			else {
				mat->create(size.height, size.width, type);
				data = mat;
			}
		}
		else {
			image = cvCreateImage(size, cvIplDepth(type), CV_MAT_CN(type));
			temp = cv::cvarrToMat(image);
		}

		// read the image data
		if (!decoder->readData(*data)) {
			cvReleaseImage(&image);
			cvReleaseMat(&matrix);
			if (mat)
				mat->release();
			return 0;
		}

		if (decoder->setScale(scale_denom) > 1) // if decoder is JpegDecoder then decoder->setScale always returns 1
		{
			cv::resize(*mat, *mat, cv::Size(size.width / scale_denom, size.height / scale_denom));
		}

		return hdrtype == LOAD_CVMAT ? (void*) matrix :
		       hdrtype == LOAD_IMAGE ? (void*) image : (void*) mat;
	}


/**
* Read an image into memory and return the information
*
* @param[in] filename File to load
* @param[in] flags Flags
* @param[in] mats Reference to C++ vector<Mat> object to hold the images
*
*/
	static bool
	imreadmulti_(const cv::String& filename, int flags, std::vector<cv::Mat>& mats) {
		/// Search for the relevant decoder to handle the imagery
		ImageDecoder decoder;

#ifdef HAVE_GDAL
		if (flags != IMREAD_UNCHANGED && (flags & IMREAD_LOAD_GDAL) == IMREAD_LOAD_GDAL){
			decoder = GdalDecoder().newDecoder();
		}
		else{
#endif
		decoder = findDecoder(filename);
#ifdef HAVE_GDAL
		}
#endif

		/// if no decoder was found, return nothing.
		if (!decoder) {
			return 0;
		}

		/// set the filename in the driver
		decoder->setSource(filename);

		// read the header to make sure it succeeds
		if (!decoder->readHeader())
			return 0;

		for (; ;) {
			// grab the decoded type
			int type = decoder->type();
			if ((flags & IMREAD_LOAD_GDAL) != IMREAD_LOAD_GDAL && flags != IMREAD_UNCHANGED) {
				if ((flags & CV_LOAD_IMAGE_ANYDEPTH) == 0)
					type = CV_MAKETYPE(CV_8U, CV_MAT_CN(type));

				if ((flags & CV_LOAD_IMAGE_COLOR) != 0 ||
				    ((flags & CV_LOAD_IMAGE_ANYCOLOR) != 0 && CV_MAT_CN(type) > 1))
					type = CV_MAKETYPE(CV_MAT_DEPTH(type), 3);
				else
					type = CV_MAKETYPE(CV_MAT_DEPTH(type), 1);
			}

			// read the image data
			cv::Mat mat(decoder->height(), decoder->width(), type);
			if (!decoder->readData(mat)) {
				break;
			}

			mats.push_back(mat);
			if (!decoder->nextPage()) {
				break;
			}
		}

		return !mats.empty();
	}

/**
 * Read an image
 *
 *  This function merely calls the actual implementation above and returns itself.
 *
 * @param[in] filename File to load
 * @param[in] flags Flags you wish to set.
*/
	cv::Mat imread(const cv::String& filename, int flags) {
		/// create the basic container
		cv::Mat img;

		/// load the data
		imread_(filename, flags, LOAD_MAT, &img);

		/// return a reference to the data
		return img;
	}

/**
* Read a multi-page image
*
*  This function merely calls the actual implementation above and returns itself.
*
* @param[in] filename File to load
* @param[in] mats Reference to C++ vector<Mat> object to hold the images
* @param[in] flags Flags you wish to set.
*
*/
	bool imreadmulti(const cv::String& filename, std::vector<cv::Mat>& mats, int flags) {
		return imreadmulti_(filename, flags, mats);
	}

	static bool imwrite_(const cv::String& filename, const cv::Mat& image,
	                     const std::vector<int>& params, bool flipv) {
		cv::Mat temp;
		const cv::Mat* pimage = &image;

		CV_Assert(image.channels() == 1 || image.channels() == 3 || image.channels() == 4);

		ImageEncoder encoder = findEncoder(filename);
		if (!encoder)
			CV_Error(CV_StsError, "could not find a writer for the specified extension");
		if (!encoder->isFormatSupported(image.depth())) {
			CV_Assert(encoder->isFormatSupported(CV_8U));
			image.convertTo(temp, CV_8U);
			pimage = &temp;
		}

		if (flipv) {
			flip(*pimage, temp, 0);
			pimage = &temp;
		}

		encoder->setDestination(filename);
		bool code = encoder->write(*pimage, params);

		//    CV_Assert( code );
		return code;
	}

	bool imwrite(const cv::String& filename, cv::InputArray _img,
	             const std::vector<int>& params) {
		cv::Mat img = _img.getMat();
		return imwrite_(filename, img, params, false);
	}

	static void*
	imdecode_(const cv::Mat& buf, int flags, int hdrtype, cv::Mat* mat = 0) {
		CV_Assert(!buf.empty() && buf.isContinuous());
		IplImage* image = 0;
		CvMat* matrix = 0;
		cv::Mat temp, * data = &temp;
		cv::String filename;

		ImageDecoder decoder = findDecoder(buf);
		if (!decoder)
			return 0;

		if (!decoder->setSource(buf)) {
			filename = cv::tempfile();
			FILE* f = fopen(filename.c_str(), "wb");
			if (!f)
				return 0;
			size_t bufSize = buf.cols * buf.rows * buf.elemSize();
			fwrite(buf.ptr(), 1, bufSize, f);
			fclose(f);
			decoder->setSource(filename);
		}

		if (!decoder->readHeader()) {
			if (!filename.empty())
				remove(filename.c_str());
			return 0;
		}

		CvSize size;
		size.width = decoder->width();
		size.height = decoder->height();

		int type = decoder->type();
		if ((flags & IMREAD_LOAD_GDAL) != IMREAD_LOAD_GDAL && flags != IMREAD_UNCHANGED) {
			if ((flags & CV_LOAD_IMAGE_ANYDEPTH) == 0)
				type = CV_MAKETYPE(CV_8U, CV_MAT_CN(type));

			if ((flags & CV_LOAD_IMAGE_COLOR) != 0 ||
			    ((flags & CV_LOAD_IMAGE_ANYCOLOR) != 0 && CV_MAT_CN(type) > 1))
				type = CV_MAKETYPE(CV_MAT_DEPTH(type), 3);
			else
				type = CV_MAKETYPE(CV_MAT_DEPTH(type), 1);
		}

		if (hdrtype == LOAD_CVMAT || hdrtype == LOAD_MAT) {
			if (hdrtype == LOAD_CVMAT) {
				matrix = cvCreateMat(size.height, size.width, type);
				temp = cv::cvarrToMat(matrix);
			}
			else {
				mat->create(size.height, size.width, type);
				data = mat;
			}
		}
		else {
			image = cvCreateImage(size, cvIplDepth(type), CV_MAT_CN(type));
			temp = cv::cvarrToMat(image);
		}

		bool code = decoder->readData(*data);
		if (!filename.empty())
			remove(filename.c_str());

		if (!code) {
			cvReleaseImage(&image);
			cvReleaseMat(&matrix);
			if (mat)
				mat->release();
			return 0;
		}

		return hdrtype == LOAD_CVMAT ? (void*) matrix :
		       hdrtype == LOAD_IMAGE ? (void*) image : (void*) mat;
	}


	cv::Mat imdecode(cv::InputArray _buf, int flags) {
		cv::Mat buf = _buf.getMat(), img;
		imdecode_(buf, flags, LOAD_MAT, &img);
		return img;
	}

	cv::Mat imdecode(cv::InputArray _buf, int flags, cv::Mat* dst) {
		cv::Mat buf = _buf.getMat(), img;
		dst = dst ? dst : &img;
		imdecode_(buf, flags, LOAD_MAT, dst);
		return *dst;
	}

	bool imencode(const cv::String& ext, cv::InputArray _image,
	              std::vector<uchar>& buf, const std::vector<int>& params) {
		cv::Mat image = _image.getMat();

		int channels = image.channels();
		CV_Assert(channels == 1 || channels == 3 || channels == 4);

		ImageEncoder encoder = findEncoder(ext);
		if (!encoder)
			CV_Error(CV_StsError, "could not find encoder for the specified extension");

		if (!encoder->isFormatSupported(image.depth())) {
			CV_Assert(encoder->isFormatSupported(CV_8U));
			cv::Mat temp;
			image.convertTo(temp, CV_8U);
			image = temp;
		}

		bool code;
		if (encoder->setDestination(buf)) {
			code = encoder->write(image, params);
			encoder->throwOnEror();
			CV_Assert(code);
		}
		else {
			cv::String filename = cv::tempfile();
			code = encoder->setDestination(filename);
			CV_Assert(code);

			code = encoder->write(image, params);
			encoder->throwOnEror();
			CV_Assert(code);

			FILE* f = fopen(filename.c_str(), "rb");
			CV_Assert(f != 0);
			fseek(f, 0, SEEK_END);
			long pos = ftell(f);
			buf.resize((size_t) pos);
			fseek(f, 0, SEEK_SET);
			buf.resize(fread(&buf[0], 1, buf.size(), f));
			fclose(f);
			remove(filename.c_str());
		}
		return code;
	}

}

/****************************************************************************************\
*                         Imgcodecs loading & saving function implementation            *
\****************************************************************************************/

CV_EXTERN_C int
cvHaveImageReader(const char* filename) {
	custom_cv::ImageDecoder decoder = custom_cv::findDecoder(filename);
	return !decoder.empty();
}

CV_EXTERN_C int cvHaveImageWriter(const char* filename) {
	custom_cv::ImageEncoder encoder = custom_cv::findEncoder(filename);
	return !encoder.empty();
}

CV_EXTERN_C IplImage
*

cvLoadImage(const char* filename, int iscolor) {
	return (IplImage*) custom_cv::imread_(filename, iscolor, custom_cv::LOAD_IMAGE);
}

CV_EXTERN_C CvMat
*

cvLoadImageM(const char* filename, int iscolor) {
	return (CvMat*) custom_cv::imread_(filename, iscolor, custom_cv::LOAD_CVMAT);
}

CV_EXTERN_C int
cvSaveImage(const char* filename, const CvArr* arr, const int* _params) {
	int i = 0;
	if (_params) {
		for (; _params[i] > 0; i += 2);
	}
	return custom_cv::imwrite_(filename, cv::cvarrToMat(arr),
	                           i > 0 ? std::vector<int>(_params, _params + i) : std::vector<int>(),
	                           CV_IS_IMAGE(arr) && ((const IplImage*) arr)->origin == IPL_ORIGIN_BL);
}

/* decode image stored in the buffer */
CV_EXTERN_C IplImage
*

cvDecodeImage(const CvMat* _buf, int iscolor) {
	CV_Assert(_buf && CV_IS_MAT_CONT(_buf->type));
	cv::Mat buf(1, _buf->rows * _buf->cols * CV_ELEM_SIZE(_buf->type), CV_8U, _buf->data.ptr);
	return (IplImage*) custom_cv::imdecode_(buf, iscolor, custom_cv::LOAD_IMAGE);
}

CV_EXTERN_C CvMat
*

cvDecodeImageM(const CvMat* _buf, int iscolor) {
	CV_Assert(_buf && CV_IS_MAT_CONT(_buf->type));
	cv::Mat buf(1, _buf->rows * _buf->cols * CV_ELEM_SIZE(_buf->type), CV_8U, _buf->data.ptr);
	return (CvMat*) custom_cv::imdecode_(buf, iscolor, custom_cv::LOAD_CVMAT);
}

CV_EXTERN_C CvMat
*

cvEncodeImage(const char* ext, const CvArr* arr, const int* _params) {
	int i = 0;
	if (_params) {
		for (; _params[i] > 0; i += 2);
	}
	cv::Mat img = cv::cvarrToMat(arr);
	if (CV_IS_IMAGE(arr) && ((const IplImage*) arr)->origin == IPL_ORIGIN_BL) {
		cv::Mat temp;
		cv::flip(img, temp, 0);
		img = temp;
	}
	std::vector<uchar> buf;

	bool code = custom_cv::imencode(ext, img, buf,
	                                i > 0 ? std::vector<int>(_params, _params + i) : std::vector<int>());
	if (!code)
		return 0;
	CvMat * _buf = cvCreateMat(1, (int) buf.size(), CV_8U);
	memcpy(_buf->data.ptr, &buf[0], buf.size());

	return _buf;
}

/* End of file. */
