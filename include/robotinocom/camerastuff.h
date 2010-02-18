//  Copyright (C) 2004-2008, Robotics Equipment Corporation GmbH

#ifndef _CAMERASTUFF_H_
#define _CAMERASTUFF_H_

/**
* @brief	A collection of different possible resolution values.
*
* Supported are QVGA (320 x 240 pixel) and VGA (640 x 480 pixel).
*/
enum RobotinoResolution {QVGA,VGA,CustomResolution,ResolutionSize};

/**
* @brief	A collection of possible image compression rates.
*
* HighCompression: compress image with low quality resulting in very small jpeg images.
*
* LowCompression: compress image with high quality resulting in larger jpeg images.
*/
enum RobotinoCompression {HighCompression,LowCompression,NoCompression,CompressionSize};
enum RobotinoImageType {JPG,JPG2000,RAW,BMP,PNG,TIFF,UnknownImageType,ImageTypeSize};

/**
* @brief	A compact representation of all adjustable camera parameters.
* @see		RobotinoCom::cameraParameters, RobotinoCom::setCameraParameters
*/
class RobotinoCameraParameters
{
public:
	/**
	* Default constructor, that initializes all parameters to common values.
	*/
	RobotinoCameraParameters()
		: resolution( QVGA )
		, compression( LowCompression )
		, brightness( 50 )
		, contrast( 50 )
		, autoWithBalance( false )
	{
	}

	/// The resolution the camera images should have (if supported).
	RobotinoResolution resolution;
	/// The image compression that should be used before sending images.
	RobotinoCompression compression;

	/// A value defining the brightness of the image.
	unsigned char brightness;
	/// A value defining the contrast in the image.
	unsigned char contrast;
	/**
	* A flag wether to use automatic white balance or not.
	* @deprecated	Not used since most cameras don't support this feature.
	*/
	bool autoWithBalance;
};

/**
* @brief	A compact representation of all parameters of camera images.
* @see		RobotinoImage
*/
class RobotinoImageParameters
{
public:
	RobotinoImageParameters()
		: type( UnknownImageType )
		, resolution( CustomResolution )
		, width( 0 )
		, height( 0 )
		, numColorChannels( 0 )
		, bitsPerChannel( 0 )
	{
	}
	RobotinoImageType type;
	/// The resolution of the given image.
	RobotinoResolution resolution;

	/// Image width in pixel.
	unsigned int width;
	/// Image height in pixel.
	unsigned int height;

	/**
	* The number of color channels in this image.
	* Usually 1 for grayscale images or 3 for color images.
	*/
	unsigned char numColorChannels;
	/**
	* The number of bits a single color channel uses.
	* Usually 8 bit for RGB images.
	*/
	unsigned char bitsPerChannel;
};

/**
* @brief	A representation of camera images.
* @see		RobotinoCom::lockImage
*/
class RobotinoImage
{
public:
	RobotinoImage()
		: data( NULL )
		, dataSize( 0 )
		, timestamp( 0 )
	{
	}

	/// A set of parameters describing the image.
	RobotinoImageParameters parameters;

	/// The byte pointer to the image data in memory.
	unsigned char* data;
	/// The total size of the image data in bytes.
	unsigned int dataSize;

	/**
	* The timestamp of the images acquisition.
	* @see		RobotinoCom::msecsElapsed
	*/
	unsigned int timestamp;
};

#endif
