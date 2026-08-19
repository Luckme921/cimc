#ifndef __BMP_UTIL__
#define __BMP_UTIL__

#include <stdio.h>
#include <string.h>
#include <vector>
#include "hpp/Camera.hpp"
#include "hpp/Processing.hpp"
#include <iostream>
#include <fstream>

namespace cs {


#pragma  pack(1)
typedef struct tagBITMAPFILEHEADER {
	short bfType;
	int bfSize;
	short bfReserved1;
	short bfReserved2;
	int bfOffBits;
} BITMAPFILEHEADER, *LPBITMAPFILEHEADER, *PBITMAPFILEHEADER;
typedef struct tagBITMAPINFOHEADER {
	int biSize;
	int biWidth;
	int biHeight;
	short biPlanes;
	short biBitCount;
	int biCompression;
	int biSizeImage;
	int biXPelsPerMeter;
	int biYPelsPerMeter;
	int biClrUsed;
	int biClrImportant;
} BITMAPINFOHEADER, *LPBITMAPINFOHEADER, *PBITMAPINFOHEADER;

typedef struct tagRGBQUAD {
	char rgbBlue;
	char rgbGreen;
	char rgbRed;
	char rgbReserved;
} RGBQUAD;
#pragma  pack()

static bool saveBmp(const char *data, int width, int height, int channel, const char *filename)
{
	BITMAPINFOHEADER pHeader; //定义信息头        
	pHeader.biSize = sizeof(BITMAPINFOHEADER);
	pHeader.biWidth = width;
	pHeader.biHeight = -height;
	pHeader.biPlanes = 1;
	pHeader.biBitCount = 8 * channel;
	pHeader.biCompression = 0;
	pHeader.biSizeImage = width * height * channel;
	pHeader.biXPelsPerMeter = 0;
	pHeader.biYPelsPerMeter = 0;
	pHeader.biClrUsed = 0;
	pHeader.biClrImportant = 0;
	
	FILE* fp = fopen(filename, "wb");
	if (fp)
	{
		BITMAPFILEHEADER fheader = { 0 };
		fheader.bfType = 'M' << 8 | 'B';
		int offset = sizeof(BITMAPINFOHEADER) + sizeof(BITMAPFILEHEADER);
		std::vector<RGBQUAD> quads;
		if (channel == 1)
		{
			quads.resize(256);
			for (int i = 0; i < 256; i++)                        //遍历源图像的颜色表并转换，然后写入目标文件
			{
				quads[i].rgbBlue = quads[i].rgbGreen = quads[i].rgbRed = i;
				quads[i].rgbReserved = 0;
			}
			offset += sizeof(RGBQUAD) * 256;
		}
		fheader.bfSize = offset + pHeader.biSizeImage;
		fheader.bfOffBits = offset;
		fwrite(&fheader, 1, sizeof(fheader), fp);
		fwrite(&pHeader, 1, sizeof(BITMAPINFOHEADER), fp);
		if (channel == 1)
		{
			fwrite(quads.data(), 1, sizeof(RGBQUAD) * 256, fp);
			fwrite(data, 1, pHeader.biSizeImage, fp);
		}
		else if (channel == 3)
		{
			//swap rgb to bgr
			std::vector<char> bgrData;
			bgrData.resize(pHeader.biSizeImage);
			char *dst_b= bgrData.data();
			char *dst_g = dst_b + 1;
			char *dst_r = dst_b + 2;
			const char *src_r = data;
			const char *src_g = data + 1;
			const char *src_b = data + 2;
			for (int i = 0; i < pHeader.biSizeImage; i += 3)
			{
				dst_r[i] = src_r[i];
				dst_g[i] = src_g[i];
				dst_b[i] = src_b[i];
			}
			fwrite(bgrData.data(), 1, pHeader.biSizeImage, fp);
		}
		else if (channel == 4)
		{
			//swap rgba to bgra
			std::vector<char> bgrData;
			bgrData.resize(pHeader.biSizeImage);
			char* dst_b = bgrData.data();
			char* dst_g = dst_b + 1;
			char* dst_r = dst_b + 2;
			char* dst_a = dst_b + 3;
			const char* src_r = data;
			const char* src_g = data + 1;
			const char* src_b = data + 2;
			const char* src_a = data + 3;
			for (int i = 0; i < pHeader.biSizeImage; i += 4)
			{
				dst_r[i] = src_r[i];
				dst_g[i] = src_g[i];
				dst_b[i] = src_b[i];
				dst_a[i] = src_a[i];
				//dst_a[i] = 10;
			}
			fwrite(bgrData.data(), 1, pHeader.biSizeImage, fp);
		}
		fclose(fp);
		return true;
	}
	return false;
}

static bool readBmp(char *bmpName, std::vector<char> &bgrData, int &width, int &height)
{
    FILE *fp;
    if ((fp = fopen(bmpName, "rb")) == NULL)  //以二进制的方式打开文件
    {
        return false;
    }
    if (fseek(fp, sizeof(BITMAPFILEHEADER), 0))  //跳过BITMAPFILEHEADE
    {
        return false;
    }
    BITMAPINFOHEADER header;
    fread(&header, sizeof(BITMAPINFOHEADER), 1, fp);   //从fp中读取BITMAPINFOHEADER信息到header中,同时fp的指针移动
    width = header.biWidth;
    height = header.biHeight;
    int channel = header.biBitCount / 8;
    int linebyte = width * channel;
    if (channel == 3)
    {
        bgrData.resize(channel * width * height);
        for (int i = height - 1; i >= 0; i--)
        {
            fread(bgrData.data() + linebyte * i , sizeof(char), linebyte, fp);
        }
    }
    fclose(fp);   
    return true;
}

static bool convertBgrToGray(std::vector<char> &bgrData, int width, int height, std::vector<char> &grayData)
{
    if (bgrData.size() == width * height * 3)
    {
        int size = width * height;
        grayData.resize(size);
        for (size_t i = 0; i < size; i++)
        {
            grayData[i] = (bgrData[3 * i] + bgrData[3 * i + 1] + bgrData[3 * i + 2]) / 3;
        }
        return true;
    }
    return false;
}

/********************************************************************************************************/
/*!
	\brief Save a RGB/RGBA image in PNG format.
	\param SVPNG_OUTPUT Output stream (by default using file descriptor).
	\param w Width of the image. (<16383)
	\param h Height of the image.
	\param img Image pixel data in 24-bit RGB or 32-bit RGBA format.
	\param alpha Whether the image contains alpha channel.
*/
static bool savePng(const char* pchFileName, const unsigned char* img, unsigned w, unsigned h, int alpha)
{
	FILE* fp = fopen(pchFileName, "wb");
	if (fp == NULL)
	{
		return false;
	}

	int iDataLen = 0;
	std::vector<unsigned char> objVectorData;

	//#define SVPNG_PUT(u) fputc(u, fp)
	//#define SVPNG_PUT(u) objVectorData.push_back(u)

	// 使用lambda表达式创建匿名函数
	auto SVPNG_PUT = [&objVectorData, &iDataLen](unsigned char ucD) -> void {
		objVectorData.push_back(ucD);
		iDataLen++;
	};

	/* CRC32 Table */
	static const unsigned t[] = { 0, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c };
	unsigned a = 1, b = 0, c, p = w * (alpha ? 4 : 3) + 1, x, y, i;   /* ADLER-a, ADLER-b, CRC, pitch */
	#define SVPNG_U8A(ua, l) for (i = 0; i < l; i++) SVPNG_PUT((ua)[i]);
	#define SVPNG_U32(u) do { SVPNG_PUT((u) >> 24); SVPNG_PUT(((u) >> 16) & 255); SVPNG_PUT(((u) >> 8) & 255); SVPNG_PUT((u) & 255); } while(0)
	#define SVPNG_U8C(u) do { SVPNG_PUT(u); c ^= (u); c = (c >> 4) ^ t[c & 15]; c = (c >> 4) ^ t[c & 15]; } while(0)
	#define SVPNG_U8AC(ua, l) for (i = 0; i < l; i++) SVPNG_U8C((ua)[i])
	#define SVPNG_U16LC(u) do { SVPNG_U8C((u) & 255); SVPNG_U8C(((u) >> 8) & 255); } while(0)
	#define SVPNG_U32C(u) do { SVPNG_U8C((u) >> 24); SVPNG_U8C(((u) >> 16) & 255); SVPNG_U8C(((u) >> 8) & 255); SVPNG_U8C((u) & 255); } while(0)
	#define SVPNG_U8ADLER(u) do { SVPNG_U8C(u); a = (a + (u)) % 65521; b = (b + a) % 65521; } while(0)
	#define SVPNG_BEGIN(s, l) do { SVPNG_U32(l); c = ~0U; SVPNG_U8AC(s, 4); } while(0)
	#define SVPNG_END() SVPNG_U32(~c)
	SVPNG_U8A("\x89PNG\r\n\32\n", 8);           /* Magic */
	SVPNG_BEGIN("IHDR", 13);                    /* IHDR chunk { */
	SVPNG_U32C(w); SVPNG_U32C(h);               /*   Width & Height (8 bytes) */
	SVPNG_U8C(8); SVPNG_U8C(alpha ? 6 : 2);     /*   Depth=8, Color=True color with/without alpha (2 bytes) */
	SVPNG_U8AC("\0\0\0", 3);                    /*   Compression=Deflate, Filter=No, Interlace=No (3 bytes) */
	SVPNG_END();                                /* } */
	SVPNG_BEGIN("IDAT", 2 + h * (5 + p) + 4);   /* IDAT chunk { */
	SVPNG_U8AC("\x78\1", 2);                    /*   Deflate block begin (2 bytes) */
	for (y = 0; y < h; y++) {                   /*   Each horizontal line makes a block for simplicity */
		SVPNG_U8C(y == h - 1);                  /*   1 for the last block, 0 for others (1 byte) */
		SVPNG_U16LC(p); SVPNG_U16LC(~p);        /*   Size of block in little endian and its 1's complement (4 bytes) */
		SVPNG_U8ADLER(0);                       /*   No filter prefix (1 byte) */
		for (x = 0; x < p - 1; x++, img++)
			SVPNG_U8ADLER(*img);                /*   Image pixel data */
	}
	SVPNG_U32C((b << 16) | a);                  /*   Deflate block end with adler (4 bytes) */
	SVPNG_END();                                /* } */
	SVPNG_BEGIN("IEND", 0); SVPNG_END();        /* IEND chunk {} */

	fwrite(objVectorData.data(), iDataLen, 1, fp);
	fclose(fp); 

	return true;
}

/********************************************************************************************************/

static bool saveFrameData(const char* saveDir, cs::IFramePtr frame, int index)
{
	char acPath[260];
	if (frame->getFormat() == STREAM_FORMAT_RGB8)
	{
		snprintf(acPath, sizeof(acPath), "%srgb%dx%d-%d-%d.bmp", saveDir, frame->getWidth(), frame->getHeight()
			, index, frame->getSize());

		if (cs::saveBmp(frame->getData(), frame->getWidth(), frame->getHeight(), 3, acPath))
		{
			//printf("save OK:%s\n", acPath);
		}
		else
		{
			printf("save bmp failed:%s\n", acPath);
		}
	}
	else if (frame->getFormat() == STREAM_FORMAT_RGBA)
	{
		snprintf(acPath, sizeof(acPath), "%srgba%dx%d-%d-%d.png", saveDir, frame->getWidth(), frame->getHeight()
			, index, frame->getSize());

		if (cs::savePng(acPath, (const unsigned char*)(frame->getData()), frame->getWidth(), frame->getHeight(), 1))
		{
			//printf("save OK:%s\n", acPath);
		}
		else
		{
			printf("save png failed:%s\n", acPath);
		}
	}
	else if (frame->getFormat() == STREAM_FORMAT_MJPG)
	{
		snprintf(acPath, sizeof(acPath), "%srgb%dx%d-%d.jpg", saveDir, frame->getWidth(), frame->getHeight(), index);

		FILE* lpf = fopen(acPath, "wb");
		if (lpf == NULL)
		{
			printf("open file failed:%s\n", acPath);
		}

		fwrite((unsigned char*)frame->getData(), 1, frame->getSize(), lpf);
		fclose(lpf);

		//printf("save OK:%s\n", acPath);
	}
	else if (frame->getFormat() == STREAM_FORMAT_Z16)
	{
		snprintf(acPath, sizeof(acPath), "%sz16-%dx%d-%d-%d.bmp", saveDir, frame->getWidth(), frame->getHeight()
			, index, frame->getSize());
		cs::colorizer color;
		int iLen = frame->getWidth()*frame->getHeight() * 2;
		if (frame->getSize() != iLen)
		{
			printf("the size is not match,format:%d,size:%d/%d\n", frame->getFormat(), frame->getSize(), iLen);
			return false;
		}

		std::vector<unsigned char> rgb;
		int length = frame->getHeight() * frame->getWidth();
		rgb.resize(length * 3);
		float scale = 0.1f;

		color.process((unsigned short *)frame->getData(FRAME_DATA_FORMAT_Z16), 0.1, rgb.data(), length);
		if (cs::saveBmp((const char *)rgb.data(), frame->getWidth(), frame->getHeight(), 3, acPath))
		{
			//printf("save OK:%s\n", acPath);
		}
		else
		{
			printf("save bmp failed:%s\n", acPath);
		}
	}
	else if (frame->getFormat() == STREAM_FORMAT_Z16Y8Y8)
	{
		//Z16Y8Y8包含一个z16数据，一个左红外数和一个右红外数据
		snprintf(acPath, sizeof(acPath), "%sz16y8y8-%dx%d-%d-%d.bmp", saveDir, frame->getWidth(), frame->getHeight()
			, index, frame->getSize());
		cs::colorizer color;
		//整体的数据长度为 宽*高*4，z16数据每个像素2字节，红外数据每个像素1字节，2+1+1。
		int iLen = frame->getWidth()*frame->getHeight() * 4;
		if (frame->getSize() != iLen)
		{
			printf("the size is not match,format:%d,size:%d/%d\n",  frame->getFormat(), frame->getSize(), iLen);
			return false;
		}

		std::vector<unsigned char> rgb;
		int length = frame->getHeight() * frame->getWidth();
		rgb.resize(length * 3);
		float scale = 0.1f;

		//getData(FRAME_DATA_FORMAT_Z16)获取Z16数据
		color.process((unsigned short *)frame->getData(FRAME_DATA_FORMAT_Z16), scale, rgb.data(), length);
		if (cs::saveBmp((const char *)rgb.data(), frame->getWidth(), frame->getHeight(), 3, acPath))
		{
			//printf("save OK:%s\n", acPath);
		}
		else
		{
			printf("save bmp failed:%s\n", acPath);
		}

		snprintf(acPath, sizeof(acPath), "%sdepth-%d-%dx%d-%d-%d_y8-left.bmp", saveDir, frame->getFormat()
			, frame->getWidth(), frame->getHeight()
			, index, frame->getSize()/2);

		//getData(FRAME_DATA_FORMAT_IR_LEFT)获取左红外数据
		cs::saveBmp((const char *)frame->getData(FRAME_DATA_FORMAT_IR_LEFT)
			, frame->getWidth(), frame->getHeight(), 1, acPath);

		snprintf(acPath, sizeof(acPath), "%sdepth-%d-%dx%d-%d-%d_y8-right.bmp", saveDir, frame->getFormat()
			, frame->getWidth(), frame->getHeight()
			, index, frame->getSize()/2);

		//getData(FRAME_DATA_FORMAT_IR_RIGHT)获取右红外数据
		cs::saveBmp((const char *)frame->getData(FRAME_DATA_FORMAT_IR_RIGHT)
			, frame->getWidth(), frame->getHeight(), 1, acPath);
	}
	else if (frame->getFormat() == STREAM_FORMAT_GRAY)
	{
		int iLen = frame->getWidth()*frame->getHeight() * 2;
	}
	else if (frame->getFormat() == STREAM_FORMAT_PAIR)
	{
		int iLen = frame->getWidth()*frame->getHeight();
		if (frame->getSize() != iLen*2)
		{
			printf("the size is not match,format:%d,size:%d/%d\n",  frame->getFormat(), frame->getSize(), iLen);
			return false;
		}

		snprintf(acPath, sizeof(acPath), "%spair-%dx%d-%d-%d-left.bmp", saveDir, frame->getWidth(), frame->getHeight()
			, index, frame->getSize()/2);
		
		cs::saveBmp((const char *)frame->getData(FRAME_DATA_FORMAT_IR_LEFT)
			, frame->getWidth(), frame->getHeight(), 1, acPath);

		snprintf(acPath, sizeof(acPath), "%spair-%dx%d-%d-%d-right.bmp", saveDir, frame->getWidth(), frame->getHeight()
			, index, frame->getSize()/2);

		cs::saveBmp((const char *)frame->getData(FRAME_DATA_FORMAT_IR_RIGHT)
			, frame->getWidth(), frame->getHeight(), 1, acPath);
		//printf("save Ok:%s\n", acPath);
	}
	else
	{
		printf("unknow format %d\n", frame->getFormat());
	}

	return true;
}


static bool saveFrameDataByTime(const char* saveDir, cs::IFramePtr frame)
{
	int iTimestamp = (int)(frame->getTimeStamp());

	char acPath[1024];
	if (frame->getFormat() == STREAM_FORMAT_RGB8)
	{
		snprintf(acPath, sizeof(acPath), "%s%d-rgb8.bmp", saveDir, iTimestamp);

		if (cs::saveBmp(frame->getData(), frame->getWidth(), frame->getHeight(), 3, acPath))
		{
			//printf("save OK:%s\n", acPath);
		}
		else
		{
			printf("save bmp failed:%s\n", acPath);
		}
	}
	else if (frame->getFormat() == STREAM_FORMAT_RGBA)
	{
		snprintf(acPath, sizeof(acPath), "%s%d-rgbA.bmp", saveDir, iTimestamp);

		if (cs::savePng(acPath, (const unsigned char*)(frame->getData()), frame->getWidth(), frame->getHeight(), 1))
		{
			//printf("save OK:%s\n", acPath);
		}
		else
		{
			printf("save png failed:%s\n", acPath);
		}
	}
	else if (frame->getFormat() == STREAM_FORMAT_MJPG)
	{
		snprintf(acPath, sizeof(acPath), "%s%d-Mjpg.jpg", saveDir, iTimestamp);

		FILE* lpf = fopen(acPath, "wb");
		if (lpf == NULL)
		{
			printf("open file failed:%s\n", acPath);
		}

		fwrite((unsigned char*)frame->getData(), 1, frame->getSize(), lpf);
		fclose(lpf);

		//printf("save OK:%s\n", acPath);
	}
	else if (frame->getFormat() == STREAM_FORMAT_Z16)
	{
		snprintf(acPath, sizeof(acPath), "%s%d-z16.bmp", saveDir, iTimestamp);

		cs::colorizer color;
		int iLen = frame->getWidth() * frame->getHeight() * 2;
		if (frame->getSize() != iLen)
		{
			printf("the size is not match,format:%d,size:%d/%d\n", frame->getFormat(), frame->getSize(), iLen);
			return false;
		}

		std::vector<unsigned char> rgb;
		int length = frame->getHeight() * frame->getWidth();
		rgb.resize(length * 3);
		float scale = 0.1f;

		color.process((unsigned short*)frame->getData(FRAME_DATA_FORMAT_Z16), 0.1, rgb.data(), length);
		if (cs::saveBmp((const char*)rgb.data(), frame->getWidth(), frame->getHeight(), 3, acPath))
		{
			//printf("save OK:%s\n", acPath);
		}
		else
		{
			printf("save bmp failed:%s\n", acPath);
		}
	}
	else if (frame->getFormat() == STREAM_FORMAT_Z16Y8Y8)
	{
		//Z16Y8Y8包含一个z16数据，一个左红外数和一个右红外数据
		snprintf(acPath, sizeof(acPath), "%s%d-z16y8y8.bmp", saveDir, iTimestamp);

		cs::colorizer color;
		//整体的数据长度为 宽*高*4，z16数据每个像素2字节，红外数据每个像素1字节，2+1+1。
		int iLen = frame->getWidth() * frame->getHeight() * 4;
		if (frame->getSize() != iLen)
		{
			printf("the size is not match,format:%d,size:%d/%d\n", frame->getFormat(), frame->getSize(), iLen);
			return false;
		}

		std::vector<unsigned char> rgb;
		int length = frame->getHeight() * frame->getWidth();
		rgb.resize(length * 3);
		float scale = 0.1f;

		//getData(FRAME_DATA_FORMAT_Z16)获取Z16数据
		color.process((unsigned short*)frame->getData(FRAME_DATA_FORMAT_Z16), scale, rgb.data(), length);
		if (cs::saveBmp((const char*)rgb.data(), frame->getWidth(), frame->getHeight(), 3, acPath))
		{
			//printf("save OK:%s\n", acPath);
		}
		else
		{
			printf("save bmp failed:%s\n", acPath);
		}

		snprintf(acPath, sizeof(acPath), "%s%d-z16y8y8-leftIR.bmp", saveDir, iTimestamp);

		//getData(FRAME_DATA_FORMAT_IR_LEFT)获取左红外数据
		cs::saveBmp((const char*)frame->getData(FRAME_DATA_FORMAT_IR_LEFT)
			, frame->getWidth(), frame->getHeight(), 1, acPath);

		snprintf(acPath, sizeof(acPath), "%s%d-z16y8y8-rightIR.bmp", saveDir, iTimestamp);

		//getData(FRAME_DATA_FORMAT_IR_RIGHT)获取右红外数据
		cs::saveBmp((const char*)frame->getData(FRAME_DATA_FORMAT_IR_RIGHT)
			, frame->getWidth(), frame->getHeight(), 1, acPath);
	}
	else if (frame->getFormat() == STREAM_FORMAT_GRAY)
	{
		int iLen = frame->getWidth() * frame->getHeight() * 2;
	}
	else if (frame->getFormat() == STREAM_FORMAT_PAIR)
	{
		int iLen = frame->getWidth() * frame->getHeight();
		if (frame->getSize() != iLen * 2)
		{
			printf("the size is not match,format:%d,size:%d/%d\n", frame->getFormat(), frame->getSize(), iLen);
			return false;
		}

		snprintf(acPath, sizeof(acPath), "%s%d-leftIR.bmp", saveDir, iTimestamp);

		cs::saveBmp((const char*)frame->getData(FRAME_DATA_FORMAT_IR_LEFT)
			, frame->getWidth(), frame->getHeight(), 1, acPath);

		snprintf(acPath, sizeof(acPath), "%s%d-rightIR.bmp", saveDir, iTimestamp);

		cs::saveBmp((const char*)frame->getData(FRAME_DATA_FORMAT_IR_RIGHT)
			, frame->getWidth(), frame->getHeight(), 1, acPath);
		//printf("save Ok:%s\n", acPath);
	}
	else
	{
		printf("unknow format %d\n", frame->getFormat());
	}

	return true;
}

};
#endif