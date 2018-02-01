#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <thread>
#include <Windows.h>
#include <opencv2\opencv.hpp>

#include "examples\Protonect.h"

/// [headers]
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

/// [headers]
#ifdef EXAMPLES_WITH_OPENGL_SUPPORT
#include "viewer.h"
#endif

using namespace std;
using namespace cv;

bool protonect_shutdown = false; ///< Whether the running application should shut down.
void Bitmap1(unsigned char * pData, int width, int height, const char * filename);

void Bitmap1(unsigned char * pData, int width, int height, const char * filename)
{
	int size = width*height * 4; // 每个像素点4个字节

	// 位图第一部分，文件信息
	BITMAPFILEHEADER bfh;
	bfh.bfType = 0X4d42;  //bm
	bfh.bfSize = size  // 数据大小
		+ sizeof(BITMAPFILEHEADER) // first section size
		+ sizeof(BITMAPINFOHEADER) // second section size
		;
	bfh.bfReserved1 = 0; // reserved
	bfh.bfReserved2 = 0; // reserved
	bfh.bfOffBits = bfh.bfSize - size;

	// 位图第二部分，数据信息
	BITMAPINFOHEADER bih;
	bih.biSize = sizeof(BITMAPINFOHEADER);
	bih.biWidth = width;
	bih.biHeight = -height;
	bih.biPlanes = 1;
	bih.biBitCount = 32;
	bih.biCompression = 0;
	bih.biSizeImage = size;
	bih.biXPelsPerMeter = 0;
	bih.biYPelsPerMeter = 0;
	bih.biClrUsed = 0;
	bih.biClrImportant = 0;

	FILE * fp = fopen(filename, "wb");
	if (!fp) return;

	fwrite(&bfh, 1, sizeof(BITMAPFILEHEADER), fp);
	fwrite(&bih, 1, sizeof(BITMAPINFOHEADER), fp);
	fwrite(pData, 1, size, fp);
	fclose(fp);
}

void sigint_handler(int s)
{
	protonect_shutdown = true;
}

bool protonect_paused = false;
bool protonect_paused1 = false;
libfreenect2::Freenect2Device *devtopause;
libfreenect2::Freenect2Device *devtopause1;
libfreenect2::Freenect2 freenect2;
libfreenect2::PacketPipeline *pipeline = 0;
std::string serial = "";

bool viewer_enabled = true;
bool enable_rgb = true;
bool enable_depth = true;
int deviceId = -1;
size_t framemax = -1;
libfreenect2::Freenect2Device *dev = 0;
libfreenect2::Freenect2Device *dev1 = 0;
libfreenect2::Freenect2Device *dev2 = 0;
libfreenect2::Freenect2Device *dev3 = 0;
libfreenect2::Freenect2Device *dev4 = 0;
//Doing non-trivial things in signal handler is bad. If you want to pause,
//do it in another thread.
//Though libusb operations are generally thread safe, I cannot guarantee
//everything above is thread safe when calling start()/stop() while
//waitForNewFrame().
void sigusr1_handler(int s)
{
	if (devtopause == 0)
		return;
	/// [pause]
	if (protonect_paused)
		devtopause->start();
	else
		devtopause->stop();
	protonect_paused = !protonect_paused;
	/// [pause]
}
void sigusr1_handler1(int s)
{
	if (devtopause1 == 0)
		return;
	/// [pause]
	if (protonect_paused1)
		devtopause1->start();
	else
		devtopause1->stop();
	protonect_paused1 = !protonect_paused1;
	/// [pause]
}
//The following demostrates how to create a custom logger
/// [logger]

#include <cstdlib>
class MyFileLogger : public libfreenect2::Logger
{
private:
	std::ofstream logfile_;
public:
	MyFileLogger(const char *filename)
	{
		if (filename)
			logfile_.open(filename);
		level_ = Debug;
	}
	bool good()
	{
		return logfile_.is_open() && logfile_.good();
	}
	virtual void log(Level level, const std::string &message)
	{
		logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
	}
};
int thread0()
{
	libfreenect2::PacketPipeline *pipeline = 0;
	std::string serial1 = "";
	/*FILE *fp;
	char dpath[] = "";
	char cpath[] = "";*/
	structcd  cd0;
	//list<structcd> listcd0;
	bool viewer_enabled1 = true;
	bool enable_rgb1 = true;
	bool enable_depth1 = true;
	int deviceId1 = -1;
	size_t framemax1 = -1;
	if (!enable_rgb1 && !enable_depth1)
	{
		std::cerr << "Disabling both streams is not allowed!" << std::endl;
		return -1;
	}
	dev1 = freenect2.openDevice(0);
	if (dev1 == 0)
	{
		std::cout << "failure opening device!" << std::endl;
		return -1;
	}

	devtopause1 = dev1;

	signal(SIGINT, sigint_handler);
#ifdef SIGUSR1
	signal(SIGUSR1, sigusr1_handler1);
#endif
	protonect_shutdown = false;

	/// [listeners]
	int types = 0;
	if (enable_rgb1)
		types |= libfreenect2::Frame::Color;
	if (enable_depth1)
		types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
	libfreenect2::SyncMultiFrameListener listener(types);
	libfreenect2::FrameMap frames;

	dev1->setColorFrameListener(&listener);
	dev1->setIrAndDepthFrameListener(&listener);
	/// [listeners]

	/// [start]
	if (enable_rgb1 && enable_depth1)
	{
		if (!dev1->start())
			return -1;
	}
	else
	{
		if (!dev1->startStreams(enable_rgb1, enable_depth1))
			return -1;
	}

	std::cout << "device serial: " << dev1->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev1->getFirmwareVersion() << std::endl;
	/// [start]

	/// [registration setup]
	libfreenect2::Registration* registration = new libfreenect2::Registration(dev1->getIrCameraParams(), dev1->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
	/// [registration setup]

	size_t framecount = 0;
	while (!protonect_shutdown && (framemax1 == (size_t)-1 || framecount < framemax1))
	{
		Sleep(1000);
		if (!listener.waitForNewFrame(frames, 10 * 1000)) // 10 sconds
		{
			std::cout << "timeout!" << std::endl;
			return -1;
		}
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		/// [loop start]

		if (enable_rgb1 && enable_depth1)
		{
			/// [registration]
			registration->apply(rgb, depth, &undistorted, &registered);
			const float* depth_data = (float*)depth->data;
			cd0.data = new unsigned char[1920 * 1080 * 4];
			cd0.depth_data = new float[512 * 424 * 4];

			memcpy(cd0.data, rgb->data, 1920 * 1080 * 4);
			memcpy(cd0.depth_data, depth_data, 512 * 424 * 4);

			mtx0.lock();
			listcd0.push_back(cd0);
			mtx0.unlock();
			std::cout << "Depth information    0-------0:" << depth_data[10000] << std::endl;
		}

		framecount++;
		if (!viewer_enabled1)
		{
			if (framecount % 100 == 0)
				std::cout << "The viewer is turned off. Received " << framecount << " frames. Ctrl-C to stop." << std::endl;
			listener.release(frames);
			continue;
		}
		listener.release(frames);
	}
	dev1->stop();
	dev1->close();
	delete registration;

	return 0;
}
int thread0_si()
{
	FILE *fp;
	char dpath[MAX_PATH] = "";
	char cpath[MAX_PATH] = "";
	char wpath[MAX_PATH] = "";
	structcd scd0;
	structdep de0;
	while (1)
	{
		Sleep(10);
		if (listcd0.size() > 0)
		{
			mtx0.lock();
			scd0.data = listcd0.front().data;
			scd0.depth_data = listcd0.front().depth_data;
			listcd0.pop_front();
			mtx0.unlock();
			SYSTEMTIME st;
			GetLocalTime(&st);
			sprintf(dpath, "D:\\0\\%04d%02d%02d-%02d%02d%02d%03d.data", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			sprintf(cpath, "D:\\0\\%04d%02d%02d-%02d%02d%02d%03d.bmp", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			sprintf(wpath, "D:\\0_jpg\\%04d%02d%02d-%02d%02d%02d%03d.jpg", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			fp = fopen(dpath, "wb");
			fwrite(scd0.depth_data, 512 * 424 * 4, 1, fp);
			fclose(fp);
			fp = NULL;
			Bitmap1(scd0.data, 1920, 1080, cpath);

			delete[] scd0.data;
			delete[] scd0.depth_data;
			strcpy(de0.cname, cpath);
			strcpy(de0.dname, dpath);
			strcpy(de0.wname, wpath);

			mtx0_0.lock();
			listdep0.push_back(de0);
			mtx0_0.unlock();
		}
		
	}
	return 0;


}
int thread0_jd()
{
	structdep dep0;
	FILE *fp;
	float*Data = new float[424 * 512 * 4];
	if (NULL != Data)
	{
		while (1)
		{
			if (!listdep0.empty())
			{
				strcpy(dep0.cname, "xx");
				strcpy(dep0.dname, "xx");
				strcpy(dep0.wname, "xx");
				mtx0_0.lock();
				strcpy(dep0.cname, listdep0.front().cname);
				strcpy(dep0.dname, listdep0.front().dname);
				strcpy(dep0.wname, listdep0.front().wname);
				listdep0.pop_front();
				mtx0_0.unlock();
				fp = fopen(dep0.dname, "rb");
				if (fp == NULL)
				{
					cout << dep0.dname << "xiancheng3 File open fail.....";
				}
				else
				{
					fread(Data, 424 * 512 * 4, 1, fp);
					fclose(fp);
					fp = NULL;
					int index = 0;
					for (int i = 16386; i < 1920*1080 - 16385; i++)
					{
						if (Data[i] < 1200 && Data[i] >0)
						{
							index++;
						}
						if (index > 25)
						{
							mtx0_1.lock();
							listdep0_0.push_back(dep0);
							mtx0_1.unlock();
							break;
						}
					}
				}
			}
			Sleep(1);
		}
		delete[] Data;
		Data = NULL;
	}
	return 0;
}
int thread0_lc()
{
	FILE *fp;
	structdep dep0;
	UINT16* Ddata = new UINT16[512 * 424 * 4];
	if (NULL != Ddata)
	{
		while (1)
		{
			if (!listdep0_0.empty())
			{
				strcpy(dep0.cname, "xx");
				strcpy(dep0.dname, "xx");
				strcpy(dep0.wname, "xx");
				mtx0_1.lock();
				strcpy(dep0.cname, listdep0_0.front().cname);
				strcpy(dep0.dname, listdep0_0.front().dname);
				strcpy(dep0.wname, listdep0_0.front().wname);
				//cout << "从链表拿出" << listdep1.front().dname << endl;
				listdep0_0.pop_front();
				//cout << "After process" << listdep1.size() << dep2.dname << endl;
				mtx0_1.unlock();
				//cout << dep2.dname << endl;
				fp = fopen(dep0.dname, "rb");
				if (fp == NULL)
				{
					cout << "xiancheng4 File open fail.....";
				}
				else
				{
					fread(Ddata, 424 * 512 * 4, 1, fp);
					fclose(fp);
					fp = NULL;
					Mat img = imread(dep0.cname);
					//cout << dep2.dname << endl;
					//cout << "有超限！！！" << endl;
					for (int i = 16386; i < 512*424 - 16385; i++)
					{
						int rows = i / 512;
						int cols = i % 512;
						if (Ddata[i] < 1200 && Ddata[i] > 0)
						{
							circle(img, Point(232 + cols * 3, (rows - 21) * 3), 1, Scalar(0, 0, 255), 12);
						}
					}
					imwrite(dep0.wname, img);
					Sleep(1);
				}
			}
		}
		delete[] Ddata;
		Ddata = NULL;
	}
	return 10004;
}
int thread1()
{
	libfreenect2::PacketPipeline *pipeline = 0;
	std::string serial1 = "";
	structcd cd1;
	bool viewer_enabled1 = true;
	bool enable_rgb1 = true;
	bool enable_depth1 = true;
	int deviceId1 = -1;
	size_t framemax1 = -1;
	if (!enable_rgb1 && !enable_depth1)
	{
		std::cerr << "Disabling both streams is not allowed!" << std::endl;
		return -1;
	}
	dev1 = freenect2.openDevice(1);
	if (dev1 == 0)
	{
		std::cout << "failure opening device!" << std::endl;
		return -1;
	}

	devtopause1 = dev1;

	signal(SIGINT, sigint_handler);
#ifdef SIGUSR1
	signal(SIGUSR1, sigusr1_handler1);
#endif
	protonect_shutdown = false;

	/// [listeners]
	int types = 0;
	if (enable_rgb1)
		types |= libfreenect2::Frame::Color;
	if (enable_depth1)
		types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
	libfreenect2::SyncMultiFrameListener listener(types);
	libfreenect2::FrameMap frames;

	dev1->setColorFrameListener(&listener);
	dev1->setIrAndDepthFrameListener(&listener);
	/// [listeners]

	/// [start]
	if (enable_rgb1 && enable_depth1)
	{
		if (!dev1->start())
			return -1;
	}
	else
	{
		if (!dev1->startStreams(enable_rgb1, enable_depth1))
			return -1;
	}

	std::cout << "device serial: " << dev1->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev1->getFirmwareVersion() << std::endl;
	/// [start]

	/// [registration setup]
	libfreenect2::Registration* registration = new libfreenect2::Registration(dev1->getIrCameraParams(), dev1->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
	/// [registration setup]

	size_t framecount = 0;
	while (!protonect_shutdown && (framemax1 == (size_t)-1 || framecount < framemax1))
	{
		Sleep(1000);
		if (!listener.waitForNewFrame(frames, 10 * 1000)) // 10 sconds
		{
			std::cout << "timeout!" << std::endl;
			return -1;
		}
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		/// [loop start]

		if (enable_rgb1 && enable_depth1)
		{
			/// [registration]
			registration->apply(rgb, depth, &undistorted, &registered);
			const float* depth_data = (float*)depth->data;
			std::cout << "Depth information    1-------1:" << depth_data[10000] << std::endl;
			cd1.data = new unsigned char[1920 * 1080 * 4];
			cd1.depth_data = new float[512 * 424 * 4];
			memcpy(cd1.data, rgb->data, 1920 * 1080 * 4);
			memcpy(cd1.depth_data, depth_data, 512 * 424 * 4);

			mtx1.lock();
			listcd1.push_back(cd1);
			mtx1.unlock();
		}

		framecount++;
		if (!viewer_enabled1)
		{
			if (framecount % 100 == 0)
				std::cout << "The viewer is turned off. Received " << framecount << " frames. Ctrl-C to stop." << std::endl;
			listener.release(frames);
			continue;
		}
		listener.release(frames);
	}
	dev1->stop();
	dev1->close();
	delete registration;

	return 0;
}
int thread1_si()
{
	FILE *fp;
	char dpath[MAX_PATH] = "";
	char cpath[MAX_PATH] = "";
	char wpath[MAX_PATH] = "";
	structcd scd1;
	structdep de1;
	while (1)
	{
		Sleep(10);
		if (listcd1.size() > 0)
		{
			mtx1.lock();
			scd1.data = listcd1.front().data;
			scd1.depth_data = listcd1.front().depth_data;
			listcd1.pop_front();
			mtx1.unlock();
			SYSTEMTIME st;
			GetLocalTime(&st);
			sprintf(dpath, "D:\\1\\%04d%02d%02d-%02d%02d%02d%03d.data", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			sprintf(cpath, "D:\\1\\%04d%02d%02d-%02d%02d%02d%03d.bmp", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			sprintf(wpath, "D:\\1_jpg\\%04d%02d%02d-%02d%02d%02d%03d.jpg", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			fp = fopen(dpath, "wb");
			fwrite(scd1.depth_data, 512 * 424 * 4, 1, fp);
			fclose(fp);
			fp = NULL;
			Bitmap1(scd1.data, 1920, 1080, cpath);

			delete[] scd1.data;
			delete[] scd1.depth_data;
			strcpy(de1.cname, cpath);
			strcpy(de1.dname, dpath);
			strcpy(de1.wname, wpath);

			mtx1_0.lock();
			listdep0.push_back(de1);
			mtx1_0.unlock();
		}

	}
	return 0;


}
int thread1_jd()
{
	structdep dep1;
	FILE *fp;
	float*Data = new float[424 * 512 * 4];
	if (NULL != Data)
	{
		while (1)
		{
			if (!listdep1.empty())
			{
				strcpy(dep1.cname, "xx");
				strcpy(dep1.dname, "xx");
				strcpy(dep1.wname, "xx");
				mtx1_0.lock();
				strcpy(dep1.cname, listdep0.front().cname);
				strcpy(dep1.dname, listdep0.front().dname);
				strcpy(dep1.wname, listdep0.front().wname);
				listdep1.pop_front();
				mtx1_0.unlock();
				fp = fopen(dep1.dname, "rb");
				if (fp == NULL)
				{
					cout << dep1.dname << "xiancheng3 File open fail.....";
				}
				else
				{
					fread(Data, 424 * 512 * 4, 1, fp);
					fclose(fp);
					fp = NULL;
					int index = 0;
					for (int i = 16386; i < 1920 * 1080 - 16385; i++)
					{
						if (Data[i] < 1200 && Data[i] >0)
						{
							index++;
						}
						if (index > 25)
						{
							mtx1_1.lock();
							listdep1_0.push_back(dep1);
							mtx1_1.unlock();
							break;
						}
					}
				}
			}
			Sleep(1);
		}
		delete[] Data;
		Data = NULL;
	}
	return 0;
}
int thread1_lc()
{
	FILE *fp;
	structdep dep1;
	float* Ddata = new float[512 * 424 * 4];
	if (NULL != Ddata)
	{
		while (1)
		{
			if (!listdep1_0.empty())
			{
				strcpy(dep1.cname, "xx");
				strcpy(dep1.dname, "xx");
				strcpy(dep1.wname, "xx");
				mtx1_1.lock();
				strcpy(dep1.cname, listdep1_0.front().cname);
				strcpy(dep1.dname, listdep1_0.front().dname);
				strcpy(dep1.wname, listdep1_0.front().wname);
				//cout << "从链表拿出" << listdep1.front().dname << endl;
				listdep1_0.pop_front();
				//cout << "After process" << listdep1.size() << dep2.dname << endl;
				mtx0_1.unlock();
				//cout << dep2.dname << endl;
				fp = fopen(dep1.dname, "rb");
				if (fp == NULL)
				{
					cout << "xiancheng4 File open fail.....";
				}
				else
				{
					fread(Ddata, 424 * 512 * 2, 1, fp);
					fclose(fp);
					fp = NULL;
					Mat img = imread(dep1.cname);
					//cout << dep2.dname << endl;
					//cout << "有超限！！！" << endl;
					for (int i = 16386; i < 512 * 424 - 16385; i++)
					{
						int rows = i / 512;
						int cols = i % 512;
						if (Ddata[i] < 1200 && Ddata[i] > 0)
						{
							circle(img, Point(232 + cols * 3, (rows - 21) * 3), 1, Scalar(0, 0, 255), 12);
						}
					}
					imwrite(dep1.wname, img);
					Sleep(1);
				}
			}
		}
		delete[] Ddata;
		Ddata = NULL;
	}
	return 10004;
}
int thread2()
{
	libfreenect2::PacketPipeline *pipeline = 0;
	std::string serial1 = "";

	structcd cd2;
	//list<structcd> listcd2;

	bool viewer_enabled1 = true;
	bool enable_rgb1 = true;
	bool enable_depth1 = true;
	int deviceId1 = -1;
	size_t framemax1 = -1;
	if (!enable_rgb1 && !enable_depth1)
	{
		std::cerr << "Disabling both streams is not allowed!" << std::endl;
		return -1;
}
	dev2 = freenect2.openDevice(2);
	if (dev2 == 0)
	{
		std::cout << "failure opening device!" << std::endl;
		return -1;
	}

	devtopause1 = dev2;

	signal(SIGINT, sigint_handler);
#ifdef SIGUSR1
	signal(SIGUSR1, sigusr1_handler1);
#endif
	protonect_shutdown = false;

	/// [listeners]
	int types = 0;
	if (enable_rgb1)
		types |= libfreenect2::Frame::Color;
	if (enable_depth1)
		types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
	libfreenect2::SyncMultiFrameListener listener(types);
	libfreenect2::FrameMap frames;

	dev2->setColorFrameListener(&listener);
	dev2->setIrAndDepthFrameListener(&listener);
	/// [listeners]

	/// [start]
	if (enable_rgb1 && enable_depth1)
	{
		if (!dev2->start())
			return -1;
	}
	else
	{
		if (!dev2->startStreams(enable_rgb1, enable_depth1))
			return -1;
	}

	std::cout << "device serial: " << dev2->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev2->getFirmwareVersion() << std::endl;
	/// [start]
	/// [registration setup]
	libfreenect2::Registration* registration = new libfreenect2::Registration(dev2->getIrCameraParams(), dev2->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
	/// [registration setup]

	size_t framecount = 0;
	while (!protonect_shutdown && (framemax1 == (size_t)-1 || framecount < framemax1))
	{
		Sleep(1000);
		if (!listener.waitForNewFrame(frames, 10 * 1000)) // 10 sconds
		{
			std::cout << "timeout!" << std::endl;
			return -1;
		}
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		/// [loop start]

		if (enable_rgb1 && enable_depth1)
		{
			/// [registration]
			registration->apply(rgb, depth, &undistorted, &registered);
			const float* depth_data = (float*)depth->data;
			std::cout << "Depth information    2-------2:" << depth_data[10000] << std::endl;
			cd2.data = new unsigned char[1920 * 1080 * 4];
			cd2.depth_data = new float[512 * 424 * 4];
			memcpy(cd2.data, rgb->data, 1920 * 1080 * 4);
			memcpy(cd2.depth_data, depth_data, 512 * 424 * 4);

			mtx2.lock();
			listcd2.push_back(cd2);
			mtx2.unlock();
		}

		framecount++;
		if (!viewer_enabled1)
		{
			if (framecount % 100 == 0)
				std::cout << "The viewer is turned off. Received " << framecount << " frames. Ctrl-C to stop." << std::endl;
			listener.release(frames);
			continue;
		}
		listener.release(frames);
	}
	dev1->stop();
	dev1->close();
	delete registration;

	return 0;
}
int thread2_si()
{
	FILE *fp;
	char dpath[MAX_PATH] = "";
	char cpath[MAX_PATH] = "";
	char wpath[MAX_PATH] = "";
	structcd scd2;
	structdep de2;
	while (1)
	{
		Sleep(10);
		if (listcd2.size() > 0)
		{
			mtx2.lock();
			scd2.data = listcd2.front().data;
			scd2.depth_data = listcd2.front().depth_data;
			listcd2.pop_front();
			mtx2.unlock();
			SYSTEMTIME st;
			GetLocalTime(&st);
			sprintf(dpath, "D:\\2\\%04d%02d%02d-%02d%02d%02d%03d.data", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			sprintf(cpath, "D:\\2\\%04d%02d%02d-%02d%02d%02d%03d.bmp", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			sprintf(wpath, "D:\\2_jpg\\%04d%02d%02d-%02d%02d%02d%03d.jpg", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			fp = fopen(dpath, "wb");
			fwrite(scd2.depth_data, 512 * 424 * 4, 1, fp);
			fclose(fp);
			fp = NULL;
			Bitmap1(scd2.data, 1920, 1080, cpath);

			delete[] scd2.data;
			delete[] scd2.depth_data;
			strcpy(de2.cname, cpath);
			strcpy(de2.dname, dpath);
			strcpy(de2.wname, wpath);

			mtx2_0.lock();
			listdep2.push_back(de2);
			mtx2_0.unlock();
		}

	}
	return 0;


}
int thread2_jd()
{
	structdep dep2;
	FILE *fp;
	float*Data = new float[424 * 512 * 4];
	if (NULL != Data)
	{
		while (1)
		{
			if (!listdep2.empty())
			{
				strcpy(dep2.cname, "xx");
				strcpy(dep2.dname, "xx");
				strcpy(dep2.wname, "xx");
				mtx2_0.lock();
				strcpy(dep2.cname, listdep2.front().cname);
				strcpy(dep2.dname, listdep2.front().dname);
				strcpy(dep2.wname, listdep2.front().wname);
				listdep2.pop_front();
				mtx2_0.unlock();
				fp = fopen(dep2.dname, "rb");
				if (fp == NULL)
				{
					cout << dep2.dname << "xiancheng3 File open fail.....";
				}
				else
				{
					fread(Data, 424 * 512 * 4, 1, fp);
					fclose(fp);
					fp = NULL;
					int index = 0;
					for (int i = 16386; i < 1920 * 1080 - 16385; i++)
					{
						if (Data[i] < 1200 && Data[i] >0)
						{
							index++;
						}
						if (index > 25)
						{
							mtx2_1.lock();
							listdep2_0.push_back(dep2);
							mtx2_1.unlock();
							break;
						}
					}
				}
			}
			Sleep(1);
		}
		delete[] Data;
		Data = NULL;
	}
	return 0;
}
int thread2_lc()
{
	FILE *fp;
	structdep dep2;
	float* Ddata = new float[512 * 424 * 4];
	if (NULL != Ddata)
	{
		while (1)
		{
			if (!listdep2_0.empty())
			{
				strcpy(dep2.cname, "xx");
				strcpy(dep2.dname, "xx");
				strcpy(dep2.wname, "xx");
				mtx2_1.lock();
				strcpy(dep2.cname, listdep2_0.front().cname);
				strcpy(dep2.dname, listdep2_0.front().dname);
				strcpy(dep2.wname, listdep2_0.front().wname);
				//cout << "从链表拿出" << listdep1.front().dname << endl;
				listdep2_0.pop_front();
				//cout << "After process" << listdep1.size() << dep2.dname << endl;
				mtx2_1.unlock();
				//cout << dep2.dname << endl;
				fp = fopen(dep2.dname, "rb");
				if (fp == NULL)
				{
					cout << "xiancheng4 File open fail.....";
				}
				else
				{
					fread(Ddata, 424 * 512 * 4, 1, fp);
					fclose(fp);
					fp = NULL;
					Mat img = imread(dep2.cname);
					//cout << dep2.dname << endl;
					//cout << "有超限！！！" << endl;
					for (int i = 16386; i < 512 * 424 - 16385; i++)
					{
						int rows = i / 512;
						int cols = i % 512;
						if (Ddata[i] < 1200 && Ddata[i] > 0)
						{
							circle(img, Point(232 + cols * 3, (rows - 21) * 3), 1, Scalar(0, 0, 255), 12);
						}
					}
					imwrite(dep2.wname, img);
					Sleep(1);
				}
			}
		}
		delete[] Ddata;
		Ddata = NULL;
	}
	//myKinect.Writeimg();
	//Sleep(1);
	return 10004;
}
int thread3()
{
	libfreenect2::PacketPipeline *pipeline = 0;
	std::string serial1 = "";
	//FILE *fp;
	//char dpath[] = "";
	//char cpath[] = "";

	structcd cd3;
	//list<structcd> listcd3;

	bool viewer_enabled1 = true;
	bool enable_rgb1 = true;
	bool enable_depth1 = true;
	int deviceId1 = -1;
	size_t framemax1 = -1;
	if (!enable_rgb1 && !enable_depth1)
	{
		std::cerr << "Disabling both streams is not allowed!" << std::endl;
		return -1;
	}
	dev3 = freenect2.openDevice(3);
	if (dev1 == 0)
	{
		std::cout << "failure opening device!" << std::endl;
		return -1;
	}

	devtopause1 = dev3;

	signal(SIGINT, sigint_handler);
#ifdef SIGUSR1
	signal(SIGUSR1, sigusr1_handler1);
#endif
	protonect_shutdown = false;

	/// [listeners]
	int types = 0;
	if (enable_rgb1)
		types |= libfreenect2::Frame::Color;
	if (enable_depth1)
		types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
	libfreenect2::SyncMultiFrameListener listener(types);
	libfreenect2::FrameMap frames;

	dev3->setColorFrameListener(&listener);
	dev3->setIrAndDepthFrameListener(&listener);
	/// [listeners]

	/// [start]
	if (enable_rgb1 && enable_depth1)
	{
		if (!dev3->start())
			return -1;
	}
	else
	{
		if (!dev3->startStreams(enable_rgb1, enable_depth1))
			return -1;
	}

	std::cout << "device serial: " << dev3->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev3->getFirmwareVersion() << std::endl;
	/// [start]

	/// [registration setup]
	libfreenect2::Registration* registration = new libfreenect2::Registration(dev3->getIrCameraParams(), dev3->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
	/// [registration setup]

	size_t framecount = 0;
	while (!protonect_shutdown && (framemax1 == (size_t)-1 || framecount < framemax1))
	{
		Sleep(1000);
		if (!listener.waitForNewFrame(frames, 10 * 1000)) // 10 sconds
		{
			std::cout << "timeout!" << std::endl;
			return -1;
		}
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		/// [loop start]

		if (enable_rgb1 && enable_depth1)
		{
			/// [registration]
			registration->apply(rgb, depth, &undistorted, &registered);
			const float* depth_data = (float*)depth->data;

			std::cout << "Depth information    3-------3:" << depth_data[10000] << std::endl;
			cd3.data = new unsigned char[1920 * 1080 * 4];
			cd3.depth_data = new float[512 * 424 * 4];

			memcpy(cd3.data, rgb->data, 1920 * 1080 * 4);
			memcpy(cd3.depth_data, depth->data, 512 * 424 * 4);

			mtx3.lock();
			listcd3.push_back(cd3);
			mtx3.unlock();
		}

		framecount++;
		if (!viewer_enabled1)
		{
			if (framecount % 100 == 0)
				std::cout << "The viewer is turned off. Received " << framecount << " frames. Ctrl-C to stop." << std::endl;
			listener.release(frames);
			continue;
		}
		listener.release(frames);
	}
	dev1->stop();
	dev1->close();
	delete registration;

	return 0;
}
int thread3_si()
{
	FILE *fp;
	char dpath[MAX_PATH] = "";
	char cpath[MAX_PATH] = "";
	char wpath[MAX_PATH] = "";
	structcd scd3;
	structdep de3;
	while (1)
	{
		Sleep(10);
		if (listcd3.size() > 0)
		{
			mtx3.lock();
			scd3.data = listcd3.front().data;
			scd3.depth_data = listcd3.front().depth_data;
			listcd3.pop_front();
			mtx3.unlock();
			SYSTEMTIME st;
			GetLocalTime(&st);
			sprintf(dpath, "D:\\3\\%04d%02d%02d-%02d%02d%02d%03d.data", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			sprintf(cpath, "D:\\3\\%04d%02d%02d-%02d%02d%02d%03d.bmp", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			sprintf(wpath, "D:\\3_jpg\\%04d%02d%02d-%02d%02d%02d%03d.jpg", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			fp = fopen(dpath, "wb");
			fwrite(scd3.depth_data, 512 * 424 * 4, 1, fp);
			fclose(fp);
			fp = NULL;
			Bitmap1(scd3.data, 1920, 1080, cpath);

			delete[] scd3.data;
			delete[] scd3.depth_data;
			strcpy(de3.cname, cpath);
			strcpy(de3.dname, dpath);
			strcpy(de3.wname, wpath);

			mtx3_0.lock();
			listdep3.push_back(de3);
			mtx3_0.unlock();
		}

	}
	return 0;


}
int thread3_jd()
{
	structdep dep3;
	FILE *fp;
	float*Data = new float[424 * 512 * 4];
	if (NULL != Data)
	{
		while (1)
		{
			if (!listdep3.empty())
			{
				strcpy(dep3.cname, "xx");
				strcpy(dep3.dname, "xx");
				strcpy(dep3.wname, "xx");
				mtx3_0.lock();
				strcpy(dep3.cname, listdep3.front().cname);
				strcpy(dep3.dname, listdep3.front().dname);
				strcpy(dep3.wname, listdep3.front().wname);
				listdep3.pop_front();
				mtx3_0.unlock();
				fp = fopen(dep3.dname, "rb");
				if (fp == NULL)
				{
					cout << dep3.dname << "xiancheng3 File open fail.....";
				}
				else
				{
					fread(Data, 424 * 512 * 4, 1, fp);
					fclose(fp);
					fp = NULL;
					int index = 0;
					for (int i = 16386; i < 1920 * 1080 - 16385; i++)
					{
						if (Data[i] < 1200 && Data[i] >0)
						{
							index++;
						}
						if (index > 25)
						{
							mtx3_1.lock();
							listdep3_0.push_back(dep3);
							mtx3_1.unlock();
							break;
						}
					}
				}
			}
			Sleep(1);
		}
		delete[] Data;
		Data = NULL;
	}
	return 0;
}
int thread3_lc()
{
	FILE *fp;
	structdep dep3;
	float* Ddata = new float[512 * 424 * 4];
	if (NULL != Ddata)
	{
		while (1)
		{
			if (!listdep3_0.empty())
			{
				strcpy(dep3.cname, "xx");
				strcpy(dep3.dname, "xx");
				strcpy(dep3.wname, "xx");
				mtx3_1.lock();
				strcpy(dep3.cname, listdep3_0.front().cname);
				strcpy(dep3.dname, listdep3_0.front().dname);
				strcpy(dep3.wname, listdep3_0.front().wname);
				//cout << "从链表拿出" << listdep1.front().dname << endl;
				listdep3_0.pop_front();
				//cout << "After process" << listdep1.size() << dep2.dname << endl;
				mtx3_1.unlock();
				//cout << dep2.dname << endl;
				fp = fopen(dep3.dname, "rb");
				if (fp == NULL)
				{
					cout << "xiancheng4 File open fail.....";
				}
				else
				{
					fread(Ddata, 424 * 512 * 4, 1, fp);
					fclose(fp);
					fp = NULL;
					Mat img = imread(dep3.cname);
					//cout << dep2.dname << endl;
					//cout << "有超限！！！" << endl;
					for (int i = 16386; i < 512 * 424 - 16385; i++)
					{
						int rows = i / 512;
						int cols = i % 512;
						if (Ddata[i] < 1200 && Ddata[i] > 0)
						{
							circle(img, Point(232 + cols * 3, (rows - 21) * 3), 1, Scalar(0, 0, 255), 12);
						}
					}
					imwrite(dep3.wname, img);
					Sleep(1);
				}
			}
		}
		delete[] Ddata;
		Ddata = NULL;
	}
	//myKinect.Writeimg();
	//Sleep(1);
	return 10004;
}
int thread4()
{
	libfreenect2::PacketPipeline *pipeline = 0;
	std::string serial1 = "";

	structcd cd4;
	//list<structcd> listcd4;
	bool viewer_enabled1 = true;
	bool enable_rgb1 = true;
	bool enable_depth1 = true;
	int deviceId1 = -1;
	size_t framemax1 = -1;
	if (!enable_rgb1 && !enable_depth1)
	{
		std::cerr << "Disabling both streams is not allowed!" << std::endl;
		return -1;
	}
	dev1 = freenect2.openDevice(4);
	if (dev1 == 0)
	{
		std::cout << "failure opening device!" << std::endl;
		return -1;
	}

	devtopause1 = dev1;

	signal(SIGINT, sigint_handler);
#ifdef SIGUSR1
	signal(SIGUSR1, sigusr1_handler1);
#endif
	protonect_shutdown = false;

	/// [listeners]
	int types = 0;
	if (enable_rgb1)
		types |= libfreenect2::Frame::Color;
	if (enable_depth1)
		types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
	libfreenect2::SyncMultiFrameListener listener(types);
	libfreenect2::FrameMap frames;

	dev1->setColorFrameListener(&listener);
	dev1->setIrAndDepthFrameListener(&listener);
	/// [listeners]

	/// [start]
	if (enable_rgb1 && enable_depth1)
	{
		if (!dev1->start())
			return -1;
	}
	else
	{
		if (!dev1->startStreams(enable_rgb1, enable_depth1))
			return -1;
	}

	std::cout << "device serial: " << dev1->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev1->getFirmwareVersion() << std::endl;
	/// [start]

	/// [registration setup]
	libfreenect2::Registration* registration = new libfreenect2::Registration(dev1->getIrCameraParams(), dev1->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
	/// [registration setup]

	size_t framecount = 0;
	while (!protonect_shutdown && (framemax1 == (size_t)-1 || framecount < framemax1))
	{
		Sleep(1000);
		if (!listener.waitForNewFrame(frames, 10 * 1000)) // 10 sconds
		{
			std::cout << "timeout!" << std::endl;
			return -1;
		}
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		/// [loop start]

		if (enable_rgb1 && enable_depth1)
		{
			/// [registration]
			registration->apply(rgb, depth, &undistorted, &registered);
			const float* depth_data = (float*)depth->data;


			cd4.data = new unsigned char[1920 * 1080 * 4];
			cd4.depth_data = new float[512 * 424 * 4];

			memcpy(cd4.data, rgb->data, 1920 * 1080 * 4);
			memcpy(cd4.depth_data, depth->data, 512 * 424 * 4);

			mtx4.lock();
			listcd4.push_back(cd4);
			mtx4.unlock();

		}

		framecount++;
		if (!viewer_enabled1)
		{
			if (framecount % 100 == 0)
				std::cout << "The viewer is turned off. Received " << framecount << " frames. Ctrl-C to stop." << std::endl;
			listener.release(frames);
			continue;
		}
		listener.release(frames);
	}
	dev1->stop();
	dev1->close();
	delete registration;

	return 0;
}
int thread4_si()
{
	FILE *fp;
	char dpath[MAX_PATH] = "";
	char cpath[MAX_PATH] = "";
	char wpath[MAX_PATH] = "";
	structcd scd4;
	structdep de4;
	while (1)
	{
		Sleep(10);
		if (listcd4.size() > 0)
		{
			mtx4.lock();
			scd4.data = listcd4.front().data;
			scd4.depth_data = listcd4.front().depth_data;
			listcd4.pop_front();
			mtx4.unlock();

			SYSTEMTIME st;
			GetLocalTime(&st);
			sprintf(dpath, "D:\\4\\%04d%02d%02d-%02d%02d%02d%03d.data", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			sprintf(cpath, "D:\\4\\%04d%02d%02d-%02d%02d%02d%03d.bmp", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			sprintf(wpath, "D:\\4_jpg\\%04d%02d%02d-%02d%02d%02d%03d.jpg", st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			fp = fopen(dpath, "wb");
			fwrite(scd4.depth_data, 512 * 424 * 4, 1, fp);
			fclose(fp);
			fp = NULL;
			Bitmap1(scd4.data, 1920, 1080, cpath);

			delete[] scd4.data;
			delete[] scd4.depth_data;
			strcpy(de4.cname, cpath);
			strcpy(de4.dname, dpath);
			strcpy(de4.wname, wpath);

			mtx4_0.lock();
			listdep4.push_back(de4);
			mtx4_0.unlock();
		}

	}
	return 0;


}
int thread4_jd()
{
	structdep dep4;
	FILE *fp;
	float*Data = new float[424 * 512 * 4];
	if (NULL != Data)
	{
		while (1)
		{
			if (!listdep4.empty())
			{
				strcpy(dep4.cname, "xx");
				strcpy(dep4.dname, "xx");
				strcpy(dep4.wname, "xx");
				mtx4_0.lock();
				strcpy(dep4.cname, listdep4.front().cname);
				strcpy(dep4.dname, listdep4.front().dname);
				strcpy(dep4.wname, listdep4.front().wname);
				listdep4.pop_front();
				mtx4_0.unlock();
				fp = fopen(dep4.dname, "rb");
				if (fp == NULL)
				{
					cout << dep4.dname << "xiancheng3 File open fail.....";
				}
				else
				{
					fread(Data, 424 * 512 * 4, 1, fp);
					fclose(fp);
					fp = NULL;
					int index = 0;
					for (int i = 16386; i < 1920 * 1080 - 16385; i++)
					{
						if (Data[i] < 1200 && Data[i] >0)
						{
							index++;
						}
						if (index > 25)
						{
							mtx4_1.lock();
							listdep4_0.push_back(dep4);
							mtx4_1.unlock();
							break;
						}
					}
				}
			}
			Sleep(1);
		}
		delete[] Data;
		Data = NULL;
	}
	return 0;
}
int thread4_lc()
{
	FILE *fp;
	structdep dep4;
	float* Ddata = new float[512 * 424 * 4];
	if (NULL != Ddata)
	{
		while (1)
		{
			if (!listdep4_0.empty())
			{
				strcpy(dep4.cname, "xx");
				strcpy(dep4.dname, "xx");
				strcpy(dep4.wname, "xx");
				mtx4_1.lock();
				strcpy(dep4.cname, listdep4_0.front().cname);
				strcpy(dep4.dname, listdep4_0.front().dname);
				strcpy(dep4.wname, listdep4_0.front().wname);
				//cout << "从链表拿出" << listdep1.front().dname << endl;
				listdep4_0.pop_front();
				//cout << "After process" << listdep1.size() << dep2.dname << endl;
				mtx4_1.unlock();
				//cout << dep2.dname << endl;
				fp = fopen(dep4.dname, "rb");
				if (fp == NULL)
				{
					cout << "xiancheng4 File open fail.....";
				}
				else
				{
					fread(Ddata, 424 * 512 * 4, 1, fp);
					fclose(fp);
					fp = NULL;
					Mat img = imread(dep4.cname);
					//cout << dep2.dname << endl;
					//cout << "有超限！！！" << endl;
					for (int i = 16386; i < 512 * 424 - 16385; i++)
					{
						int rows = i / 512;
						int cols = i % 512;
						if (Ddata[i] < 1200 && Ddata[i] > 0)
						{
							circle(img, Point(232 + cols * 3, (rows - 21) * 3), 1, Scalar(0, 0, 255), 12);
						}
					}
					imwrite(dep4.wname, img);
					Sleep(1);
				}
			}
		}
		delete[] Ddata;
		Ddata = NULL;
	}
	//myKinect.Writeimg();                                                                                                                                                                                                                                                                                                          
	//Sleep(1);
	return 10004;
}
int main()
{
	std::cout << "Kinect v2 num:" << freenect2.enumerateDevices() << std::endl;
	if (!enable_rgb && !enable_depth)
	{
		std::cerr << "Disabling both streams is not allowed!" << std::endl;
		return -1;
	}
	dev = freenect2.openDefaultDevice();
	if (dev == 0)
	{
		std::cout << "failure opening device!" << std::endl;
		return -1;
	}
	std::thread t1(thread0);
	std::thread t2(thread1);
	std::thread t3(thread2);
	std::thread t4(thread3);
	std::thread t5(thread4);
	std::thread t1_1(thread0_si);
	std::thread t2_1(thread1_si);
	std::thread t3_1(thread2_si);
	std::thread t4_1(thread3_si);
	std::thread t5_1(thread4_si);
	std::thread t1_2(thread0_jd);
	std::thread t2_2(thread1_jd);
	std::thread t3_2(thread2_jd);
	std::thread t4_2(thread3_jd);
	std::thread t5_2(thread4_jd);
	std::thread t1_3(thread0_lc);
	std::thread t2_3(thread1_lc);
	std::thread t3_3(thread2_lc);
	std::thread t4_3(thread3_lc);
	std::thread t5_3(thread4_lc);
	t1.join();
	t2.join();
	t5.join();
	t3.join();
	t4.join();
	t1_1.join();
	t2_1.join();
	t3_1.join();
	t4_1.join();
	t5_1.join();
	t1_2.join();
	t2_2.join();
	t3_2.join();
	t4_2.join();
	t5_2.join();
	t1_3.join();
	t2_3.join();
	t3_3.join();
	t4_3.join();
	t5_3.join();
	return 0;

}
