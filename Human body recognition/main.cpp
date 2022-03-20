#include <iostream>
#include <opencv2\imgproc.hpp>	//opencv头文件
#include <opencv2\calib3d.hpp>
#include <opencv2\highgui.hpp>
#include <Kinect.h>	//Kinect头文件
#include<math.h>

using   namespace   std;
using   namespace   cv;

int a = 0;
int b = 0;
int m = 7;
int p = 0;
int q = 0;
int c = 6;
int d = 6;
int f = 0;
int r = 0;
int s = 0;
int open = 0;
int mod = 0;
//****************************滑动条初值*****************************************
int v1 = 180, v2 = 300, v3 = 155, v4 = 133, v5 = 86, v6 = 222, v7 = 202, v8 = 158;

void    Drawtoback(Mat image, Mat background, int y, int x);
void    draw(Mat & img, Joint & r_1, Joint & r_2, ICoordinateMapper * myMapper);
void    Drawtofull(Mat image1, Mat  background1, int y1, int x1);


int main()
{
	namedWindow("contral", WINDOW_NORMAL);
	cvCreateTrackbar("v1往左变大", "contral", &v1, 300);
	cvCreateTrackbar("v2往左变长", "contral", &v2, 300);
	cvCreateTrackbar("v3往右左移", "contral", &v3, 300);
	cvCreateTrackbar("v4往左下移", "contral", &v4, 300);
	cvCreateTrackbar("v5往右变窄", "contral", &v5, 300);
	cvCreateTrackbar("v6往右变短", "contral", &v6, 300);
	cvCreateTrackbar("v7往左右移", "contral", &v7, 300);
	cvCreateTrackbar("v8往左下移", "contral", &v8, 300);
	

	IKinectSensor   * mySensor = nullptr;
	GetDefaultKinectSensor(&mySensor);
	mySensor->Open();

	IColorFrameSource   * myColorSource = nullptr;
	mySensor->get_ColorFrameSource(&myColorSource);

	IColorFrameReader   * myColorReader = nullptr;
	myColorSource->OpenReader(&myColorReader);

	int colorHeight = 0, colorWidth = 0;
	IFrameDescription   * myDescription = nullptr;
	myColorSource->get_FrameDescription(&myDescription);
	myDescription->get_Height(&colorHeight);
	myDescription->get_Width(&colorWidth);

	IColorFrame * myColorFrame = nullptr;
	Mat original(colorHeight, colorWidth, CV_8UC4);

	//**********************以上为ColorFrame的读取前准备**************************
	IDepthFrameSource   * myDepthSource = nullptr;
	mySensor->get_DepthFrameSource(&myDepthSource);

	IDepthFrameReader   * myDepthReader = nullptr;
	myDepthSource->OpenReader(&myDepthReader);

	int DepthHeight = 0, DepthWidth = 0;
	IFrameDescription   * myDepthDescription = nullptr;
	myDepthSource->get_FrameDescription(&myDepthDescription);
	myDepthDescription->get_Height(&DepthHeight);
	myDepthDescription->get_Width(&DepthWidth);

	IDepthFrame * myDepthFrame = nullptr;
	Mat depthImg(DepthHeight, DepthWidth, CV_8UC3);
	unsigned int bufferSize = DepthWidth * DepthHeight * sizeof(unsigned short);
	cv::Mat bufferMat(DepthHeight, DepthWidth, CV_16SC1);

	//**********************以上为DepthFrame的读取前准备**************************

	IBodyFrameSource    * myBodySource = nullptr;
	mySensor->get_BodyFrameSource(&myBodySource);

	IBodyFrameReader    * myBodyReader = nullptr;
	myBodySource->OpenReader(&myBodyReader);

	int myBodyCount = 0;
	myBodySource->get_BodyCount(&myBodyCount);

	int BodyHeight = 0, BodyWidth = 0;
	

	IBodyFrame  * myBodyFrame = nullptr;

	ICoordinateMapper   * myMapper = nullptr;
	mySensor->get_CoordinateMapper(&myMapper);

	//**********************以上为BodyFrame以及Mapper的准备***********************
	Mat src0 = imread("0.jpg");
	Mat tubiao1001 = imread("10.jpg");
	Mat tubiao1002 = imread("20.jpg");
	Mat tubiao1003 = imread("30.jpg");
	Mat tubiao1004 = imread("40.jpg");
	Mat tubiao1005 = imread("50.jpg");
    Mat tubiao1006 = imread("60.jpg");
	Mat tubiao1007 = imread("70.jpg");
	Mat tubiao1008 = imread("80.jpg");
	Mat tubiao1009 = imread("90.jpg");
	Mat tubiao1010 = imread("100.jpg");
	Mat tubiao1011 = imread("110.jpg");
	Mat tubiao1012 = imread("120.jpg");
	Mat tubiao1013 = imread("130.jpg");
	Mat tubiao1014 = imread("140.jpg");
	Mat tubiao1015 = imread("150.jpg");
	Mat tubiao1016 = imread("160.jpg");
	Mat tubiao1017 = imread("170.jpg");
	Mat tubiao1018 = imread("180.jpg");

	Mat no1=imread("0.jpg");
	Mat no2 = imread("0.jpg");

	//******************附加衣服颜色模板**************************
	Mat src10 = imread("10.jpg");
	Mat src11 = imread("11.jpg");
	Mat src12 = imread("12.jpg");

	Mat tubiao10 = imread("10.jpg");
	Mat tubiao11 = imread("11.jpg");
	Mat tubiao12 = imread("12.jpg");

	Mat src20 = imread("20.jpg");
	Mat src21 = imread("21.jpg");
	Mat src22 = imread("22.jpg");

	Mat tubiao20 = imread("20.jpg");
	Mat tubiao21 = imread("21.jpg");
	Mat tubiao22 = imread("22.jpg");

	Mat src30 = imread("30.jpg");
	Mat src31 = imread("31.jpg");
	Mat src32 = imread("32.jpg");

	Mat tubiao30 = imread("30.jpg");
	Mat tubiao31 = imread("31.jpg");
	Mat tubiao32 = imread("32.jpg");
	
	Mat src70 = imread("70.jpg");
	Mat src71 = imread("71.jpg");
	Mat src72 = imread("72.jpg");

	Mat tubiao70 = imread("70.jpg");
	Mat tubiao71 = imread("71.jpg");
	Mat tubiao72 = imread("72.jpg");

	Mat src80 = imread("80.jpg");
	Mat src81 = imread("81.jpg");
	Mat src82 = imread("82.jpg");

	Mat tubiao80 = imread("80.jpg");
	Mat tubiao81 = imread("81.jpg");
	Mat tubiao82 = imread("82.jpg");

	Mat src90 = imread("90.jpg");
	Mat src91 = imread("91.jpg");
	Mat src92 = imread("92.jpg");

	Mat tubiao90 = imread("90.jpg");
	Mat tubiao91 = imread("91.jpg");
	Mat tubiao92 = imread("92.jpg");

	Mat src130 = imread("130.jpg");
	Mat src131 = imread("131.jpg");
	Mat src132 = imread("132.jpg");

	Mat tubiao130 = imread("130.jpg");
	Mat tubiao131 = imread("131.jpg");
	Mat tubiao132 = imread("132.jpg");

	Mat src140 = imread("140.jpg");
	Mat src141 = imread("141.jpg");
	Mat src142 = imread("142.jpg");

	Mat tubiao140 = imread("140.jpg");
	Mat tubiao141 = imread("141.jpg");
	Mat tubiao142 = imread("142.jpg");

	Mat src150 = imread("150.jpg");
	Mat src151 = imread("151.jpg");
	Mat src152 = imread("152.jpg");

	Mat tubiao150 = imread("150.jpg");
	Mat tubiao151 = imread("151.jpg");
	Mat tubiao152 = imread("152.jpg");
	
	//***************附加裤子颜色模板****************************
	Mat src40 = imread("40.jpg");
	Mat src41 = imread("41.jpg");
	Mat src42 = imread("42.jpg");

	Mat tubiao40 = imread("40.jpg");
	Mat tubiao41 = imread("41.jpg");
	Mat tubiao42 = imread("42.jpg");

	Mat src50 = imread("50.jpg");
	Mat src51 = imread("51.jpg");
	Mat src52 = imread("52.jpg");

	Mat tubiao50 = imread("50.jpg");
	Mat tubiao51 = imread("51.jpg");
	Mat tubiao52 = imread("52.jpg");

	Mat src60 = imread("60.jpg");
	Mat src61 = imread("61.jpg");
	Mat src62 = imread("62.jpg");

	Mat tubiao60 = imread("60.jpg");
	Mat tubiao61 = imread("61.jpg");
	Mat tubiao62 = imread("62.jpg");

	Mat src100 = imread("100.jpg");
	Mat src101 = imread("101.jpg");
	Mat src102 = imread("102.jpg");

	Mat tubiao100 = imread("100.jpg");
	Mat tubiao101 = imread("101.jpg");
	Mat tubiao102 = imread("102.jpg");

	Mat src110 = imread("110.jpg");
	Mat src111 = imread("111.jpg");
	Mat src112 = imread("112.jpg");

	Mat tubiao110 = imread("110.jpg");
	Mat tubiao111 = imread("111.jpg");
	Mat tubiao112 = imread("112.jpg");

	Mat src120 = imread("120.jpg");
	Mat src121 = imread("121.jpg");
	Mat src122 = imread("122.jpg");

	Mat tubiao120 = imread("120.jpg");
	Mat tubiao121 = imread("121.jpg");
	Mat tubiao122 = imread("122.jpg");

	Mat src160 = imread("160.jpg");
	Mat src161 = imread("161.jpg");
	Mat src162 = imread("162.jpg");

	Mat tubiao160 = imread("160.jpg");
	Mat tubiao161 = imread("161.jpg");
	Mat tubiao162 = imread("162.jpg");

	Mat src170 = imread("170.jpg");
	Mat src171 = imread("171.jpg");
	Mat src172 = imread("172.jpg");

	Mat tubiao170 = imread("170.jpg");
	Mat tubiao171 = imread("171.jpg");
	Mat tubiao172 = imread("172.jpg");

	Mat src180 = imread("180.jpg");
	Mat src181 = imread("181.jpg");
	Mat src182 = imread("182.jpg");

	Mat tubiao180 = imread("180.jpg");
	Mat tubiao181 = imread("181.jpg");
	Mat tubiao182 = imread("182.jpg");


	//****************添加功能按键********************************
	Mat back = imread("fanhui.jpg");
	Mat lastpage = imread("lastpage.jpg");
	Mat nextpage = imread("nextpage.jpg");
	Mat yifu = imread("yifu.jpg");
	Mat kuzi = imread("kuzi.jpg");
	Mat man = imread("man.jpg");
	Mat wom = imread("wom.jpg");
	Mat dou = imread("dou.jpg");

	double juli, shengao, yaokuan, tuichang,shendu;

	while (1)
	{
		while (myColorReader->AcquireLatestFrame(&myColorFrame) != S_OK);
		myColorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * 4, original.data, ColorImageFormat_Bgra);
		Mat copy = original.clone();        //读取彩色图像并输出到矩阵


		while (myDepthReader->AcquireLatestFrame(&myDepthFrame) != S_OK);
		myDepthFrame->AccessUnderlyingBuffer(&bufferSize, reinterpret_cast<UINT16**>(&bufferMat.data));
		bufferMat.convertTo(depthImg, CV_8U, 255.0f / 4500.0f, 0.0f);//获取深度图像

	    //cout << "原图高：" << copy.size().height << "原图宽：" << copy.size().width << endl;

		resize(depthImg, depthImg, copy.size(), 0, 0, INTER_NEAREST);
		resize(tubiao1001, tubiao1001, Size(200, 200), 0, 0, INTER_NEAREST);
		resize(tubiao1002, tubiao1002, Size(200, 200), 0, 0, INTER_NEAREST);
		resize(tubiao1003, tubiao1003, Size(200, 200), 0, 0, INTER_NEAREST);
		resize(tubiao1004, tubiao1004, Size(200, 200), 0, 0, INTER_NEAREST);
		resize(tubiao1005, tubiao1005, Size(200, 200), 0, 0, INTER_NEAREST);
		resize(tubiao1006, tubiao1006, Size(200, 200), 0, 0, INTER_NEAREST);

		resize(tubiao1007, tubiao1007, Size(200, 200), 0, 0, INTER_NEAREST);
		resize(tubiao1008, tubiao1008, Size(200, 200), 0, 0, INTER_NEAREST);
		resize(tubiao1009, tubiao1009, Size(200, 200), 0, 0, INTER_NEAREST);
		resize(tubiao1010, tubiao1010, Size(200, 200), 0, 0, INTER_NEAREST);
		resize(tubiao1011, tubiao1011, Size(200, 200), 0, 0, INTER_NEAREST);
		resize(tubiao1012, tubiao1012, Size(200, 200), 0, 0, INTER_NEAREST);

		resize(tubiao1013, tubiao1013, Size(200, 200), 0, 0, INTER_NEAREST);
		resize(tubiao1014, tubiao1014, Size(200, 200), 0, 0, INTER_NEAREST);
		resize(tubiao1015, tubiao1015, Size(200, 200), 0, 0, INTER_NEAREST);
		resize(tubiao1016, tubiao1016, Size(200, 200), 0, 0, INTER_NEAREST);
		resize(tubiao1017, tubiao1017, Size(200, 200), 0, 0, INTER_NEAREST);
		resize(tubiao1018, tubiao1018, Size(200, 200), 0, 0, INTER_NEAREST);

		resize(yifu, yifu, Size(140, 90), 0, 0, INTER_NEAREST);
		resize(kuzi, kuzi, Size(140, 90), 0, 0, INTER_NEAREST);
		resize(back, back, Size(300, 90), 0, 0, INTER_NEAREST);
		resize(lastpage, lastpage, Size(240, 180), 0, 0, INTER_NEAREST);
		resize(nextpage, nextpage, Size(240, 180), 0, 0, INTER_NEAREST);
		resize(man, man, Size(240, 180), 0, 0, INTER_NEAREST);
		resize(wom, wom, Size(240, 180), 0, 0, INTER_NEAREST);
		resize(dou, dou, Size(240, 180), 0, 0, INTER_NEAREST);

		while (myBodyReader->AcquireLatestFrame(&myBodyFrame) != S_OK); //读取身体图像
		IBody   **  myBodyArr = new IBody *[myBodyCount];       //为存身体数据的数组做准备,分配空间
		for (int i = 0; i < myBodyCount; i++)
			myBodyArr[i] = nullptr;

		if (myBodyFrame->GetAndRefreshBodyData(myBodyCount, myBodyArr) == S_OK)     //把身体数据输入数组
			for (int i = 0; i < myBodyCount; i++)
			{
				//cout << myBodyCount << endl;
				BOOLEAN     result = false;
				if (myBodyArr[i]->get_IsTracked(&result) == S_OK && result) //先判断是否侦测到
				{
					Joint   myJointArr[JointType_Count];
					if (myBodyArr[i]->GetJoints(JointType_Count, myJointArr) == S_OK)   //如果侦测到就把关节数据输入到数组并画图
					{
						//draw(copy, myJointArr[JointType_Head], myJointArr[JointType_Neck], myMapper);
						//draw(copy, myJointArr[JointType_Neck], myJointArr[JointType_SpineShoulder], myMapper);

						//draw(copy, myJointArr[JointType_SpineShoulder], myJointArr[JointType_ShoulderLeft], myMapper);
						//draw(copy, myJointArr[JointType_SpineShoulder], myJointArr[JointType_SpineMid], myMapper);
						//draw(copy, myJointArr[JointType_SpineShoulder], myJointArr[JointType_ShoulderRight], myMapper);

						////draw(copy, myJointArr[JointType_ShoulderLeft], myJointArr[JointType_ElbowLeft], myMapper);
						//draw(copy, myJointArr[JointType_SpineMid], myJointArr[JointType_SpineBase], myMapper);
						////draw(copy, myJointArr[JointType_ShoulderRight], myJointArr[JointType_ElbowRight], myMapper);

						////draw(copy, myJointArr[JointType_ElbowLeft], myJointArr[JointType_WristLeft], myMapper);
						//draw(copy, myJointArr[JointType_SpineBase], myJointArr[JointType_HipLeft], myMapper);
						//draw(copy, myJointArr[JointType_SpineBase], myJointArr[JointType_HipRight], myMapper);
						////draw(copy, myJointArr[JointType_ElbowRight], myJointArr[JointType_WristRight], myMapper);

						////draw(copy, myJointArr[JointType_WristLeft], myJointArr[JointType_ThumbLeft], myMapper);
						////draw(copy, myJointArr[JointType_WristLeft], myJointArr[JointType_HandLeft], myMapper);
						//draw(copy, myJointArr[JointType_HipLeft], myJointArr[JointType_KneeLeft], myMapper);
						//draw(copy, myJointArr[JointType_HipRight], myJointArr[JointType_KneeRight], myMapper);
						////draw(copy, myJointArr[JointType_WristRight], myJointArr[JointType_ThumbRight], myMapper);
						////draw(copy, myJointArr[JointType_WristRight], myJointArr[JointType_HandRight], myMapper);

						////draw(copy, myJointArr[JointType_HandLeft], myJointArr[JointType_HandTipLeft], myMapper);
						////draw(copy, myJointArr[JointType_KneeLeft], myJointArr[JointType_FootLeft], myMapper);
						////draw(copy, myJointArr[JointType_KneeRight], myJointArr[JointType_FootRight], myMapper);
						////draw(copy, myJointArr[JointType_HandRight], myJointArr[JointType_HandTipRight], myMapper);

					}

					/*cout<<"肩宽："<<sqrt(
					pow(myJointArr[JointType_ShoulderLeft].Position.X - myJointArr[JointType_ShoulderRight].Position.X, 2) +
					pow(myJointArr[JointType_ShoulderLeft].Position.Y - myJointArr[JointType_ShoulderRight].Position.Y, 2) +
					pow(myJointArr[JointType_ShoulderLeft].Position.Z - myJointArr[JointType_ShoulderRight].Position.Z, 2))<< endl;
					cout << "上身高度："<<sqrt(
					pow(myJointArr[JointType_Neck].Position.X - myJointArr[JointType_SpineBase].Position.X, 2) +
					pow(myJointArr[JointType_Neck].Position.Y - myJointArr[JointType_SpineBase].Position.Y, 2) +
					pow(myJointArr[JointType_Neck].Position.Z - myJointArr[JointType_SpineBase].Position.Z, 2)) << endl;
					*/

					ColorSpacePoint point, point1, point2, point4, point3, point5, point6, point7, point8, point9, point10;
					//要把关节点用的摄像机坐标下的点（三维）转换成彩色空间（二维）的点
					Point   leftx, lefty, rightx, righty, upx, upy, lowx, lowy;
					Point	handleftx, handlefty, handrightx, handrighty;
					Point	hipleftx, hiplefty, hiprightx, hiprighty, kneeleftx, kneelefty, kneerightx, kneerighty;

					CameraSpacePoint headJoint = myJointArr[JointType_Head].Position;
					shendu = headJoint.Z;
					cout << "深度图距离      " << shendu << "m" << endl;

					myMapper->MapCameraPointToColorSpace(myJointArr[JointType_ShoulderLeft].Position, &point1);
					leftx.x = point1.X;
					lefty.y = point1.Y;
					
					myMapper->MapCameraPointToColorSpace(myJointArr[JointType_ShoulderRight].Position, &point2);
					rightx.x = point2.X;
					righty.y = point2.Y;

					myMapper->MapCameraPointToColorSpace(myJointArr[JointType_SpineShoulder].Position, &point3);
					upx.x = point3.X;
					upy.y = point3.Y;

					myMapper->MapCameraPointToColorSpace(myJointArr[JointType_SpineBase].Position, &point4);
					lowx.x = point4.X;
					lowy.y = point4.Y;

					myMapper->MapCameraPointToColorSpace(myJointArr[JointType_HandLeft].Position, &point5);
					handleftx.x = point5.X;
					handlefty.y = point5.Y;

					myMapper->MapCameraPointToColorSpace(myJointArr[JointType_HandRight].Position, &point6);
					handrightx.x = point6.X;
					handrighty.y = point6.Y;

					myMapper->MapCameraPointToColorSpace(myJointArr[JointType_HandRight].Position, &point6);
					handrightx.x = point6.X;
					handrighty.y = point6.Y;

					myMapper->MapCameraPointToColorSpace(myJointArr[JointType_HipLeft].Position, &point7);
					hipleftx.x = point7.X;
					hiplefty.y = point7.Y;

					myMapper->MapCameraPointToColorSpace(myJointArr[JointType_HipRight].Position, &point8);
					hiprightx.x = point8.X;
					hiprighty.y = point8.Y;

					myMapper->MapCameraPointToColorSpace(myJointArr[JointType_KneeLeft].Position, &point9);
					kneeleftx.x = point9.X;
					kneelefty.y = point9.Y;

					myMapper->MapCameraPointToColorSpace(myJointArr[JointType_KneeRight].Position, &point10);
					kneerightx.x = point10.X;
					kneerighty.y = point10.Y;

					if (shendu < 2)
						open = 1;
					if (shendu > 3.5)
					{
						open = 0;
						mod = 0;
					}
					if (open == 1) 
					{
						///模式选择
						if (mod == 0) {
							Drawtofull(man, copy, 80, 120);
							Drawtofull(wom, copy, 80, 335);
							Drawtofull(dou, copy, 80, 550);
							no1 = src0.clone();
							no2 = src0.clone();
							if (handleftx.x < 350 && handlefty.y < 335)
							{
								mod = 1;
							}
							else if (handleftx.x < 350 && handlefty.y>345 && handlefty.y <550)
							{
								mod = 2;
							}
							else if (handleftx.x < 350 && handlefty.y >560)
							{
								mod = 3;

							}

						}
						else {
							Drawtofull(back, copy, 560, 10);
							Drawtofull(lastpage, copy, 80, 120);
							Drawtofull(nextpage, copy, 80, 550);
							Drawtofull(yifu, copy, 410, 10);
							Drawtofull(kuzi, copy, 870, 10);
						}

			
////////////////////////////////////////////////////////////////男装//////////////////////////////////////////////////////////////////////
						if (mod == 1) {
							if (handleftx.x <550 && handlefty.y < 100)
							{
								f = 1;
							}
							if (handrightx.x > 870 && handrighty.y < 100)
							{
								f = 2;
							}
							if (handleftx.x < 840 && handleftx.x > 580 && handlefty.y < 100)
							{
								mod = 0;
							}
							if (handrightx.x < 840 && handrightx.x > 580 && handrighty.y < 100)
							{
								mod = 0;
							}

							//************************试衣代码**************************
							if (f == 1)
							{
						
								if (handleftx.x < 350 && handlefty.y < 400)
								{
									c--;
									q = 0;
								}
								else if (handleftx.x < 350 && handlefty.y >550)
								{
									c++;
									q = 0;

								}

								else if (handleftx.x < 840 && handleftx.x > 580 && handlefty.y < 100)
								{
									c = 0;
									q = 0;
									f = 0;
									mod = 0;

								}
								else if (handrightx.x < 840 && handrightx.x > 580 && handrighty.y < 100)
								{
									c = 0;
									q = 0;
									f = 0;
									mod = 0;
								}
								else { no1 = src0.clone(); }

								//********************************对c做一下限制*********
								if (c < 0)
								{
									c = 23;
								}
								if (c >= 24)
								{
									c = 7;
								}
								p = c / 6;


								if (p == 0) { no1 = src0.clone(); }

								else if (p == 1)
								{
									Drawtoback(tubiao1001, copy, 100, 325);
									resize(tubiao10, tubiao10, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao11, tubiao11, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao12, tubiao12, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao10, copy, 1100, 120);
									Drawtoback(tubiao11, copy, 1100, 320);
									Drawtoback(tubiao12, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										q = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										q = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										q = 3;
									}

									else { no1 = src0.clone(); }
								}



								else if (p == 2)
								{
									Drawtoback(tubiao1002, copy, 100, 325);
									resize(tubiao20, tubiao20, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao21, tubiao21, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao22, tubiao22, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao20, copy, 1100, 120);
									Drawtoback(tubiao21, copy, 1100, 320);
									Drawtoback(tubiao22, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										q = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										q = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										q = 3;
									}
									else { no1 = src0.clone(); }
								}


								else if (p == 3)
								{
									Drawtoback(tubiao1003, copy, 100, 325);
									resize(tubiao30, tubiao30, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao31, tubiao31, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao32, tubiao32, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao30, copy, 1100, 120);
									Drawtoback(tubiao31, copy, 1100, 320);
									Drawtoback(tubiao32, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										q = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										q = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										q = 3;
									}
									else { no1 = src0.clone(); }
								}

								switch (p)
								{
								case 1:
								{
									switch (q)
									{
									case 0: no1 = src0.clone(); break;
									case 1: no1 = src10.clone(); break;
									case 2: no1 = src11.clone(); break;
									case 3: no1 = src12.clone(); break;
									}
									break;
								}
								case 2:
								{
									switch (q)
									{
									case 0: no1 = src0.clone(); break;
									case 1: no1 = src20.clone(); break;
									case 2: no1 = src21.clone(); break;
									case 3: no1 = src22.clone(); break;
									}
									break;
								}
								case 3:
								{
									switch (q)
									{
									case 0: no1 = src0.clone(); break;
									case 1: no1 = src30.clone(); break;
									case 2: no1 = src31.clone(); break;
									case 3: no1 = src32.clone(); break;
									}
									break;
								}
								}

							}

							//************************试裤子代码************************
							else if (f == 2)
							{
							
								if (handleftx.x < 350 && handlefty.y < 400)
								{
									d--;
									s = 0;
								}
								else if (handleftx.x < 350 && handlefty.y >550)
								{
									d++;
									s = 0;

								}

								else if (handleftx.x < 840 && handleftx.x > 580 && handlefty.y < 100)
								{
									d = 0;
									s = 0;
									f = 0;
									mod = 0;
								}
								else if (handrightx.x < 840 && handrightx.x > 580 && handrighty.y < 100)
								{
									d = 0;
									s = 0;
									f = 0;
									mod = 0;

								}
								else { no2 = src0.clone(); }

								//*******对c做一下限制******//
								if (d < 0)
								{
									d = 23;
								}
								if (d >= 24)
								{
									d = 7;
								}
								r = d / 6;

								if (r == 0) { no2 = src0.clone(); }


								else if (r == 1)
								{
									Drawtoback(tubiao1004, copy, 100, 325);
									resize(tubiao40, tubiao40, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao41, tubiao41, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao42, tubiao42, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao40, copy, 1100, 120);
									Drawtoback(tubiao41, copy, 1100, 320);
									Drawtoback(tubiao42, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										s = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										s = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										s = 3;
									}
									else { no2 = src0.clone(); }
								}



								else if (r == 2)
								{
									Drawtoback(tubiao1005, copy, 100, 325);
									resize(tubiao50, tubiao50, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao51, tubiao51, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao52, tubiao52, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao50, copy, 1100, 120);
									Drawtoback(tubiao51, copy, 1100, 320);
									Drawtoback(tubiao52, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										s = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										s = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										s = 3;
									}
									else { no2 = src0.clone(); }
								}


								else if (r == 3)
								{
									Drawtoback(tubiao1006, copy, 100, 325);
									resize(tubiao60, tubiao60, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao61, tubiao61, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao62, tubiao62, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao60, copy, 1100, 120);
									Drawtoback(tubiao61, copy, 1100, 320);
									Drawtoback(tubiao62, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										s = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										s = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										s = 3;
									}
									else { no2 = src0.clone(); }
								}

								switch (r)
								{
								case 1:
								{
									switch (s)
									{
									case 0: no2 = src0.clone(); break;
									case 1: no2 = src40.clone(); break;
									case 2: no2 = src41.clone(); break;
									case 3: no2 = src42.clone(); break;
									}
									break;
								}
								case 2:
								{
									switch (s)
									{
									case 0: no2 = src0.clone(); break;
									case 1: no2 = src50.clone(); break;
									case 2: no2 = src51.clone(); break;
									case 3: no2 = src52.clone(); break;
									}
									break;
								}
								case 3:
								{
									switch (s)
									{
									case 0: no2 = src0.clone(); break;
									case 1: no2 = src60.clone(); break;
									case 2: no2 = src61.clone(); break;
									case 3: no2 = src62.clone(); break;
									}
									break;
								}
								}


							}

						}

////////////////////////////////////////////////////////////////女装//////////////////////////////////////////////////////////////////////
						else if (mod == 2) {
							if (handleftx.x <550 && handlefty.y < 100)
							{
								f = 1;
							}
							if (handrightx.x > 870 && handrighty.y < 100)
							{
								f = 2;
							}
							if (handleftx.x < 840 && handleftx.x > 580 && handlefty.y < 100)
							{
								mod = 0;
							}
							if (handrightx.x < 840 && handrightx.x > 580 && handrighty.y < 100)
							{
								mod = 0;
							}

							//************************试衣代码**************************
							if (f == 1)
							{
								
								if (handleftx.x < 350 && handlefty.y < 400)
								{
									c--;
									q = 0;
								}
								else if (handleftx.x < 350 && handlefty.y >550)
								{
									c++;
									q = 0;

								}

								else if (handleftx.x < 840 && handleftx.x > 580 && handlefty.y < 100)
								{
									c = 0;
									q = 0;
									f = 0;
									mod = 0;

								}
								else if (handrightx.x < 840 && handrightx.x > 580 && handrighty.y < 100)
								{
									c = 0;
									q = 0;
									f = 0;
									mod = 0;
								}
								else { no1 = src0.clone(); }

								//********************************对c做一下限制*********
								if (c < 0)
								{
									c = 23;
								}
								if (c >= 24)
								{
									c = 7;
								}
								p = c / 6;


								if (p == 0) { no1 = src0.clone(); }

								else if (p == 1)
								{
									Drawtoback(tubiao1007, copy, 100, 325);
									resize(tubiao70, tubiao70, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao71, tubiao71, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao72, tubiao72, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao70, copy, 1100, 120);
									Drawtoback(tubiao71, copy, 1100, 320);
									Drawtoback(tubiao72, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										q = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										q = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										q = 3;
									}

									else { no1 = src0.clone(); }
								}



								else if (p == 2)
								{
									Drawtoback(tubiao1008, copy, 100, 325);
									resize(tubiao80, tubiao80, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao81, tubiao81, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao82, tubiao82, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao80, copy, 1100, 120);
									Drawtoback(tubiao81, copy, 1100, 320);
									Drawtoback(tubiao82, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										q = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										q = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										q = 3;
									}
									else { no1 = src0.clone(); }
								}


								else if (p == 3)
								{
									Drawtoback(tubiao1009, copy, 100, 325);
									resize(tubiao90, tubiao90, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao91, tubiao91, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao92, tubiao92, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao90, copy, 1100, 120);
									Drawtoback(tubiao91, copy, 1100, 320);
									Drawtoback(tubiao92, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										q = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										q = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										q = 3;
									}
									else { no1 = src0.clone(); }
								}

								
								switch (p)
								{
								case 1:
								{
									switch (q)
									{
									case 0: no1 = src0.clone(); break;
									case 1: no1 = src70.clone(); break;
									case 2: no1 = src71.clone(); break;
									case 3: no1 = src72.clone(); break;
									}
									break;
								}
								case 2:
								{
									switch (q)
									{
									case 0: no1 = src0.clone(); break;
									case 1: no1 = src80.clone(); break;
									case 2: no1 = src81.clone(); break;
									case 3: no1 = src82.clone(); break;
									}
									break;
								}
								case 3:
								{
									switch (q)
									{
									case 0: no1 = src0.clone(); break;
									case 1: no1 = src90.clone(); break;
									case 2: no1 = src91.clone(); break;
									case 3: no1 = src92.clone(); break;
									}
									break;
								}
								}

							}

							//************************试裤子代码************************
							else if (f == 2)
							{
							
								if (handleftx.x < 350 && handlefty.y < 400)
								{
									d--;
									s = 0;
								}
								else if (handleftx.x < 350 && handlefty.y >550)
								{
									d++;
									s = 0;

								}

								else if (handleftx.x < 840 && handleftx.x > 580 && handlefty.y < 100)
								{
									d = 0;
									s = 0;
									f = 0;
									mod = 0;
								}
								else if (handrightx.x < 840 && handrightx.x > 580 && handrighty.y < 100)
								{
									d = 0;
									s = 0;
									f = 0;
									mod = 0;

								}
								else { no2 = src0.clone(); }

								//*******对c做一下限制******//
								if (d < 0)
								{
									d = 23;
								}
								if (d >= 24)
								{
									d = 7;
								}
								r = d / 6;

								if (r == 0) { no2 = src0.clone(); }


								else if (r == 1)
								{
									Drawtoback(tubiao1010, copy, 100, 325);
									resize(tubiao100, tubiao100, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao101, tubiao101, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao102, tubiao102, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao100, copy, 1100, 120);
									Drawtoback(tubiao101, copy, 1100, 320);
									Drawtoback(tubiao102, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										s = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										s = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										s = 3;
									}
									else { no2 = src0.clone(); }
								}



								else if (r == 2)
								{
									Drawtoback(tubiao1011, copy, 100, 325);
									resize(tubiao110, tubiao110, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao111, tubiao111, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao112, tubiao112, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao110, copy, 1100, 120);
									Drawtoback(tubiao111, copy, 1100, 320);
									Drawtoback(tubiao112, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										s = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										s = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										s = 3;
									}
									else { no2 = src0.clone(); }
								}


								else if (r == 3)
								{
									Drawtoback(tubiao1012, copy, 100, 325);
									resize(tubiao120, tubiao120, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao121, tubiao121, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao122, tubiao122, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao120, copy, 1100, 120);
									Drawtoback(tubiao121, copy, 1100, 320);
									Drawtoback(tubiao122, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										s = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										s = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										s = 3;
									}
									else { no2 = src0.clone(); }
								}

								switch (r)
								{
								case 1:
								{
									switch (s)
									{
									case 0: no2 = src0.clone(); break;
									case 1: no2 = src100.clone(); break;
									case 2: no2 = src101.clone(); break;
									case 3: no2 = src102.clone(); break;
									}
									break;
								}
								case 2:
								{
									switch (s)
									{
									case 0: no2 = src0.clone(); break;
									case 1: no2 = src110.clone(); break;
									case 2: no2 = src111.clone(); break;
									case 3: no2 = src112.clone(); break;
									}
									break;
								}
								case 3:
								{
									switch (s)
									{
									case 0: no2 = src0.clone(); break;
									case 1: no2 = src120.clone(); break;
									case 2: no2 = src121.clone(); break;
									case 3: no2 = src122.clone(); break;
									}
									break;
								}
								}


							}

						}
////////////////////////////////////////////////////////////////双人//////////////////////////////////////////////////////////////////////
						else if (mod == 3) {
							if (handleftx.x <550 && handlefty.y < 100)
							{
								f = 1;
							}
							if (handrightx.x > 870 && handrighty.y < 100)
							{
								f = 2;
							}
								if (handleftx.x < 840 && handleftx.x > 580 && handlefty.y < 100)
							{
								mod = 0;
							}
							if (handrightx.x < 840 && handrightx.x > 580 && handrighty.y < 100)
							{
								mod = 0;
							}


							//************************试衣代码**************************
							if (f == 1)
							{
							
								if (handleftx.x < 350 && handlefty.y < 400)
								{
									c--;
									q = 0;
								}
								else if (handleftx.x < 350 && handlefty.y >550)
								{
									c++;
									q = 0;

								}

								else if (handleftx.x < 840 && handleftx.x > 580 && handlefty.y < 100)
								{
									c = 0;
									q = 0;
									f = 0;
									mod = 0;

								}
								else if (handrightx.x < 840 && handrightx.x > 580 && handrighty.y < 100)
								{
									c = 0;
									q = 0;
									f = 0;
									mod = 0;
								}
								else { no1 = src0.clone(); }

								//********************************对c做一下限制*********
								if (c < 0)
								{
									c = 23;
								}
								if (c >= 24)
								{
									c = 7;
								}
								p = c / 6;


								if (p == 0) { no1 = src0.clone(); }

								else if (p == 1)
								{
									Drawtoback(tubiao1013, copy, 100, 325);
									resize(tubiao130, tubiao130, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao131, tubiao131, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao132, tubiao132, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao130, copy, 1100, 120);
									Drawtoback(tubiao131, copy, 1100, 320);
									Drawtoback(tubiao132, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										q = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										q = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										q = 3;
									}

									else { no1 = src0.clone(); }
								}



								else if (p == 2)
								{
									Drawtoback(tubiao1014, copy, 100, 325);
									resize(tubiao140, tubiao140, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao141, tubiao141, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao142, tubiao142, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao140, copy, 1100, 120);
									Drawtoback(tubiao141, copy, 1100, 320);
									Drawtoback(tubiao142, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										q = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										q = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										q = 3;
									}
									else { no1 = src0.clone(); }
								}


								else if (p == 3)
								{
									Drawtoback(tubiao1015, copy, 100, 325);
									resize(tubiao150, tubiao150, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao151, tubiao151, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao152, tubiao152, Size(200, 200), 0, 0, INTER_NEAREST);
								
									Drawtoback(tubiao150, copy, 1100, 120);
									Drawtoback(tubiao151, copy, 1100, 320);
									Drawtoback(tubiao152, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										q = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										q = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										q = 3;
									}
									else { no1 = src0.clone(); }
								}

								switch (p)
								{
								case 1:
								{
									switch (q)
									{
									case 0: no1 = src0.clone(); break;
									case 1: no1 = src130.clone(); break;
									case 2: no1 = src131.clone(); break;
									case 3: no1 = src132.clone(); break;
									}
									break;
								}
								case 2:
								{
									switch (q)
									{
									case 0: no1 = src0.clone(); break;
									case 1: no1 = src140.clone(); break;
									case 2: no1 = src141.clone(); break;
									case 3: no1 = src142.clone(); break;
									}
									break;
								}
								case 3:
								{
									switch (q)
									{
									case 0: no1 = src0.clone(); break;
									case 1: no1 = src150.clone(); break;
									case 2: no1 = src151.clone(); break;
									case 3: no1 = src152.clone(); break;
									}
									break;
								}
								}

							}

							//************************试裤子代码************************
							else if (f == 2)
							{  
							
								if (handleftx.x < 350 && handlefty.y < 400)
								{
									d--;
									s = 0;
								}
								else if (handleftx.x < 350 && handlefty.y >550)
								{
									d++;
									s = 0;

								}

								else if (handleftx.x < 840 && handleftx.x > 580 && handlefty.y < 100)
								{
									d = 0;
									s = 0;
									f = 0;
									mod = 0;
								}
								else if (handrightx.x < 840 && handrightx.x > 580 && handrighty.y < 100)
								{
									d = 0;
									s = 0;
									f = 0;
									mod = 0;

								}
								else { no2 = src0.clone(); }

								//*******对c做一下限制******//
								if (d < 0)
								{
									d = 23;
								}
								if (d >= 24)
								{
									d = 7;
								}
								r = d / 6;

								if (r == 0) { no2 = src0.clone(); }


								else if (r == 1)
								{
									Drawtoback(tubiao1016, copy, 100, 325);
									resize(tubiao160, tubiao160, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao161, tubiao161, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao162, tubiao162, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao160, copy, 1100, 120);
									Drawtoback(tubiao161, copy, 1100, 320);
									Drawtoback(tubiao162, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										s = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										s = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										s = 3;
									}
									else { no2 = src0.clone(); }
								}



								else if (r == 2)
								{
									Drawtoback(tubiao1017, copy, 100, 325);
									resize(tubiao170, tubiao170, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao171, tubiao171, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao172, tubiao172, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao170, copy, 1100, 120);
									Drawtoback(tubiao171, copy, 1100, 320);
									Drawtoback(tubiao172, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										s = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										s = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										s = 3;
									}
									else { no2 = src0.clone(); }
								}


								else if (r == 3)
								{
									Drawtoback(tubiao1018, copy, 100, 325);
									resize(tubiao180, tubiao180, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao181, tubiao181, Size(200, 200), 0, 0, INTER_NEAREST);
									resize(tubiao182, tubiao182, Size(200, 200), 0, 0, INTER_NEAREST);
									
									Drawtoback(tubiao180, copy, 1100, 120);
									Drawtoback(tubiao181, copy, 1100, 320);
									Drawtoback(tubiao182, copy, 1100, 520);
									
									if (handrightx.x > 1100 && handrighty.y < 315)
									{
										s = 1;
									}
									else if (handrightx.x > 1100 && handrighty.y > 325 && handrighty.y < 515)
									{
										s = 2;
									}
									else if (handrightx.x > 1100 && handrighty.y > 525)
									{
										s = 3;
									}
									else { no2 = src0.clone(); }
								}

								switch (r)
								{
								case 1:
								{
									switch (s)
									{
									case 0: no2 = src0.clone(); break;
									case 1: no2 = src160.clone(); break;
									case 2: no2 = src161.clone(); break;
									case 3: no2 = src162.clone(); break;
									}
									break;
								}
								case 2:
								{
									switch (s)
									{
									case 0: no2 = src0.clone(); break;
									case 1: no2 = src170.clone(); break;
									case 2: no2 = src171.clone(); break;
									case 3: no2 = src172.clone(); break;
									}
									break;
								}
								case 3:
								{
									switch (s)
									{
									case 0: no2 = src0.clone(); break;
									case 1: no2 = src180.clone(); break;
									case 2: no2 = src181.clone(); break;
									case 3: no2 = src182.clone(); break;
									}
									break;
								}
								}


							}

						}
					/*	else
						{
							no1 = src0.clone();
							no2 = src0.clone();
						}*/
//*********************最后处理操作*******************************************************************************
				if (mod != 0) {
					juli = rightx.x - leftx.x;
					shengao = lowy.y - upy.y;
					yaokuan = hiprightx.x - hipleftx.x;
					tuichang = kneelefty.y - lowy.y;
					Mat no3 = no1.clone();
					Mat no4 = no2.clone();

					int x1, y1;
					int x2, y2;
					//衣服与人体匹配
					/*//double rat1, rat2;
					//rat1 = (juli / 180);
					//rat2 = (shengao / 289);
					//if(rat1!=0&&rat2!=0)
					//resize(no3, no3, Size(0, 0), rat1, rat2, INTER_NEAREST);
					//x1 = (int)(leftx.x - 140 * (juli / 180));
					//y1 = (int)(lefty.y - 155 * (shengao / 289));*/

					double rat1, rat2;
					rat1 = (juli / v1);
					rat2 = (shengao / v2);
					if (rat1 != 0 && rat2 != 0)
					{
						resize(no3, no3, Size(0, 0), rat1, rat2, INTER_NEAREST);
					}
					x1 = (int)(leftx.x - v3 * rat1);
					y1 = (int)(lefty.y - v4 * rat2);

					//imshow("no3", no3);

					//裤子与人体匹配
				/*	double rat3, rat4;
					rat3 = (yaokuan / 87);
					rat4 = (tuichang / 194);
					if (rat3 != 0 && rat4 != 0)
					resize(no4, no4, Size(0, 0), rat3, rat4, INTER_NEAREST);
					x2 = (int)(hipleftx.x - 204 * (yaokuan / 87));
					y2 = (int)(hiplefty.y - 154 * (tuichang / 194));*/
					double rat3, rat4;
					rat3 = (yaokuan / v5);
					rat4 = (tuichang / v6);
					if (rat3 != 0 && rat4 != 0)
					{
						resize(no4, no4, Size(0, 0), rat3, rat4, INTER_NEAREST);
					}
					x2 = (int)(hipleftx.x - v7 * rat3);
					y2 = (int)(hiplefty.y - v8 * rat4);
					//imshow("no4", no4);

					Drawtoback(no4, copy, x2, y2);
					Drawtoback(no3, copy, x1, y1);
					
					}
					}
					//imshow("TEST1", copy);
				}

			}
		delete[] myBodyArr;

		myBodyFrame->Release();
		myColorFrame->Release();
		myDepthFrame->Release();

		imshow("Depth", depthImg);
		imshow("TEST", copy);

		if (waitKey(30) == VK_ESCAPE)
			break;
	}



	myMapper->Release();

	myDescription->Release();
	myColorReader->Release();
	myColorSource->Release();

	myDepthDescription->Release();
	myDepthReader->Release();
	myDepthSource->Release();

	myBodyReader->Release();
	myBodySource->Release();
	mySensor->Close();
	mySensor->Release();

	return  0;
}

void    draw(Mat & img, Joint & r_1, Joint & r_2, ICoordinateMapper * myMapper)//因为void没有返回值，所以用@可以通过形参改变实参
{
	//用两个关节点来做线段的两端，并且进行状态过滤
	if (r_1.TrackingState == TrackingState_Tracked && r_2.TrackingState == TrackingState_Tracked)
	{
		ColorSpacePoint t_point;    //要把关节点用的摄像机坐标下的点转换成彩色空间的点
		Point   p_1, p_2;
		myMapper->MapCameraPointToColorSpace(r_1.Position, &t_point);
		p_1.x = t_point.X;
		p_1.y = t_point.Y;
		myMapper->MapCameraPointToColorSpace(r_2.Position, &t_point);
		p_2.x = t_point.X;
		p_2.y = t_point.Y;

		line(img, p_1, p_2, Vec3b(0, 255, 0), 5);
		circle(img, p_1, 10, Vec3b(255, 0, 0), -1);
		circle(img, p_2, 10, Vec3b(255, 0, 0), -1);
	}
}
void Drawtoback(Mat image, Mat  background, int y, int x)
{
	for (int i = 0; i < image.rows; i++)
	{
		for (int j = 0; j < image.cols; j++)
		{
			int a = image.at<Vec3b>(i, j)[2];
			int b = image.at<Vec3b>(i, j)[1];
			int c = image.at<Vec3b>(i, j)[0];
			if (!(a >235 && b > 235 && c > 235))//opencv中 bgr   原图rgb
			{
				background.at<Vec4b>(i + x, j + y)[2] = image.at<Vec3b>(i, j)[2];
				background.at<Vec4b>(i + x, j + y)[1] = image.at<Vec3b>(i, j)[1];
				background.at<Vec4b>(i + x, j + y)[0] = image.at<Vec3b>(i, j)[0];
			}
		}
	}
}

void Drawtofull(Mat image1, Mat  background1, int y1, int x1)
{
	for (int i1 = 0; i1< image1.rows; i1++)
	{
		for (int j1 = 0; j1 < image1.cols; j1++)
		{
			int a1 = image1.at<Vec3b>(i1, j1)[2];
			int b1 = image1.at<Vec3b>(i1, j1)[1];
			int c1 = image1.at<Vec3b>(i1, j1)[0];
			if (!(a1 <0 && b1 <0 && c1 <0))//opencv中 bgr   原图rgb
			{
				background1.at<Vec4b>(i1 + x1, j1 + y1)[2] = image1.at<Vec3b>(i1, j1)[2];
				background1.at<Vec4b>(i1 + x1, j1 + y1)[1] = image1.at<Vec3b>(i1, j1)[1];
				background1.at<Vec4b>(i1 + x1, j1 + y1)[0] = image1.at<Vec3b>(i1, j1)[0];
			}
		}
	}
}