#include <iostream>
#include <opencv2/opencv.hpp>



int main1(void){
	// 创建两个窗口，分别显示输入和输出的图像
	cv::namedWindow("Example2-5_in", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Example2-5_out", cv::WINDOW_AUTOSIZE);

	// 读取图像，并用输入的窗口显示输入图像
	cv::Mat img = cv::imread("C:\\Users\\陈巍瑜\\Pictures\\背景\\736461.png", -1);
	cv::imshow("Example2-5_in", img);

	// 声明输出矩阵
	cv::Mat out;

	// 进行平滑操作，可以使用GaussianBlur()、blur()、medianBlur()或bilateralFilter()
	// 此处共进行了两次模糊操作
	cv::GaussianBlur(img, out, cv::Size(5, 5), 3, 3);
	cv::GaussianBlur(out, out, cv::Size(5, 5), 3, 3);

	// 在输出窗口显示输出图像
	cv::imshow("Example2-5_out", out);
	// 等待键盘事件
	cv::waitKey(0);

	// 关闭窗口并释放相关联的内存空间
	cv::destroyAllWindows();

	return 0;
	
}