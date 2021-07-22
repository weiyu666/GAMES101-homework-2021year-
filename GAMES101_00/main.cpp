#include <iostream>
#include <opencv2/opencv.hpp>



int main1(void){
	// �����������ڣ��ֱ���ʾ����������ͼ��
	cv::namedWindow("Example2-5_in", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Example2-5_out", cv::WINDOW_AUTOSIZE);

	// ��ȡͼ�񣬲�������Ĵ�����ʾ����ͼ��
	cv::Mat img = cv::imread("C:\\Users\\��Ρ�\\Pictures\\����\\736461.png", -1);
	cv::imshow("Example2-5_in", img);

	// �����������
	cv::Mat out;

	// ����ƽ������������ʹ��GaussianBlur()��blur()��medianBlur()��bilateralFilter()
	// �˴�������������ģ������
	cv::GaussianBlur(img, out, cv::Size(5, 5), 3, 3);
	cv::GaussianBlur(out, out, cv::Size(5, 5), 3, 3);

	// �����������ʾ���ͼ��
	cv::imshow("Example2-5_out", out);
	// �ȴ������¼�
	cv::waitKey(0);

	// �رմ��ڲ��ͷ���������ڴ�ռ�
	cv::destroyAllWindows();

	return 0;
	
}