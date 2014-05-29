#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>

//using namespace cv;

static const std::string OPENCV_WINDOW = "Image Window: Baxter Camera Image";


int global_win_width = 480; //640
int global_win_height = 300; //400
/*
int global_win_x1 = 10;
int global_win_x2 = 690;
int global_win_y1 = 20;
int global_win_y2 = 420;
*/

int global_win_x1 = 10;
int global_win_x2 = 500;
int global_win_x3 = 980;
int global_win_y1 = 20;
int global_win_y2 = 320;

int global_min_threshold = 5;
int global_squareness_ratio = 50;

/*
int hMin = 5;
int sMin = 50;
int vMin = 50;

int hMax = 15;
int sMax = 255;
int vMax = 255;
*/

int hMin = 5;
int sMin = 50;
int vMin = 50;

int hMax = 15;
int sMax = 255;
int vMax = 255;


void getBoundingRectPCA(cv::Mat& binaryImg){
int count = cv::countNonZero(binaryImg);
	cv::Mat data(2, count, CV_32FC1);
	int dataColumnIndex = 0;
	for (int row = 0; row < binaryImg.rows; row++) {
		for (int col = 0; col < binaryImg.cols; col++) {
			if (binaryImg.at<unsigned char>(row, col) != 0) {
				data.at<float>(0, dataColumnIndex) = (float) col; //x coordinate
				data.at<float>(1, dataColumnIndex) = (float) (binaryImg.rows - row); //y coordinate, such that y axis goes up
				++dataColumnIndex;
			}
		}
	}
	
	const int maxComponents = 1;
	cv::PCA pca(data, cv::Mat() /*mean*/, CV_PCA_DATA_AS_COL, maxComponents);
	//result is contained in pca.eigenvectors (as row vectors)
	//std::cout << pca.eigenvectors << std::endl;

	//3. get angle of principal axis
	float dx = pca.eigenvectors.at<float>(0, 0);
	float dy = pca.eigenvectors.at<float>(0, 1);
	float angle = atan2f(dy, dx)  / (float)CV_PI*180.0f;
	
	printf("angle: %.2f\n", angle );
}

void func_callback(int, void*){
//do nothing
}

void update_global_min_threshold(int, void*){
//do nothing
}

void update_global_squareness_ratio(int, void*){
//do nothing
}

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	//image_transport::Publisher image_pub_;

	public:
	ImageConverter()
	: it_(nh_)
	{
		image_sub_ = it_.subscribe("/cameras/right_hand_camera/image", 1, &ImageConverter::imageCb, this);
		//image_pub_ = it_.advertise("/image_converter/output_video", 1);

		cv::namedWindow(OPENCV_WINDOW, 0);
		cv::resizeWindow(OPENCV_WINDOW, global_win_width, global_win_height);
		cv::moveWindow(OPENCV_WINDOW, global_win_x1, global_win_y1);
	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;

		try{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch(cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge::Exception %s", e.what());
		return;
	}

	//if(cv_ptr->image.rows > 200 && cv_ptr->image.cols > 200)
	//cv::circle(cv_ptr->image, cv::Point(300, 300), 10, CV_RGB(0, 0, 255));
	
	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	cv::imwrite("images/image01_baxtercamera.jpg", cv_ptr->image);

	cv::Mat blur_image;
	cv::blur(cv_ptr->image, blur_image, cv::Size(3, 3));
	
	cv::Mat hsv_image;
	cv::cvtColor(blur_image, hsv_image, CV_BGR2HSV);
	
	cv::namedWindow("HSV Image", 0);
	cv::resizeWindow("HSV Image", global_win_width, global_win_height);
	cv::moveWindow("HSV Image", global_win_x1, global_win_y2);
	cv::imshow("HSV Image", hsv_image);
	cv::imwrite("images/image02_hsv.jpg", hsv_image);

	/*
	cv::Mat filtered_image2;
	cv::cvtColor(cv_ptr->image, filtered_image2, CV_BGR2HLS);
	
	cv::namedWindow("Range2", 0);
	cv::resizeWindow("Range2", global_win_width, global_win_height);
	cv::moveWindow("Range2", global_win_x2, global_win_y1);
	cv::imshow("Range2", filtered_image2);
	
	
	cv::Mat filtered_image3;
	cv::cvtColor(cv_ptr->image, filtered_image3, CV_BGR2GRAY);
	
	cv::namedWindow("Range3", 0);
	cv::resizeWindow("Range3", global_win_width, global_win_height);
	cv::moveWindow("Range3", global_win_x2, global_win_y2);
	cv::imshow("Range3", filtered_image3);
	*/
	
	//cv::imwrite("output.jpg", filtered_image);
	
	//OpenCV uses H: 0 - 180, S: 0 - 255, V: 0 - 255

	
	
	cv::Mat filtered_image;
	cv::inRange(hsv_image, cv::Scalar(hMin, sMin, vMin), cv::Scalar(hMax, sMax, vMax), filtered_image);

	cv::namedWindow("Filtered Image", 0);
	cv::resizeWindow("Filtered Image", global_win_width, global_win_height);
	cv::moveWindow("Filtered Image", global_win_x2, global_win_y1);
	cv::imshow("Filtered Image", filtered_image);
	cv::imwrite("images/image03_filtered.jpg", filtered_image);
	//cv_ptr->image and hsv_image are CV_8U : 0 to 255
	//filtered_image is CV_32F : 0 to 1
	
	cv::createTrackbar("Min H:", "Filtered Image", &hMin, 180, func_callback);
	func_callback( 0, 0);
	
	cv::createTrackbar("Min S:", "Filtered Image", &sMin, 255, func_callback);
	func_callback( 0, 0);
	
	cv::createTrackbar("Min V:", "Filtered Image", &vMin, 255, func_callback);
	func_callback( 0, 0);
	
	cv::createTrackbar("Max H:", "Filtered Image", &hMax, 180, func_callback);
	func_callback( 0, 0);
	
	cv::createTrackbar("Max S:", "Filtered Image", &sMax, 255, func_callback);
	func_callback( 0, 0);
	
	cv::createTrackbar("Max V:", "Filtered Image", &vMax, 255, func_callback);
	func_callback( 0, 0);
	
	
	/*
	cv::Mat filtered_image_bgr;
	cv::cvtColor(filtered_image, filtered_image_bgr, CV_HSV2BGR);
	cv::namedWindow("Filtered BGR", 0);
	cv::resizeWindow("Filtered BGR", global_win_width, global_win_height);
	cv::moveWindow("Filtered BGR", global_win_x2, global_win_y1);
	cv::imshow("Filtered BGR", filtered_image_bgr);
	cv::imwrite("image04_filtered BGR.jpg", filtered_image_bgr);
	*/
	
	/*
	cv::Mat morph_image;
	cv::cvtColor(filtered_image, morph_image, CV_BGR2GRAY);
	cv::Mat element5(5, 5, CV_8U, cv::Scalar(1));
	cv::morphologyEx(filtered_image, filtered_image, cv::MORPH_CLOSE, element5);
	cv::morphologyEx(filtered_image, filtered_image, cv::MORPH_OPEN, element5);

	cv::blur(filtered_image, filtered_image, cv::Size(3, 3));

	cv::namedWindow("Filtered", 0);
	cv::resizeWindow("Filtered", global_win_width, global_win_height);
	cv::moveWindow("Filtered", global_win_x2, global_win_y1);
	cv::imshow("Filtered", filtered_image);
	cv::imwrite("image03_filtered.jpg", filtered_image);
	*/

	
	cv::Mat contoured_image;
	cv::morphologyEx(filtered_image, contoured_image, cv::MORPH_GRADIENT, cv::Mat());

	//MORPH_GRADIENT
	cv::namedWindow("Contoured Image", 0);
	cv::resizeWindow("Contoured Image", global_win_width, global_win_height);
	cv::moveWindow("Contoured Image", global_win_x2, global_win_y2);
	cv::imshow("Contoured Image", contoured_image);
	cv::imwrite("images/image04_countoured.jpg", contoured_image);

	
	cv::Mat threshold_image;
	cv::threshold(contoured_image, threshold_image, global_min_threshold, 255, cv::THRESH_BINARY);

	cv::namedWindow("Threshold Image", 0);
	cv::resizeWindow("Threshold Image", global_win_width, global_win_height);
	cv::moveWindow("Threshold Image", global_win_x3, global_win_y1);
	cv::imshow("Threshold Image", threshold_image);
	cv::imwrite("images/image05_threshold.jpg", contoured_image);
	
	cv::createTrackbar("Min Threshold:", "Threshold Image", &global_min_threshold, 255, update_global_min_threshold);
	update_global_min_threshold(global_min_threshold, 0);

	
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findContours(threshold_image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE); // CV_RETR_TREE

	// moments
	cv::vector<cv::Moments> mu( contours.size() );
	for( int i=0; i<contours.size(); i++) {
		mu[i] = moments( contours[i], false ); }
	
	// mass
	cv::vector<cv::Point2f> mc( contours.size() );
	for( int i=0; i<contours.size(); i++) {
		mc[i] = cv::Point2f( mu[i].m10/mu[i].m00, mu[i].m10/mu[i].m00 ); }
		
	// contours
	cv::Mat contours_image = cv::Mat::zeros( threshold_image.size(), CV_32FC3 );
	
	// prev
	/*
	printf("\tInfo: Area and Contour Length \n");
	for( int i=0; i<contours.size(); i++) {
		//printf(" * Contour[%d]- Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n ", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
		cv::Scalar color = cv::Scalar ( 0, 255, 0 );
		drawContours( contours_image, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
		//circle( contours_image, mc[i], 4, color, -1, 8, 0 );
	}
	cv::namedWindow("Countours Image", 0);
	cv::resizeWindow("Countours Image", global_win_width, global_win_height);
	cv::moveWindow("Countours Image", global_win_x3, global_win_y2);
	cv::imshow("Countours Image", contours_image);
	cv::imwrite("images/image06_contours.jpg", contours_image);
	
	
	*/
	
	int largest_area = 0;
	int largest_contour_index = 0;
	cv::Rect bounding_rect;
	
	for( int i=0; i<contours.size(); i++) {
		//printf(" * Contour[%d]- Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n ", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
		//cv::Scalar color = cv::Scalar ( 0, 255, 0 );
		//drawContours( contours_image, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
		//circle( contours_image, mc[i], 4, color, -1, 8, 0 );
		double a=contourArea( contours[i], false );
		if( a>largest_area ) {
				largest_area = a;
				largest_contour_index = i;
				bounding_rect = boundingRect( contours[i] );
		}
	}
	
	cv::Scalar color(255, 255, 255);
	drawContours( contours_image, contours, largest_contour_index, color, CV_FILLED, 8, hierarchy );
	rectangle( blur_image, bounding_rect, cv::Scalar( 0, 255, 0 ), 1, 8, 0 ); // later change blur_image
	
	int tlx = bounding_rect.tl().x;
	int tly = bounding_rect.tl().y;
	
	int brx = bounding_rect.br().x;
	int bry = bounding_rect.br().y;
	
	int length_x = brx - tlx;
	int length_y = bry - tly;
	
	int center_x = (int)( tlx + brx)/2;
	int center_y = (int)( tly + bry)/2;
	
	//printf(" tlx: %d\t tly: %d\t brx: %d\t bry: %d\t center_x: %d center_y: %d \n ", tlx, tly, brx, bry, center_x, center_y );
	//printf(" x: %d\t %d\t %d\t %d\t, y: %d\t %d\t %d\t %d \n", tlx, center_x, brx, length_x, tly, center_y, bry, length_y );
	
	cv::namedWindow("Blur Image", 0);
	cv::resizeWindow("Blur Image", global_win_width, global_win_height);
	cv::moveWindow("Blur Image", global_win_x3, global_win_y2);
	cv::imshow("Blur Image", blur_image);
	cv::imwrite("images/image07_blur.jpg", blur_image);
	
	cv::namedWindow("Bounding Image", 0);
	cv::resizeWindow("Bounding Image", global_win_width, global_win_height);
	cv::moveWindow("Bounding Image", global_win_x3, 400);
	cv::imshow("Bounding Image", contours_image);
	cv::imwrite("images/image08_bounding.jpg", contours_image);
	
	//cv::Mat& binaryImg;
	
	//getBoundingRectPCA( contours_image );
	
	//printf(" x: %d\t %d\t %d\t %d,\ty: %d\t %d\t %d\t %d,\tangle: %.2f\n", tlx, center_x, brx, length_x, tly, center_y, bry, length_y, angle );
	printf(" x: %d\t %d\t %d\t %d,\ty: %d\t %d\t %d\t %d,\n", tlx, center_x, brx, length_x, tly, center_y, bry, length_y );
	
	double simpleangle;
	double param;
	if( length_y > length_x ) {
		param = length_y / length_x;
	} else {
	param = length_x / length_y;
	}
	
	simpleangle = atan( param ) * 180 / 3.14159265;
	printf("angle of %.2f is: %.2f,\n", param, simpleangle );
	
	
	//printf(" * Contour[%d]- Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n ", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
	
	/*
	printf("\tInfo: Area and Contour Length \n");
	for( int i=0; i<contours.size(); i++) {
		printf(" * Contour[%d]- Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n ", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
		cv::Scalar color = cv::Scalar ( 0, 0, 255 );
		drawContours( contours_image, contours, i, color, 2, 8, hierarchy, 0, cv::Point() );
		circle( contours_image, mc[i], 4, color, -1, 8, 0 );
	}
	*/

	/* // start of comment mode1
	std::vector<cv::RotatedRect> minRect(contours.size());
	cv::RotatedRect new_rectangle;


	double length_side_a, length_side_b;
	cv::Point2f new_rect_points[4];


	for (int i=0; i<contours.size(); i++) {
		// only keep track of rectangles that might be cubelets... needs work

		new_rectangle=cv::minAreaRect(cv::Mat(contours[i]));
		new_rectangle.points(new_rect_points);

		std::cout<<"INFO:\t point.x = "<<new_rect_points[0].x<<std::endl;
		length_side_a=pow(new_rect_points[0].x-new_rect_points[1].x,2)+
		pow(new_rect_points[0].y-new_rect_points[1].y,2);
		length_side_b=pow(new_rect_points[0].x-new_rect_points[3].x,2)+
		pow(new_rect_points[0].y-new_rect_points[3].y,2);

		std::cout<<"INFO:\tsize, l_a/l_b = "<<new_rectangle.size.area()<<" ,"<<length_side_a/length_side_b<<std::endl;
		if (new_rectangle.size.area()>=500 && // minimum size requirement
		length_side_a/length_side_b>(100.0-global_squareness_ratio)/100.0 && // minimum squareness
		length_side_a/length_side_b<(100.0+global_squareness_ratio)/100.0) {
			minRect[i]=new_rectangle;
		}
		
		std::cout<<"______________________________"<<std::endl;
	}
	

	//// Draw contours & rectangles
	
	//cv::Mat contour_output(threshold_image.size(),CV_8U,cv::Scalar(255));
	
	cv::Mat contour_output(threshold_image.size(),CV_32F,cv::Scalar(255));
	cv::Mat bounding_boxes=cv::Mat::zeros(contour_output.size(),CV_32FC3);;
	int i=0;
	for (; i >=0; i=hierarchy[i][0]) {
		cv::Point2f rect_points[4];
		minRect[i].points(rect_points);
		cv::drawContours(bounding_boxes,contours,i,cv::Scalar(255,255,255),1,8,hierarchy,0,cv::Point());
		for (int j=0; j<4; j++) {
			cv::line(bounding_boxes,rect_points[j],rect_points[(j+1)%4],cv::Scalar(255),2,8);
		}
	}
	cv::namedWindow("Bounding boxes", 0);
	cv::resizeWindow("Bounding boxes", global_win_width, global_win_height);
	cv::moveWindow("Bounding boxes", global_win_x3, global_win_y2);
	cv::imshow("Bounding boxes", bounding_boxes);
	cv::imwrite("images/image06_boundingbox.jpg", bounding_boxes);
	
	cv::createTrackbar("Out-of-square acceptance: ","Bounding boxes", &global_squareness_ratio, 100, update_global_squareness_ratio);
	*/ // end of comment mode1
	
	// end of processing
	cv::waitKey(3);

	//image_pub_.publish(cv_ptr->toImageMsg());

	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter"); //68
	ImageConverter ic;
	ros::spin();

	return 0;
}

