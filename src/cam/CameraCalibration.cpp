#include "CameraCalibation.hpp"

using namespace cv;

namespace cam {

CameraCalibration::CameraCalibration(CJ2B2Client& j2b2) : j2b2(j2b2) {
}

}

Mat CameraCalibration::getImage() {
	MaCI::Image::CImageData imgData;
	unsigned int cameraSeq = 0;
	
	if (j2b2.iImageCameraFront->GetImageData(imgData, &cameraSeq)) {
		MaCI::Image::CImageContainer srcimg;
		if (imgData.GetImage(srcimg, NULL, true)) {
			std::cout << "GetImage() success" << std::endl;
		} else {
			dPrint(1, "WTF, got image data with no data");
		}
	}

		
	const MaCI::Image::TImageInfo imginfo = srcimg.GetImageInfoRef();
	const unsigned int rows = imginfo.imageheight;
	const unsigned int cols = imginfo.imagewidth;
	Mat img = Mat(rows, cols, CV_8UC1);
	
	if (srcimg.GetImageDataType() != MaCI::Image::EImageDataType::KImageDataJPEG) {
		std::cout << "ImgDataType not JPEG!!!" << std::endl;;
		return img;
	}

	bool success = srcimg.ConvertTo(MaCI::Image::EImageDataType::KImageDataRGB);
	if (success) {
		img = Mat(rows, cols, CV_8UC3, (unsigned char*)srcimg.GetImageDataPtr());
	}
	else {
		std::cout << "Error converting image to Mat" << std::endl;
	}

	return img.clone();
}

void CameraCalibration::runCalibration() {
	// http://www.aishack.in/2010/07/calibrating-undistorting-with-opencv-in-c-oh-yeah/
	// http://opencv.willowgarage.com/documentation/cpp/camera_calibration_and_3d_reconstruction.html
	
    int numBoards = 20;
    int numCornersHor = 6;
    int numCornersVer = 7;

/*	
	printf("Enter number of corners along width: ");
    scanf("%d", &numCornersHor);
 
    printf("Enter number of corners along height: ");
    scanf("%d", &numCornersVer);
 
    printf("Enter number of boards: ");
    scanf("%d", &numBoards);
*/
	
	int numSquares = numCornersHor * numCornersVer;
    Size board_sz = Size(numCornersHor, numCornersVer);
	
	vector<vector<Point3f> > object_points;
    vector<vector<Point2f> > image_points;
	
	vector<Point2f> corners;
    int successes=0;
	
	Mat image = getImage();
    Mat gray_image;
	
	vector<Point3f> obj;
    for(int j=0;j<numSquares;j++)
        obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0.0f));
		
    while(successes<numBoards)
    {
	    cvtColor(image, gray_image, CV_BGR2GRAY);
		
        bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
 
        if(found)
        {
            cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray_image, board_sz, corners, found);
        }
		
        imshow("win1", image);
        imshow("win2", gray_image);
 
        image = getImage();
 
        int key = waitKey(1);

        if(key==27)
            return 0;
 
        if(key==' ' && found!=0)
        {
            image_points.push_back(corners);
            object_points.push_back(obj);
            printf("Snap stored!\n");
 
            successes++;
 
            if(successes>=numBoards)
                break;
        }
    }

	Mat intrinsic = Mat(3, 3, CV_32FC1);
    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;
	
    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;
	
	// double calibrateCamera(const vector<vector<Point3f> >& objectPoints, const vector<vector<Point2f> >& imagePoints, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvecs, vector<Mat>& tvecs, int flags=0)¶
	calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);	

	
    Mat imageUndistorted;
    while(1)
    {
        image = getImage();
        undistort(image, imageUndistorted, intrinsic, distCoeffs);
 
		std::cout << intrinsic << distCoeffs << std::endl;
 
        imshow("win1", image);
        imshow("win2", imageUndistorted);
 
        waitKey(1);
    }
} 
