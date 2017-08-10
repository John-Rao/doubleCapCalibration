#include <QCoreApplication>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


Mat myResize(Mat in,float offset,int offsetY);
vector<vector<Point2f> >corners_l_array, corners_r_array;
int array_index = 0;
bool ChessboardStable(vector<Point2f>corners_l, vector<Point2f>corners_r){
  if (corners_l_array.size() < 10){
    corners_l_array.push_back(corners_l);
    corners_r_array.push_back(corners_r);
    return false;
  }
  else{
    corners_l_array[array_index % 10] = corners_l;
    corners_r_array[array_index % 10] = corners_r;
    array_index++;
    double error = 0.0;
    for (int i = 0; i < corners_l_array.size(); i++){
      for (int j = 0; j < corners_l_array[i].size(); j++){
        error += abs(corners_l[j].x - corners_l_array[i][j].x) + abs(corners_l[j].y - corners_l_array[i][j].y);
        error += abs(corners_r[j].x - corners_r_array[i][j].x) + abs(corners_r[j].y - corners_r_array[i][j].y);
      }
    }
    if (error < 1000)
    {
      corners_l_array.clear();
      corners_r_array.clear();
      array_index = 0;
      return true;
    }
    else
      return false;
  }
}
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    VideoCapture camera_l(1);
    VideoCapture camera_r(0);

    while (!camera_l.isOpened()) { camera_l.open(1); };
    while (!camera_r.isOpened()) { camera_r.open(0); };
//    camera_l.set(CAP_PROP_FRAME_WIDTH, 640);
//    camera_l.set(CAP_PROP_FRAME_HEIGHT, 480);
//    camera_r.set(CAP_PROP_FRAME_WIDTH, 640);
//    camera_r.set(CAP_PROP_FRAME_HEIGHT, 480);

    Mat frame_l, frame_r;

    Size boardSize(9, 7);
    const float squareSize = 21.5f;  // Set this to your actual square size

    vector<vector<Point2f> > imagePoints_l;
    vector<vector<Point2f> > imagePoints_r;



    int nimages = 0;

    while (1){
      camera_l >> frame_l;
      camera_r >> frame_r;

      frame_l=myResize(frame_l,84,15);
      bool found_l = false, found_r = false;

      vector<Point2f>corners_l, corners_r;
      found_l = findChessboardCorners(frame_l, boardSize, corners_l,
        CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
      found_r = findChessboardCorners(frame_r, boardSize, corners_r,
        CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);


      if (found_l && found_r &&ChessboardStable(corners_l, corners_r)) {
        Mat viewGray;
        cvtColor(frame_l, viewGray, COLOR_BGR2GRAY);
        cornerSubPix(viewGray, corners_l, Size(11, 11),
          Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
        cvtColor(frame_r, viewGray, COLOR_BGR2GRAY);
        cornerSubPix(viewGray, corners_r, Size(11, 11),
          Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));

        imagePoints_l.push_back(corners_l);
        imagePoints_r.push_back(corners_r);
        ++nimages;
        frame_l += 100;
        frame_r += 100;

        drawChessboardCorners(frame_l, boardSize, corners_l, found_l);
        drawChessboardCorners(frame_r, boardSize, corners_r, found_r);

        putText(frame_l, to_string(nimages), Point(20, 20), 1, 1, Scalar(0, 0, 255));
        putText(frame_r, to_string(nimages), Point(20, 20), 1, 1, Scalar(0, 0, 255));
        imshow("Left Camera", frame_l);
        imshow("Right Camera", frame_r);
        char c = (char)waitKey(500);
        if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
          exit(-1);

        if (nimages >= 30)
          break;
      }
      else{
        drawChessboardCorners(frame_l, boardSize, corners_l, found_l);
        drawChessboardCorners(frame_r, boardSize, corners_r, found_r);

        putText(frame_l, to_string(nimages), Point(20, 20), 1, 1, Scalar(0, 0, 255));
        putText(frame_r, to_string(nimages), Point(20, 20), 1, 1, Scalar(0, 0, 255));
        imshow("Left Camera", frame_l);
        imshow("Right Camera", frame_r);

        char key = waitKey(1);
        if (key == 27)
          break;
      }


    }
    if (nimages < 20){ cout << "Not enough" << endl; return -1; }

    vector<vector<Point2f> > imagePoints[2] = { imagePoints_l, imagePoints_r };
    vector<vector<Point3f> > objectPoints;
    objectPoints.resize(nimages);

    for (int i = 0; i < nimages; i++)
    {
      for (int j = 0; j < boardSize.height; j++)
      for (int k = 0; k < boardSize.width; k++)
        objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
    }

    cout << "Running stereo calibration ..." << endl;

    Size imageSize(320, 240);

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints_l, imageSize, 0);
    cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints_r, imageSize, 0);
    Mat R, T, E, F;

    double rms = stereoCalibrate(objectPoints, imagePoints_l, imagePoints_r,
      cameraMatrix[0], distCoeffs[0],
      cameraMatrix[1], distCoeffs[1],
      imageSize, R, T, E, F,
      CALIB_FIX_ASPECT_RATIO +
      CALIB_ZERO_TANGENT_DIST +
      CALIB_USE_INTRINSIC_GUESS +
      CALIB_SAME_FOCAL_LENGTH +
      CALIB_RATIONAL_MODEL +
      CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
      TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
    cout << "done with RMS error=" << rms << endl;


    double err = 0;
    int npoints = 0;
    vector<Vec3f> lines[2];
    for (int i = 0; i < nimages; i++)
    {
      int npt = (int)imagePoints_l[i].size();
      Mat imgpt[2];
      imgpt[0] = Mat(imagePoints_l[i]);
      undistortPoints(imgpt[0], imgpt[0], cameraMatrix[0], distCoeffs[0], Mat(), cameraMatrix[0]);
      computeCorrespondEpilines(imgpt[0], 0 + 1, F, lines[0]);

      imgpt[1] = Mat(imagePoints_r[i]);
      undistortPoints(imgpt[1], imgpt[1], cameraMatrix[1], distCoeffs[1], Mat(), cameraMatrix[1]);
      computeCorrespondEpilines(imgpt[1], 1 + 1, F, lines[1]);

      for (int j = 0; j < npt; j++)
      {
        double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
          imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
          fabs(imagePoints[1][i][j].x*lines[0][j][0] +
          imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
        err += errij;
      }
      npoints += npt;
    }
    cout << "average epipolar err = " << err / npoints << endl;

    FileStorage fs("intrinsics.yml", FileStorage::WRITE);
    if (fs.isOpened())
    {
      fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
        "M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
      fs.release();
    }
    else
      cout << "Error: can not save the intrinsic parameters\n";


    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];

    stereoRectify(cameraMatrix[0], distCoeffs[0],
      cameraMatrix[1], distCoeffs[1],
      imageSize, R, T, R1, R2, P1, P2, Q,
      CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

    fs.open("extrinsics.yml", FileStorage::WRITE);
    if (fs.isOpened())
    {
      fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
      fs.release();
    }
    else
      cout << "Error: can not save the extrinsic parameters\n";

    return a.exec();
}
Mat myResize(Mat in,float offset,int offsetY){
    Mat temp;
    float newWidth,newHeight;
    newWidth=640-2*offset;
    newHeight=newWidth*3/4;

    Mat t=in(Rect((int)(0+offset),(int)((480-newHeight)/2+offsetY),(int)newWidth,(int)newHeight));
    t.copyTo(temp);

    resize(temp,temp,Size(640,480));
    return temp;
}
