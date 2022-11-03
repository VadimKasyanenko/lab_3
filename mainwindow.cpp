#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}
// https://russianblogs.com/article/9236915207/
// https://www.geeksforgeeks.org/python-thresholding-techniques-using-opencv-set-1-simple-thresholding/
// https://docs.opencv.org/4.x/dc/dd3/tutorial_gausian_median_blur_bilateral_filter.html

// (поиск линий) https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html
void MainWindow::on_pushButton_clicked()
{
     Mat dst, cdst;
     Mat src = imread(path, IMREAD_GRAYSCALE );
     // Edge detection
     Canny(src, dst, 50, 200, 3);
     // Copy edges to the images that will display the results in BGR
     cvtColor(dst, cdst, COLOR_GRAY2BGR);
     // Standard Hough Line Transform
     vector<Vec2f> lines; // will hold the results of the detection
     HoughLines(dst, lines, 1, CV_PI/180, 150, 0, 0 ); // runs the actual detection
     // Draw the lines
     for( size_t i = 0; i < lines.size(); i++ )
     {
         float rho = lines[i][0], theta = lines[i][1];
         Point pt1, pt2;
         double a = cos(theta), b = sin(theta);
         double x0 = a*rho, y0 = b*rho;
         pt1.x = cvRound(x0 + 1000*(-b));
         pt1.y = cvRound(y0 + 1000*(a));
         pt2.x = cvRound(x0 - 1000*(-b));
         pt2.y = cvRound(y0 - 1000*(a));
         line( cdst, pt1, pt2, Scalar(0,0,255), 3, LINE_AA);
     }
     imshow("Detected Lines (in red) - Standard Hough Line Transform", cdst);
}
// (поиск точек) https://docs.opencv.org/3.4/d2/d2c/tutorial_sobel_derivatives.html
void MainWindow::on_pushButton_2_clicked()
{

    Point anchor( -1, -1 );
      double delta = 0;
      int ddepth = -1;
      Mat dst;

      float data[2][5] = {{11,11,11,11,11},{11,11,11,11,11}};
      float kernel[2][2] = {{2,2},{2,2}};

      Mat src = Mat(2, 5, CV_32FC1, &data);
      Mat ker = Mat(2, 2, CV_32FC1, &kernel);
      Mat img = imread(path, IMREAD_GRAYSCALE);
      Mat res;
      filter2D(img, dst, ddepth , ker,anchor);
      filter2D(img, res, ddepth , ker,anchor,0,BORDER_DEFAULT);
      cout << dst << endl;
       imshow("Points", res);


  /*   Mat image,src, src_gray;
     Mat grad;
     const String window_name = "Sobel Demo - Simple Edge Detector";
     int ksize = 1; // размер ядра(нечет) ширина и высота фильтра до 31
     int scale = 1; // масштабный коэффициент
     int delta = 0; // смещение
     int ddepth = CV_16S; // глубина входного изображения
     // As usual we load our source image (src)
     image = imread(path, IMREAD_COLOR ); // Load an image
     for (;;)
     {
       // Remove noise by blurring with a Gaussian filter ( kernel size = 3 )
       GaussianBlur(image, src, Size(3, 3), 0, 0, BORDER_DEFAULT);
       // Convert the image to grayscale
       cvtColor(src, src_gray, COLOR_BGR2GRAY);
       Mat grad_x, grad_y;
       Mat abs_grad_x, abs_grad_y;
       Sobel(src_gray, grad_x, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT);
       Sobel(src_gray, grad_y, ddepth, 0, 1, ksize, scale, delta, BORDER_DEFAULT);
       // converting back to CV_8U
       convertScaleAbs(grad_x, abs_grad_x);
       convertScaleAbs(grad_y, abs_grad_y);
       addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
       imshow(window_name, grad);
       char key = (char)waitKey(0);
}*/
}

void MainWindow::on_pushButton_3_clicked()
{
    QString str;
    double lower_bound,upper_bound;
    str = ui->lineEdit_11->text();
    lower_bound =str.toDouble();
    str = ui->lineEdit_12->text();
    upper_bound =str.toDouble();
    Mat g_srcImage, g_srcGrayImage, g_dstImage;
    Mat g_cannyDetectedEdges;
    int g_cannyLowThreshold = 1;// TrackBar позиционный параметр
    g_srcImage = imread(path);
    g_dstImage.create(g_srcImage.size(), g_srcImage.type());
    cvtColor(g_srcImage, g_srcGrayImage, COLOR_BGRA2GRAY);

    // Сначала используем ядро ​​3x3 для уменьшения шума
    blur(g_srcGrayImage, g_cannyDetectedEdges, Size(3, 3));

    // Запускаем наш оператор Canny
    Canny(g_cannyDetectedEdges, g_cannyDetectedEdges, lower_bound, upper_bound);

    // Сначала устанавливаем все элементы в g_dstImage на 0
    g_dstImage = Scalar::all(0);

    // Используем изображение края g_cannyDetectedEdges, выведенное оператором Canny, в качестве маски для копирования исходного изображения g_srcImage в целевое изображение g_dstImage
    g_srcImage.copyTo(g_dstImage, g_cannyDetectedEdges);
    imshow("Canny edge", g_dstImage);
}
void MainWindow::on_pushButton_4_clicked()
{
    QString str;
    int dx,dy,kernel_size;
    double scale,delta;
    str = ui->lineEdit_6->text();
    dx =str.toInt();
    str = ui->lineEdit_7->text();
    dy =str.toInt();
    str = ui->lineEdit_10->text();
    kernel_size =str.toInt();
    str = ui->lineEdit_9->text();
    scale =str.toDouble();
    str = ui->lineEdit_8->text();
    delta =str.toDouble();
    Mat g_srcImage, g_srcGrayImage, g_dstImage;
    Mat g_sobelGradient_X, g_sobelGradient_Y;
    Mat g_sobelAbsGradient_X, g_sobelAbsGradient_Y;
    g_srcImage = imread(path);
    g_dstImage.create(g_srcImage.size(), g_srcImage.type());
    cvtColor(g_srcImage, g_srcGrayImage, COLOR_BGRA2GRAY);
    Sobel(g_srcImage, g_sobelGradient_X, CV_16S, dx, 0,kernel_size, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(g_sobelGradient_X, g_sobelAbsGradient_X);
    Sobel(g_srcImage, g_sobelGradient_Y, CV_16S, 0, dy,kernel_size, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(g_sobelGradient_Y, g_sobelAbsGradient_Y);
    addWeighted(g_sobelAbsGradient_X, 0.5, g_sobelAbsGradient_Y, 0.5, 0, g_dstImage);
    imshow("Sobel edge", g_dstImage);



    //меньше действий но результат хуже
    /*
    Mat g_srcImage, g_srcGrayImage, end;
    g_srcImage = imread(path);
    cvtColor(g_srcImage, g_srcGrayImage, COLOR_BGRA2GRAY);
    Sobel(g_srcImage, end, CV_16S, dx, dy,kernel_size, scale, delta, BORDER_DEFAUL);
    convertScaleAbs(end, end);
    imshow("Sobel edge", end);
     */

}

void MainWindow::on_pushButton_5_clicked()
{
    QString str;
    int dx,dy;
    double scale,delta;
    str = ui->lineEdit_6->text();
    dx =str.toInt();
    str = ui->lineEdit_7->text();
    dy =str.toInt();
    str = ui->lineEdit_9->text();
    scale =str.toDouble();
    str = ui->lineEdit_8->text();
    delta =str.toDouble();
    Mat g_srcImage, g_srcGrayImage, g_dstImage;
    Mat g_scharrGradient_X, g_scharrGradient_Y;
    Mat g_scharrAbsGradient_X, g_scharrAbsGradient_Y;
    g_srcImage = imread(path);
    g_dstImage.create(g_srcImage.size(), g_srcImage.type());
    cvtColor(g_srcImage, g_srcGrayImage, COLOR_BGRA2GRAY);
    Scharr(g_srcImage, g_scharrGradient_X, CV_16S, dx, 0, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(g_scharrGradient_X, g_scharrAbsGradient_X);
    Scharr(g_srcImage, g_scharrGradient_Y, CV_16S, 0, dy, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(g_scharrGradient_Y, g_scharrAbsGradient_Y);
    addWeighted(g_scharrAbsGradient_X, 0.5, g_scharrAbsGradient_Y, 0.5, 0, g_dstImage);
    imshow("Shcarr edge", g_dstImage);


    //меньше действий но результат хуже
    /*
    Mat g_srcImage, g_srcGrayImage, end;
    g_srcImage = imread(path);
    cvtColor(g_srcImage, g_srcGrayImage, COLOR_BGRA2GRAY);
    Scharr(g_srcImage, end, CV_16S, dx, dy,scale,delta, BORDER_DEFAULT);
    convertScaleAbs(end, end);
    imshow("Sobel edge", end);
     */
}

void MainWindow::on_pushButton_6_clicked()
{
    QString str;
    double x,z;
    int y;
    str = ui->lineEdit_3->text();
    x = str.toDouble();
    str = ui->lineEdit_4->text();
    y = str.toInt();
    str = ui->lineEdit_5->text();
    z = str.toDouble();
    Mat img = imread(path,0);
    Mat src;
    medianBlur(img,src,5);
    Mat th;
    adaptiveThreshold(src,th,x,ADAPTIVE_THRESH_MEAN_C,\
                THRESH_BINARY,y,z);
    imshow("Adaptive Mean threshold",th);
}

void MainWindow::on_pushButton_7_clicked()
{
    QString str;
    double x,y;
    str = ui->lineEdit->text();
    x = str.toDouble();
    str = ui->lineEdit_2->text();
    y = str.toDouble();
    Mat src = imread(path, IMREAD_GRAYSCALE);
    Mat dst;
    threshold(src,dst,x,y, THRESH_BINARY);
    imshow("Binary threshold", dst);
}

void MainWindow::on_pushButton_11_clicked()
{
    QString str;
    double x,y;
    str = ui->lineEdit->text();
    x = str.toDouble();
    str = ui->lineEdit_2->text();
    y = str.toDouble();
    Mat src = imread(path, IMREAD_GRAYSCALE);
    Mat dst;
    threshold(src,dst,x,y, THRESH_BINARY_INV);
    imshow("Inverted binary threshold", dst);
}

void MainWindow::on_pushButton_10_clicked()
{
    QString str;
    double x,y;
    str = ui->lineEdit->text();
    x = str.toDouble();
    str = ui->lineEdit_2->text();
    y = str.toDouble();
    Mat src = imread(path, IMREAD_GRAYSCALE);
    Mat dst;
    threshold(src,dst,x,y, THRESH_TRUNC);
    imshow("Truncated threshold", dst);
}

void MainWindow::on_pushButton_8_clicked()
{
    QString str;
    double x,y;
    str = ui->lineEdit->text();
    x = str.toDouble();
    str = ui->lineEdit_2->text();
    y = str.toDouble();
    Mat src = imread(path, IMREAD_GRAYSCALE);
    Mat dst;
    threshold(src,dst,x,y, THRESH_TOZERO);
    imshow("To Zero threshold", dst);
}

void MainWindow::on_pushButton_9_clicked()
{
    QString str;
    double x,y;
    str = ui->lineEdit->text();
    x = str.toDouble();
    str = ui->lineEdit_2->text();
    y = str.toDouble();
    Mat src = imread(path, IMREAD_GRAYSCALE);
    Mat dst;
    threshold(src,dst,x,y, THRESH_TOZERO_INV);
    imshow("To Zero inverted threshold", dst);
}

void MainWindow::on_pushButton_13_clicked()
{
    QString str;
    double x,y;
    str = ui->lineEdit->text();
    x = str.toDouble();
    str = ui->lineEdit_2->text();
    y = str.toDouble();
    Mat src = imread(path, IMREAD_GRAYSCALE);
    Mat dst;
    threshold(src,dst,x,y,THRESH_BINARY+THRESH_OTSU);
    imshow("Threshold with Otsu", dst);
}

void MainWindow::on_pushButton_14_clicked()
{
    QString str;
    double x,y;
    str = ui->lineEdit->text();
    x = str.toDouble();
    str = ui->lineEdit_2->text();
    y = str.toDouble();
    Mat src = imread(path, IMREAD_GRAYSCALE);
    Mat dst;
    threshold(src,dst,x,y,THRESH_BINARY+THRESH_TRIANGLE);
    imshow("Threshold with Triangle", dst);
}

void MainWindow::on_pushButton_12_clicked()
{

    QString str;
    double x,z;
    int y;
    str = ui->lineEdit_3->text();
    x = str.toDouble();
    str = ui->lineEdit_4->text();
    y = str.toInt();
    str = ui->lineEdit_5->text();
    z = str.toDouble();
    Mat img = imread(path,0);
    Mat src;
    medianBlur(img,src,5);
    Mat th;
    adaptiveThreshold(src,th,x,ADAPTIVE_THRESH_GAUSSIAN_C,\
                THRESH_BINARY,y,z);
    imshow("Adaptive Gaussian threshold",th);
}

void MainWindow::on_pushButton_15_clicked()
{
    QString arr[]={".jpg",".gif",".bmp",".png",".jpeg"};
    QString arch;
         QString fileName = QFileDialog ::getOpenFileName(
            this,
            tr("open a file."),
            "D:/",
            tr("images(*jpg *gif *bmp *png *jpeg);;archive(*zip)"));
       path = fileName.toStdString();
       cout << path;
}

void MainWindow::on_pushButton_16_clicked()
{
    Mat img = imread(path);
    imshow("Original picture",img);
}
