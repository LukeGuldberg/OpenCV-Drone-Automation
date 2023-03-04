# Using OpenCV
## <u>Week One</u>
### Installing OpenCV Library
- The task of installing OpenCV on Windows turned out to be a more challenging task than expected. The method that ended up working was downloading the library using MSYS2. MSYS2 makes downloading different softwares and libraries on Windows much easier. It does this by doing work behind the scenes like adding things to the file path.
### Using Webcam
- The very first step of using OpevCV was getting a window that displayed the webcam from my laptop. OpenCV makes this very easy, and it only requires that you set the capture to 0.   
`cv::VideoCapture capture{0};`
- The next step was to then convert the image captured by my webcam into grey scale. This was done easily with a helper function that converts the color of an image.
```c++
cv::Mat frame;
cv::Mat greyMat;
cv::cvtColor(frame, greyMat, cv::COLOR_BGR2GRAY);
cv::imshow("Tello", frame);
cv::imshow("Tello_Grey", greyMat);
```
## <u>Week Two</u>
### Picking Out Key Points
- The idea behind this project revolves around being able to track key points....
