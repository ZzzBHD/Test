#include <fstream>
#include <sstream>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <vector>
#include "stdio.h"
#include <ctime>

using namespace std;
using namespace cv;
using namespace dnn;

vector<string> classes;

vector<String> getOutputsNames(Net&net)
{
    static vector<String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}
void drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
{
    //Draw a rectangle displaying the bounding box
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);

    //Get the label for the class name and its confidence
    string label = format("%.5f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    }

    //Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 1);
}
void postprocess(Mat& frame, const vector<Mat>& outs, float confThreshold, float nmsThreshold)
{
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;

    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
//        if(classIds[idx]==0||classIds[idx]==18)
//        {
//            cout<<"classID:"<<classIds[idx]<<endl<<"confidence:"<<confidences[idx]<<endl;
//            cout<<"boundingbox:   x:"<<box.x<<"   y:"<<box.y<<"    width:"<<box.width<<"    height:"<<box.height
//               <<"    size:"<<box.width*box.height<<endl;
//            cout<<"--------------------------------------------------------"<<endl;
//        }
        drawPred(classIds[idx], confidences[idx], box.x, box.y,
            box.x + box.width, box.y + box.height, frame);
    }
}

int main(int argc,char * argv[])
{
    string names_file = "/home/cyber-z/test/coco.names";
    String model_def = "/home/cyber-z/test/yolov3.cfg";
    String weights = "/home/cyber-z/test/yolov3.weights";

    string s_image_name;
    cv::VideoWriter writer;
    int isColor = 1;//不知道是干啥用的
    int frame_fps = 10;
    int frame_width = 1890;
    int frame_height = 1018;
    using namespace cv;
    string video_name = "out1.avi";
    writer = VideoWriter(video_name, CV_FOURCC('M', 'J', 'P', 'G'), frame_fps, Size(frame_width, frame_height), isColor);
    stringstream str;

    clock_t start_time,end_time;
    double duration;
    int in_w, in_h;
    double thresh = 0.5;
    double nms_thresh = 0.25;
    in_w = in_h = 608;

//    string img_path = "/home/cyberc3/darknet/data/person.jpg";

    //read names

    ifstream ifs(names_file.c_str());
    string line;
    while (getline(ifs, line)) classes.push_back(line);

    //init model

    Net net = readNetFromDarknet(model_def, weights);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);

    int frame_num = 526;//输入的图片总张数
    int i = 1;

    while (i <= frame_num)

    //while(waitKey (30)!=27)
    {

    start_time=clock();
    //read image and forward
//    VideoCapture capture(2);// VideoCapture:OENCV中新增的类，捕获视频并显示出来
//    while (waitKey(30)!=27)
//    {
    Mat frame, blob;
//    capture >> frame;
    //frame=imread("/home/cyberc3/darknet/data/kite.jpg",1);
    str << i << ".jpg";
    s_image_name = "/home/cyber-z/test/img_k/" + str.str();
    frame = imread(s_image_name,1);
    if (!frame.data)//判断图片调入是否成功
    {
        cout << "Could not load image file...\n" << endl;
    }
//    frame=imread("/home/cyberc3/darknet/data/dog.jpg",1);
    blobFromImage(frame, blob, 1 / 255.0, Size(in_w, in_h), Scalar(), true, false);

    vector<Mat> mat_blob;
    imagesFromBlob(blob, mat_blob);

    //Sets the input to the network
    net.setInput(blob);

    // Runs the forward pass to get output of the output layers
    vector<Mat> outs;
    net.forward(outs, getOutputsNames(net));

    postprocess(frame, outs, thresh, nms_thresh);
//    end_time=clock();
//    duration=(double)(end_time-start_time)/1000000;
    vector<double> layersTimes;
    double freq = getTickFrequency() / 1000;
    double t = net.getPerfProfile(layersTimes) / freq;
    string label = format("Inference time for a frame : %.2f ms", t);
    putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
//    while(waitKey (30)!=27)
//    cout<<duration<<endl;
    writer.write(frame);
    cout<<"NO."<<i<<"finished"<<endl;
    i++;

    str.str("");
//    imshow("res", frame);

//    waitKey(10);
    }
    cout<<"finished"<<endl;
    return 0;
}
