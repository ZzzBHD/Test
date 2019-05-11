#include "kinect_yolo.h"

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
//    string label = format("%.5f", conf);
//    if (!classes.empty())
//    {
//        CV_Assert(classId < (int)classes.size());
//        label = classes[classId] + ":" + label;
//    }

    //Display the label at the top of the bounding box
//    int baseLine;
//    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
//    top = max(top, labelSize.height);
//    rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
//    rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255));
//    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 0), 1);
}

void drawmainPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
{
    //Draw a rectangle displaying the bounding box
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 0, 255), 3);

    //Get the label for the class name and its confidence
//    string label = format("%.5f", conf);
    string label = "Target";
//    if (!classes.empty())
//    {
//        CV_Assert(classId < (int)classes.size());
//        label = classes[classId] + ":" + label;
//    }

    //Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
//    rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
//    rectangle(frame, Point(left, top - round(1.5*labelSize.height)), Point(left + round(1.5*labelSize.width), top + baseLine), Scalar(255, 255, 255));
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 2);
}

static vector<int>bounding_data;
static int bounding_flag=0;

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
    int max_boxsize=0;
    int max_idx=0;
    bool get_flag=false;
    Rect max_box;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
        if(classIds[idx]==0)
        {
            if(max_boxsize<=box.width*box.height)
            {
                max_box=box;
                max_idx=idx;
                max_boxsize=box.width*box.height;
                get_flag=true;
            }
            drawPred(classIds[idx], confidences[idx], box.x, box.y,box.x + box.width, box.y + box.height, frame);
        }
    }
    if(get_flag)
    {
        bounding_data.push_back (max_box.width*max_box.height);
        bounding_data.push_back (max_box.x);
        drawmainPred(classIds[max_idx], confidences[max_idx], max_box.x, max_box.y,max_box.x + max_box.width, max_box.y + max_box.height, frame);
        bounding_flag=1;
    }

}


static string names_file = "/home/cyberc3/darknet/data/coco.names";
static String model_def = "/home/cyberc3/darknet/cfg/yolov3-tiny.cfg";
static String weights = "/home/cyberc3/darknet/yolov3-tiny.weights";

static int in_w, in_h;
static double thresh = 0.5;
static double nms_thresh = 0.25;


Net net = readNetFromDarknet(model_def, weights);

void yolo_detect_callback(const std_msgs::Int32::ConstPtr& msg)
{
    Mat frame, blob;
    frame=imread("/home/cyberc3/socket/3.png",1);
//    frame=imread("/home/cyberc3/darknet/data/kite.jpg",1);
    bounding_data.clear ();
    if(msg->data==1)
    {
        if(frame.data)
        {
            blobFromImage(frame, blob, 1 / 255.0, Size(in_w, in_h), Scalar(), true, false);

            vector<Mat> mat_blob;
            imagesFromBlob(blob, mat_blob);

            //Sets the input to the network
            net.setInput(blob);

            // Runs the forward pass to get output of the output layers
            vector<Mat> outs;
            net.forward(outs, getOutputsNames(net));

            postprocess(frame, outs, thresh, nms_thresh);

//            vector<double> layersTimes;
//            double freq = getTickFrequency() / 1000;
//            double t = net.getPerfProfile(layersTimes) / freq;
//            string label = format("Inference time for a frame : %.2f ms", t);
//            putText(frame, label, Point(0, 15), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
//            resize(frame,frame,Size(800,500));

            resize (frame,frame,Size(800,500));
            imshow("res", frame);
            waitKey (30);
            bounding_flag=1;
        }

    }
    else
        bounding_flag=2;
}

int main(int argc,char * argv[])
{
    ros::init (argc,argv,"kinect_yolo_pub");
    ros::NodeHandle nh;
    ros::Publisher kinect_box_pub=nh.advertise <std_msgs::Int32MultiArray>("kinect_bounding_box",20);
    ros::Subscriber kinect_rgb_pub=nh.subscribe <std_msgs::Int32>("kinect_rgb_flag",10,yolo_detect_callback);
    std_msgs::Int32MultiArray bounding_box;

    in_w = in_h = 608;
    //read names
    bounding_flag=0;
    ifstream ifs(names_file.c_str());
    string line;
    while (getline(ifs, line)) classes.push_back(line);

    //init model
//    Net net = readNetFromDarknet(model_def, weights);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);
    printf("YOLO init\n");
    while(ros::ok ())
    {
        ros::spinOnce ();
        if(bounding_flag==1)
        {
            for(int i=0;i<bounding_data.size ();i=i+1)
            {
                cout<<"bounding_data.size:"<<bounding_data.size ()<<endl;
                bounding_box.data.push_back (bounding_data[i]);
//                bounding_box.data.push_back (bounding_data[i+1]);
            }
            kinect_box_pub.publish(bounding_box);
            bounding_box.data.clear ();
            bounding_flag=0;
        }
        else if(bounding_flag==2)
        {
            bounding_box.data.push_back(0);
            bounding_box.data.push_back(0);
            kinect_box_pub.publish(bounding_box);
            break;
        }
        else ;
    }
    printf ("YOLO node close...\n");

    return 0;
}
