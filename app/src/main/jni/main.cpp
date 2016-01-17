#include <jni.h>
#include <iostream>
#include <stdlib.h>
#include <vector>

#include "com_app_vision_MainActivity.h"

#include "opencv2/text.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;
using namespace cv::text;

#ifdef __cplusplus
extern "C" {
#endif

bool isRepetitive(const string& s)
{
    /*int count = 0;
    for (int i=0; i<(int)s.size(); i++)
    {
        if ((s[i] == 'i') ||
                (s[i] == 'l') ||
                (s[i] == 'I'))
            count++;
    }
    if (count > ((int)s.size()+1)/2)
    {
        return true;
    }*/
    return false;
}


void er_draw(vector<Mat> &channels, vector<vector<ERStat> > &regions, vector<Vec2i> group, Mat& segmentation)
{
    for (int r=0; r<(int)group.size(); r++)
    {
        ERStat er = regions[group[r][0]][group[r][1]];
        if (er.parent != NULL)
        {
            int newMaskVal = 255;
            int flags = 4 + (newMaskVal << 8) + FLOODFILL_FIXED_RANGE + FLOODFILL_MASK_ONLY;
            floodFill(channels[group[r][0]],segmentation,Point(er.pixel%channels[group[r][0]].cols,er.pixel/channels[group[r][0]].cols),
                      Scalar(255),0,Scalar(er.level),Scalar(0),flags);
        }
    }
}

bool sort_by_lenght(const string &a, const string &b){return (a.size()>b.size());}

vector<Rect> getText(Mat image) {
        vector<Mat> channels;

        Mat grey;
        cvtColor(image,grey,COLOR_RGB2GRAY);

        channels.push_back(grey);
        channels.push_back(255-grey);

        double t_d = (double)getTickCount();

        Ptr<ERFilter> er_filter1 = createERFilterNM1(loadClassifierNM1("/storage/sdcard0/SeeSharp/xmls/trained_classifierNM1.xml"),8,0.00015f,0.13f,0.2f,true,0.1f);
        Ptr<ERFilter> er_filter2 = createERFilterNM2(loadClassifierNM2("/storage/sdcard0/SeeSharp/xmls/trained_classifierNM2.xml"),0.5);

        vector<vector<ERStat> > regions(channels.size());
        for (int c=0; c<(int)channels.size(); c++)
        {
            er_filter1->run(channels[c], regions[c]);
            er_filter2->run(channels[c], regions[c]);
        }

        Mat out_img_decomposition= Mat::zeros(image.rows+2, image.cols+2, CV_8UC1);
        vector<Vec2i> tmp_group;
        for (int i=0; i<(int)regions.size(); i++)
        {
            for (int j=0; j<(int)regions[i].size();j++)
            {
                tmp_group.push_back(Vec2i(i,j));
            }
            Mat tmp= Mat::zeros(image.rows+2, image.cols+2, CV_8UC1);
            er_draw(channels, regions, tmp_group, tmp);
            if (i > 0)
                tmp = tmp / 2;
            out_img_decomposition = out_img_decomposition | tmp;
            tmp_group.clear();
        }

        double t_g = (double)getTickCount();

        vector< vector<Vec2i> > nm_region_groups;
        vector<Rect> nm_boxes;
        erGrouping(image, channels, regions, nm_region_groups, nm_boxes,ERGROUPING_ORIENTATION_HORIZ);

        return nm_boxes;

        /*double t_r = (double)getTickCount();
        Ptr<OCRTesseract> ocr = OCRTesseract::create();
        string output;

        Mat out_img;
        Mat out_img_detection;
        Mat out_img_segmentation = Mat::zeros(image.rows+2, image.cols+2, CV_8UC1);
        image.copyTo(out_img);
        image.copyTo(out_img_detection);
        float scale_img  = 600.f/image.rows;
        float scale_font = (float)(2-scale_img)/1.4f;
        vector<string> words_detection;

        t_r = (double)getTickCount();

        for (int i=0; i<(int)nm_boxes.size(); i++)
        {

            rectangle(out_img_detection, nm_boxes[i].tl(), nm_boxes[i].br(), Scalar(0,255,255), 3);

            Mat group_img = Mat::zeros(image.rows+2, image.cols+2, CV_8UC1);
            er_draw(channels, regions, nm_region_groups[i], group_img);
            Mat group_segmentation;
            group_img.copyTo(group_segmentation);

            group_img(nm_boxes[i]).copyTo(group_img);
            copyMakeBorder(group_img,group_img,15,15,15,15,BORDER_CONSTANT,Scalar(0));

            vector<Rect>   boxes;
            vector<string> words;
            vector<float>  confidences;
            ocr->run(group_img, output, &boxes, &words, &confidences, OCR_LEVEL_WORD);

            output.erase(remove(output.begin(), output.end(), '\n'), output.end());
            if (output.size() < 3)
                continue;

            for (int j=0; j<(int)boxes.size(); j++)
            {
                boxes[j].x += nm_boxes[i].x-15;
                boxes[j].y += nm_boxes[i].y-15;

                if ((words[j].size() < 2) || (confidences[j] < 51) ||
                        ((words[j].size()==2) && (words[j][0] == words[j][1])) ||
                        ((words[j].size()< 4) && (confidences[j] < 60)) ||
                        isRepetitive(words[j]))
                    continue;
                words_detection.push_back(words[j]);
                rectangle(out_img, boxes[j].tl(), boxes[j].br(), Scalar(255,0,255),3);
                Size word_size = getTextSize(words[j], FONT_HERSHEY_SIMPLEX, (double)scale_font, (int)(3*scale_font), NULL);
                rectangle(out_img, boxes[j].tl()-Point(3,word_size.height+3), boxes[j].tl()+Point(word_size.width,0), Scalar(255,0,255),-1);
                cout<<words[j]<<endl;
                putText(out_img, words[j], boxes[j].tl()-Point(1,1), FONT_HERSHEY_SIMPLEX, scale_font, Scalar(255,255,255),(int)(3*scale_font));
                out_img_segmentation = out_img_segmentation | group_segmentation;
            }

        }*/
/*
        if(argc>2)
        {
            int num_gt_characters   = 0;
            vector<string> words_gt;
            for (int i=2; i<argc; i++)
            {
                string s = string(argv[i]);
                if (s.size() > 0)
                {
                    words_gt.push_back(string(argv[i]));
                    num_gt_characters += (int)(words_gt[words_gt.size()-1].size());
                }
            }

            if (words_detection.empty())
            {
            }
            else
            {

                sort(words_gt.begin(),words_gt.end(),sort_by_lenght);

                int max_dist=0;
                vector< vector<int> > assignment_mat;
                for (int i=0; i<(int)words_gt.size(); i++)
                {
                    vector<int> assignment_row(words_detection.size(),0);
                    assignment_mat.push_back(assignment_row);
                    for (int j=0; j<(int)words_detection.size(); j++)
                    {
                        assignment_mat[i][j] = (int)(edit_distance(words_gt[i],words_detection[j]));
                        max_dist = max(max_dist,assignment_mat[i][j]);
                    }
                }

                vector<int> words_detection_matched;

                int total_edit_distance = 0;
                int tp=0, fp=0, fn=0;
                for (int search_dist=0; search_dist<=max_dist; search_dist++)
                {
                    for (int i=0; i<(int)assignment_mat.size(); i++)
                    {
                        int min_dist_idx =  (int)distance(assignment_mat[i].begin(),
                                            min_element(assignment_mat[i].begin(),assignment_mat[i].end()));
                        if (assignment_mat[i][min_dist_idx] == search_dist)
                        {
                            if(search_dist == 0)
                                tp++;
                            else { fp++; fn++; }

                            total_edit_distance += assignment_mat[i][min_dist_idx];
                            words_detection_matched.push_back(min_dist_idx);
                            words_gt.erase(words_gt.begin()+i);
                            assignment_mat.erase(assignment_mat.begin()+i);
                            for (int j=0; j<(int)assignment_mat.size(); j++)
                            {
                                assignment_mat[j][min_dist_idx]=INT_MAX;
                            }
                            i--;
                        }
                    }
                }

                for (int j=0; j<(int)words_gt.size(); j++)
                {
                    fn++;
                    total_edit_distance += (int)words_gt[j].size();
                }
                for (int j=0; j<(int)words_detection.size(); j++)
                {
                    if (find(words_detection_matched.begin(),words_detection_matched.end(),j) == words_detection_matched.end())
                    {
                        fp++;
                        total_edit_distance += (int)words_detection[j].size();
                    }
                }
            }
        }*/
}

static jintArray make_row(JNIEnv *env, jsize count, const int* elements[])
{
    jintArray row = env->NewIntArray(4);
    jint *ptrRow = env->GetIntArrayElements(row, NULL);

    for (int i = 0; i < count; ++i) {
        ptrRow[i] = (jint)elements[i];
    }
    env->ReleaseIntArrayElements(row, ptrRow, NULL);
    return row;
}

JNIEXPORT jintArray JNICALL Java_com_app_vision_MainActivity_text(JNIEnv *env, jobject instance,
    jlong addrGray) {

    Mat& gray = *(Mat*)addrGray;
    vector<Rect> nm_boxes = getText(gray);

    int size = nm_boxes.size();

   /* jintArray ele = env->NewIntArray(4);

    jclass elementClass = env->GetObjectClass(ele);
    jobjectArray result = env->NewObjectArray(*env, size, elementClass, 0);

    for(int i=0; i<size; ++i) {
        Rect rect = nm_boxes[i];
        const int ele[4] = { rect.x, rect.y, rect.width, rect.height };
        env->SetObjectArrayElement(env, result, i, (jobject)make_row(env, 4, ele));
    }*/

    int count = 4*size;
    jintArray result = env->NewIntArray(count);
    jint *ptr = env->GetIntArrayElements(result, NULL);

    for (int i = 0; i < count; ++i) {
        int mod = i%4;
        if(mod == 0)
            ptr[i] = (jint)nm_boxes[i/4].x;
        else if(mod == 1)
            ptr[i] = (jint)nm_boxes[i/4].y;
        else if(mod == 2)
            ptr[i] = (jint)nm_boxes[i/4].width;
        else
            ptr[i] = (jint)nm_boxes[i/4].height;
    }

    env->ReleaseIntArrayElements(result, ptr, NULL);

    return result;
}

#ifdef __cplusplus
}
#endif