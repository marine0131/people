/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
// #include "laser_processor.h"
// #include "calc_leg_features.h"
#include <leg_detector/laser_processor.h>
#include <leg_detector/calc_leg_features.h>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/ml.h>

#include <people_msgs/PositionMeasurement.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;
using namespace laser_processor;
using namespace ros;

enum LoadType {LOADING_NONE, LOADING_POS, LOADING_NEG, LOADING_TEST};

class TrainLegDetector
{
public:
  ScanMask mask_;
  int mask_count_;

  vector< vector<float> > pos_data_;
  vector< vector<float> > neg_data_;
  vector< vector<float> > test_data_;

  CvRTrees forest;

  float connected_thresh_;
  int remove_less_than_;

  int feat_count_;

  TrainLegDetector(float connect_th, int remove_less_than) : mask_count_(0), connected_thresh_(0.06), feat_count_(0)
  {
      connected_thresh_ = connect_th;
      remove_less_than_ = remove_less_than;
  }

  void loadData(LoadType load, char* file)
  {
    if (load != LOADING_NONE)
    {
      switch (load)
      {
      case LOADING_POS:
        printf("Loading positive training data from file: %s\n", file);
        break;
      case LOADING_NEG:
        printf("Loading negative training data from file: %s\n", file);
        break;
      case LOADING_TEST:
        printf("Loading test data from file: %s\n", file);
        break;
      default:
        break;
      }

      // read scan topic from bag file, cluster data saved in pose_data_ neg_data_, test_data_
      // ros::record::Player p;
      //rosbag::Bag p;
      //p.open(file, rosbag::bagmode::Read); 
      
      mask_.clear();
      mask_count_ = 0;

      switch (load)
      {
        case LOADING_POS:
        // p.addHandler<sensor_msgs::LaserScan>(string("*"), &TrainLegDetector::loadCb, this, &pos_data_);
        loadCb(file, "scan", pos_data_);
        break;
        case LOADING_NEG:
        mask_count_ = 1000; // effectively disable masking
        // p.addHandler<sensor_msgs::LaserScan>(string("*"), &TrainLegDetector::loadCb, this, &neg_data_);
        loadCb(file, "scan", neg_data_);
        break;
        case LOADING_TEST:
        // p.addHandler<sensor_msgs::LaserScan>(string("*"), &TrainLegDetector::loadCb, this, &test_data_);
        loadCb(file, "scan", test_data_);
        break;
        default:
        break;
      }

      // while (p.nextMsg())
      //{}
      
    }
  }
  void loadCb(
    const char* rosbag_file, 
    const char* scan_topic, 
    std::vector< std::vector<float> > &data
    )
  {
    rosbag::Bag bag;
    bag.open(rosbag_file, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string(scan_topic));
    rosbag::View view(bag); 

    int message_num = 0;
    int initial_neg_data_size = (int)data.size();
    BOOST_FOREACH(rosbag::MessageInstance m, view){
      sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
      if (scan != NULL)
      {
        ScanProcessor processor(*scan, mask_);
        processor.splitConnected(connected_thresh_);
        processor.removeLessThan(remove_less_than_);
 
        for (list<SampleSet*>::iterator i = processor.getClusters().begin();
             i != processor.getClusters().end();
             i++)
            data.push_back(calcLegFeatures(*i, *scan));                 
        message_num++;
      } 
    }
    bag.close();

    printf("\t Got %i scan messages, %i samples, from %s  \n",message_num, (int)data.size() - initial_neg_data_size, rosbag_file);
  } 

// process every scan message and cluster
  // void loadCb(string name, sensor_msgs::LaserScan* scan, ros::Time t, ros::Time t_no_use, void* n)
  // {
  //   vector< vector<float> >* data = (vector< vector<float> >*)(n);

  //   if (mask_count_++ < 20)
  //   {
  //     mask_.addScan(*scan);
  //   }
  //   else
  //   {
  //     ScanProcessor processor(*scan, mask_);
  //     processor.splitConnected(connected_thresh_);
  //     processor.removeLessThan(5);

  //     for (list<SampleSet*>::iterator i = processor.getClusters().begin();
  //          i != processor.getClusters().end();
  //          i++)
  //       data->push_back(calcLegFeatures(*i, *scan));
  //   }
  // }

  void train()
  {
    int sample_size = pos_data_.size() + neg_data_.size(); //sample count
    feat_count_ = pos_data_[0].size();  //feature count

    CvMat* cv_data = cvCreateMat(sample_size, feat_count_, CV_32FC1);  //cv data array
    CvMat* cv_resp = cvCreateMat(sample_size, 1, CV_32S);  //cv result array

    // Put positive data in opencv format.
    int j = 0;
    for (vector< vector<float> >::iterator i = pos_data_.begin();
         i != pos_data_.end();
         i++)
    {
      float* data_row = (float*)(cv_data->data.ptr + cv_data->step * j);
      for (int k = 0; k < feat_count_; k++)
        data_row[k] = (*i)[k];

      cv_resp->data.i[j] = 1;
      j++;
    }

    // Put negative data in opencv format.
    for (vector< vector<float> >::iterator i = neg_data_.begin();
         i != neg_data_.end();
         i++)
    {
      float* data_row = (float*)(cv_data->data.ptr + cv_data->step * j);
      for (int k = 0; k < feat_count_; k++)
        data_row[k] = (*i)[k];

      cv_resp->data.i[j] = -1;
      j++;
    }

    CvMat* var_type = cvCreateMat(1, feat_count_ + 1, CV_8U);
    cvSet(var_type, cvScalarAll(CV_VAR_ORDERED));
    cvSetReal1D(var_type, feat_count_, CV_VAR_CATEGORICAL);

    float priors[] = {1.0, 1.0};

    CvRTParams fparam(8, 20, 0, false, 10, priors, false, 5, 50, 0.001f, CV_TERMCRIT_ITER);
    fparam.term_crit = cvTermCriteria(CV_TERMCRIT_ITER, 100, 0.1);

    forest.train(cv_data, CV_ROW_SAMPLE, cv_resp, 0, 0, var_type, 0,
                 fparam);


    cvReleaseMat(&cv_data);
    cvReleaseMat(&cv_resp);
    cvReleaseMat(&var_type);
  }

  void test()
  {
    CvMat* tmp_mat = cvCreateMat(1, feat_count_, CV_32FC1);

    int pos_right = 0;
    int pos_total = 0;
    for (vector< vector<float> >::iterator i = pos_data_.begin();
         i != pos_data_.end();
         i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest.predict(tmp_mat) > 0)
        pos_right++;
      pos_total++;
    }

    int neg_right = 0;
    int neg_total = 0;
    for (vector< vector<float> >::iterator i = neg_data_.begin();
         i != neg_data_.end();
         i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest.predict(tmp_mat) < 0)
        neg_right++;
      neg_total++;
    }

    int test_right = 0;
    int test_total = 0;
    for (vector< vector<float> >::iterator i = test_data_.begin();
         i != test_data_.end();
         i++)
    {
      for (int k = 0; k < feat_count_; k++)
        tmp_mat->data.fl[k] = (float)((*i)[k]);
      if (forest.predict(tmp_mat) > 0)
        test_right++;
      test_total++;
    }

    printf(" Pos train set: %d/%d %g\n", pos_right, pos_total, (float)(pos_right) / pos_total);
    printf(" Neg train set: %d/%d %g\n", neg_right, neg_total, (float)(neg_right) / neg_total);
    printf(" Test set:      %d/%d %g\n", test_right, test_total, (float)(test_right) / test_total);

    cvReleaseMat(&tmp_mat);

  }

  void save(char* file)
  {
    forest.save(file);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "train_body_detector");
  ros::NodeHandle nh;

  float connect_th = 0.06; 
  int remove_less_than = 5; 
  ros::param::get("~connect_th", connect_th);
  ros::param::get("~remove_less_than", remove_less_than);
  printf("connect_th: %f", connect_th);
  printf("remove less than: %d", remove_less_than);
  TrainLegDetector tld(connect_th, remove_less_than);

  LoadType loading = LOADING_NONE;

  char save_file[100];
  save_file[0] = 0;

  printf("Loading data...\n");
  for (int i = 1; i < argc; i++)
  {
    if (!strcmp(argv[i], "--train"))
      loading = LOADING_POS;
    else if (!strcmp(argv[i], "--neg"))
      loading = LOADING_NEG;
    else if (!strcmp(argv[i], "--test"))
      loading = LOADING_TEST;
    else if (!strcmp(argv[i], "--save"))
    {
      if (++i < argc)
        strncpy(save_file, argv[i], 100);
      continue;
    }
    else
      tld.loadData(loading, argv[i]);
  }

  printf("Training classifier...\n");
  tld.train();

  printf("Evlauating classifier...\n");
  tld.test();

  if (strlen(save_file) > 0)
  {
    printf("Saving classifier as: %s\n", save_file);
    tld.save(save_file);
  }

  ros::spin();
  return 0;
}
