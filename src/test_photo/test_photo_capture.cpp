/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Robert Bosch LLC.
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
 *   * Neither the name of the Robert Bosch nor the names of its
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
 *
 *********************************************************************/
#include <iostream>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/highgui.h> //for cvSaveImage()
#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;

#include <iai_photo/photo.h>
#include <iai_photo/GetConfig.h>
#include <iai_photo/SetConfig.h>
#include <iai_photo/Capture.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_photo");

  cv_bridge::CvImagePtr cv_ptr;

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<iai_photo::Capture>("/photo/capture");
  iai_photo::Capture srv;
  std::stringstream filename;
  filename<<"image_"<<std::time(NULL)<<".jpg";
  if(client.call(srv))
  {
    printf("Calling photo/capture service was successful\n");
    cv_ptr = cv_bridge::toCvCopy(srv.response.image, enc::BGR8);
    IplImage tmp = cv_ptr->image;
    cvSaveImage(filename.str().c_str(), &tmp);
    printf("Saved photo: %s\n",filename.str().c_str());
  }
  else
  {
    printf("Could not query photo/capture service\n");
  }

  return 0;
}

