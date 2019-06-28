#include "TrackBase.h"


using namespace ov_core;


void TrackBase::display_active(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2) {

    // Get the largest width and height
    int max_width = -1;
    int max_height = -1;
    for(auto const& pair : img_last) {
        if(max_width < pair.second.cols) max_width = pair.second.cols;
        if(max_height < pair.second.rows) max_height = pair.second.rows;
    }

    // Return if we didn't have a last image
    if(max_width==-1 || max_height==-1)
        return;

    // If the image is "new" then draw the images from scratch
    // Otherwise, we grab the subset of the main image and draw on top of it
    bool image_new = ((int)img_last.size()*max_width != img_out.cols || max_height != img_out.rows);

    // If new, then resize the current image
    if(image_new) img_out = cv::Mat(max_height,(int)img_last.size()*max_width,CV_8UC3,cv::Scalar(0,0,0));

    // Loop through each image, and draw
    for(auto const& pair : img_last) {
        // select the subset of the image
        cv::Mat img_temp;
        if(image_new) cv::cvtColor(img_last[pair.first], img_temp, CV_GRAY2RGB);
        else img_temp = img_out(cv::Rect(max_width*pair.first,0,max_width,max_height));
        // draw, loop through all keypoints
        for(size_t i=0; i<pts_last[pair.first].size(); i++) {
            // Get bounding pts for our boxes
            cv::Point2f pt_l = pts_last[pair.first].at(i).pt;
            // Draw the extracted points and ID
            cv::circle(img_temp, pt_l, 2, cv::Scalar(r1,g1,b1), CV_FILLED);
            //cv::putText(img_out, std::to_string(ids_left_last.at(i)), pt_l, cv::FONT_HERSHEY_SIMPLEX,0.5,cv::Scalar(0,0,255),1,cv::LINE_AA);
            // Draw rectangle around the point
            cv::Point2f pt_l_top = cv::Point2f(pt_l.x-5,pt_l.y-5);
            cv::Point2f pt_l_bot = cv::Point2f(pt_l.x+5,pt_l.y+5);
            cv::rectangle(img_temp,pt_l_top,pt_l_bot, cv::Scalar(r2,g2,b2), 1);
        }
        // Draw what camera this is
        cv::putText(img_temp, "CAM:"+std::to_string((int)pair.first), cv::Point(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 3.0, cv::Scalar(0,255,0),3);
        // Replace the output image
        img_temp.copyTo(img_out(cv::Rect(max_width*pair.first,0,img_last[pair.first].cols,img_last[pair.first].rows)));
    }

}


void TrackBase::display_history(cv::Mat &img_out, int r1, int g1, int b1, int r2, int g2, int b2) {

    // Get the largest width and height
    int max_width = -1;
    int max_height = -1;
    for(auto const& pair : img_last) {
        if(max_width < pair.second.cols) max_width = pair.second.cols;
        if(max_height < pair.second.rows) max_height = pair.second.rows;
    }

    // Return if we didn't have a last image
    if(max_width==-1 || max_height==-1)
        return;

    // If the image is "new" then draw the images from scratch
    // Otherwise, we grab the subset of the main image and draw on top of it
    bool image_new = ((int)img_last.size()*max_width != img_out.cols || max_height != img_out.rows);

    // If new, then resize the current image
    if(image_new) img_out = cv::Mat(max_height,(int)img_last.size()*max_width,CV_8UC3,cv::Scalar(0,0,0));


    // Max tracks to show (otherwise it clutters up the screen)
    //size_t maxtracks = 10;
    size_t maxtracks = (size_t)-1;

    // Loop through each image, and draw
    for(auto const& pair : img_last) {
        // select the subset of the image
        cv::Mat img_temp;
        if(image_new) cv::cvtColor(img_last[pair.first], img_temp, CV_GRAY2RGB);
        else img_temp = img_out(cv::Rect(max_width*pair.first,0,max_width,max_height));
        // draw, loop through all keypoints
        for(size_t i=0; i<ids_last[pair.first].size(); i++) {
            // Get the feature from the database
            Feature* feat = database->get_feature(ids_last[pair.first].at(i));
            // Skip if the feature is null
            if(feat == nullptr || feat->uvs[pair.first].empty())
                continue;
            // Draw the history of this point (start at the last inserted one)
            for(size_t z=feat->uvs[pair.first].size()-1; z>0; z--) {
                // Check if we have reached the max
                if(feat->uvs[pair.first].size()-z > maxtracks)
                    break;
                // Calculate what color we are drawing in
                int color_r = r2-(int)(r1/feat->uvs[pair.first].size()*z);
                int color_g = g2-(int)(g1/feat->uvs[pair.first].size()*z);
                int color_b = b2-(int)(b1/feat->uvs[pair.first].size()*z);
                // Draw current point
                cv::Point2f pt_c(feat->uvs[pair.first].at(z)(0),feat->uvs[pair.first].at(z)(1));
                cv::circle(img_temp, pt_c, 2, cv::Scalar(color_r,color_g,color_b), CV_FILLED);
                // If there is a next point, then display the line from this point to the next
                if(z+1 < feat->uvs[pair.first].size()) {
                    cv::Point2f pt_n(feat->uvs[pair.first].at(z+1)(0),feat->uvs[pair.first].at(z+1)(1));
                    cv::line(img_temp, pt_c, pt_n, cv::Scalar(color_r,color_g,color_b));
                }
                // If the first point, display the ID
                if(z==feat->uvs[pair.first].size()-1) {
                    //cv::putText(img_out0, std::to_string(feat->featid), pt_c, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
                    //cv::circle(img_out0, pt_c, 2, cv::Scalar(color,color,255), CV_FILLED);
                }
            }
        }
        // Draw what camera this is
        cv::putText(img_temp, "CAM:"+std::to_string((int)pair.first), cv::Point(30,60), cv::FONT_HERSHEY_COMPLEX_SMALL, 3.0, cv::Scalar(0,255,0),3);
        // Replace the output image
        img_temp.copyTo(img_out(cv::Rect(max_width*pair.first,0,img_last[pair.first].cols,img_last[pair.first].rows)));
    }

}


