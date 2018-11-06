#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main(int argc, char *argv[]) {
  if (argc != 3) {
    cout << "usage: feature img1 img2" << endl;
    return 1;
  }

  Mat img1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
  Mat img2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

  std::vector<KeyPoint> keypoints1, keypoints2;
  Mat descriptors1, descriptors2;
  Ptr<ORB> orb = ORB::create(500, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE, 31, 20);

  orb->detect(img1, keypoints1);
  orb->detect(img2, keypoints2);
  orb->compute(img1, keypoints1, descriptors1);
  orb->compute(img2, keypoints2, descriptors2);

  Mat outimg1;
  drawKeypoints(img1, keypoints1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
  imshow("ORB features", outimg1);

  vector<DMatch> matches;
  BFMatcher matcher(NORM_HAMMING);
  matcher.match(descriptors1, descriptors2, matches);

  double min_dist = 10000, max_dist = 0;
  for (int i = 0; i < descriptors1.rows; ++i) {
    double dist = matches[i].distance;
    if (dist < min_dist) {
      min_dist = dist;
    }
    if (dist > max_dist) {
      max_dist = dist;
    }
  }
  cout << "-- Max dist: " << max_dist << endl;
  cout << "-- Min dist: " << min_dist << endl;

  vector<DMatch> good_matches;
  for (int i = 0; i < descriptors1.rows; ++i) {
    if (matches[i].distance <= max(2 * min_dist, 30.)) {
      good_matches.push_back(matches[i]);
    }
  }

  Mat img_match;
  Mat img_goodmatch;
  drawMatches(img1, keypoints1, img2, keypoints2, matches, img_match);
  drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_goodmatch);
  imshow("all matches", img_match);
  imshow("good matches", img_goodmatch);
  waitKey(0);
}
