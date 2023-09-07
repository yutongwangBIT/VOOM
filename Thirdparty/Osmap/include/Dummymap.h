/**
* This file is part of OSMAP.
*
* Copyright (C) 2018-2019 Alejandro Silvestri <alejandrosilvestri at gmail>
* For more information see <https://github.com/AlejandroSilvestri/osmap>
*
* OSMAP is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OSMAP is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OSMAP. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DUMMYMAP_H_
#define DUMMYMAP_H_

#include <set>
#include <map>
#include <opencv2/core.hpp>

namespace ORB_SLAM2{

class KeyFrame;
class Map;
class Osmap;

class MapPoint{
public:
  static unsigned int nNextId;
  unsigned int mnId=0;
  bool mbBad = false;
  cv::Mat mWorldPos;
  int mnVisible=0;
  int mnFound=0;
  cv::Mat mDescriptor;
  KeyFrame *mpRefKF = NULL;
  std::map<KeyFrame*,size_t> mObservations;
  Map *mpMap = NULL;

  MapPoint(Osmap*){};
  void AddObservation(KeyFrame*, size_t){}
  void SetBadFlag(){}
  void UpdateNormalAndDepth(){}
};

class KeyFrame{
public:
  static unsigned int nNextId;
  unsigned int mnId=0;
  bool mbBad = false;
  double mTimeStamp = 0;
  cv::Mat Tcw;
  cv::Mat mK;
  cv::Mat mDescriptors;
  int N=0;
  std::vector<cv::KeyPoint> mvKeysUn;
  std::vector<MapPoint*> mvpMapPoints;
  std::set<KeyFrame*> mspLoopEdges;

  bool mbNotErase=false;
  const int mnMinX=0, mnMinY=0, mnMaxX=2, mnMaxY=2, mnGridCols=1, mnGridRows=1;
  const float mfGridElementWidthInv=.5, mfGridElementHeightInv=.5;
  KeyFrame *mpParent = NULL;
  std::vector<std::vector<std::vector<size_t> > > mGrid;
  std::map<KeyFrame*, int> mConnectedKeyFrameWeights;
  std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;

  KeyFrame(Osmap*){};
  void ComputeBoW(){}
  void SetPose(cv::Mat){}
  void UpdateConnections(){}
  void SetBadFlag(){}
  void ChangeParent(KeyFrame *pKF){}
};


class Map{
public:
  std::set<MapPoint*> mspMapPoints;
  std::set<KeyFrame*> mspKeyFrames;
  std::vector<KeyFrame*> mvpKeyFrameOrigins;
  long unsigned int mnMaxKFid;
};

class KeyFrameDatabase{
public:
	void add(KeyFrame *pKF){}
	void clear(){}
};

class Frame{};

class System{
public:
	Map *mpMap;
	KeyFrameDatabase *mpKeyFrameDatabase;

	class Tracker{
	public:
		Frame mCurrentFrame;
		int mState = 0;
		void Reset(){};
	} *mpTracker;
	class DummyClasses{
	public:
		void RequestStop(){};
		bool isStopped(){return true;};
		void Release(){};
		void Update(Tracker*){};
	} *mpLocalMapper, *mpViewer, *mpFrameDrawer;
};

}	// namespace ORB_SLAM2

#endif /* DUMMYMAP_H_ */
