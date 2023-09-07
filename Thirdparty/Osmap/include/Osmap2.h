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



#ifndef OSMAP_H_
#define OSMAP_H_

//#include <string>
#include <set>
#include <vector>
#include <map>
#include <bitset>
#include <memory>
#include <iterator>
#include "osmap.pb.h"
#include <set>
#include <opencv2/core/core.hpp>

#ifdef OSMAP_DUMMY_MAP

// Don't include when porting osmap to os1 and orb-slam2
#include <Dummymap.h>

#else

// Only for use with OrbSlam2
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "System.h"
#include "Frame.h"
#include "System.h"
#include "Tracking.h"
#include "Ellipsoid.h"
#include "Object.h"
#include "Graph.h"

#endif
namespace ORB_SLAM2{
class Osmap;

/**
 * Wrapped MapPoint to let Osmap access protected properties without modifying MapPoint code.
 * It adds a constructor used by Osmap when loading maps.
 */
class OsmapMapPoint: public MapPoint{
public:
	friend class Osmap;
	OsmapMapPoint(Osmap*);
};


class OsmapObject: public Object{
public:
	friend class Osmap;
	OsmapObject(const Ellipsoid& ell);

  Ellipsoid ellipsoid;
  unsigned object_id;
};


/**
 * Wrapped KeyFrame to let Osmap access protected properties without modifying KeyFrame code.
 * It adds a constructor used by Osmap when loading maps.
 */
class OsmapKeyFrame: public KeyFrame{
public:
	friend class Osmap;
	OsmapKeyFrame(Osmap*);
};


/**
 * Wrapped Map to let Osmap access protected properties without modifying Map code.
 * This class is only used for casting, not to instantiate new maps.
 */
class OsmapMap: public Map{
public:
	friend class Osmap;
};

class OsmapTracking: public Tracking{
public:
	friend class Osmap;
};

struct Node {
  int id;
  int object_id;
  int category_id;
  std::vector<int> neighbours;
  Eigen::Vector4d bbox;
};

/**
 * FEATURES_MESSAGE_LIMIT is the maximum number of features allowed in a single protocol buffer's message, to avoid known size problems in protocol buffers.
 * When a map exceed this limit, mapSave will save features file in delimited form, sequencing many messages each of them below this limit.
 * This constant can be defined elsewhere before this point, or else is defined here.
 */
#ifndef FEATURES_MESSAGE_LIMIT
#define FEATURES_MESSAGE_LIMIT 1000000
#endif

using namespace std;
using namespace cv;


/**
This is a class for a singleton attached to ORB-SLAM2's map.

Its constructor attaches it to the map.  Its methods mapLoad and mapSave load and save a map.

A map is saved to a set of files in the actual folder, with a filename provided:
- filename.keyframes
- filename.mappoints
- filename.features
- filename.yaml, the header

The header is the only text file, in yaml format.  Other files are in binary format.
keyframes, mappoints and features files consist on a single protocol buffers 3 message.
features file can also be an ad hoc delimited array of protocol buffers 3 messages.

Protocol buffers messages format can be found in osmap.proto file.
Some of these objects has another object like KeyPoint, nested serialized with the appropiate serialize signature.


Methods:

- serialize(object, message): this method provides many signatures for many pairs {object, message},const
where object, passed by reference, is the instance to be serialized (Mat, MapPoint, KeyFrame, etc.) and message is the protocol buffers destination object.
- deserialize(message, object): this method provides many signatures for many pairs {object, message},
where message is the protocol buffers source object, and object (Mat, MapPoint, KeyFrame, etc.) is the actual destination.
For MapPoint and KeyFrame, object can be either passed by reference or not provided at all, the method creates an empty one, and returns it deserialized.
- messageArray is a message with only one field of repeated messages.  messageArray serialized MapPoints, KeyFrames, Features...

General rules:
- deserialize get a const message
- serialize need a mutable message, it asks for a pointer to mutable message, as provided by protocol buferrs' mutable_field and add_field
- serializeArray and deserializeArray serialize a messageArray, i.e. set of objects
- unlike serialize, serializeArray uses a messageArray, not a pointer to it.

Usage example:

In Orb-Slam2's main.cc, this code will save and load a map:

    ...
    // Construct the osmap object, can be right after SLAM construction.  You only need one instance to load and save as many maps you want.
    Osmap osmap = ORB_SLAM2::Osmap(SLAM);
	...
	// Whe you already has a map to save
	osmap.mapSave("myFirstMap");	// "myFirstMap" or "myFirstMap.yaml", same thing
	...
	// Now you want to load the map
	osmap.mapLoad("myFirstMap.yaml");

In Linux, zenity comes handy as dialog to get file name to load and save map.  You need zenity installed, this runs the command line:

	char cstringfilename[1024];

	// Load dialog
	FILE *f = popen("zenity --file-selection", "r");
    fgets(cstringfilename, 1024, f);

    // Save dialog
    FILE *f = popen("zenity --file-selection --save --confirm-overwrite --filename=mapa", "r");
    fgets(cstringfilename, 1024, f);


You probably want to go to localization only mode (tracking only, no mapping) right before saving or loading.  To do that:

    SLAM->ActivateLocalizationMode();

User also can control this with the button "Localization mode" in viewer.


*/
class Osmap{
public:
  // Properties

  /**
  All the options available.  Options are persetted into options property.  Default is zero.  Options are activated with 1.

  To test an option:

      if(options[ORB_SLAM2::Osmap::ONLY_MAPPOINTS_FEATURES])...

  To set an option:

      options.set(ORB_SLAM2::Osmap::ONLY_MAPPOINTS_FEATURES);

  Available options are documented in enum Options.  New options can be added in the future, but the order of existing options must be keeped.  So, new options must be added to the end of enum.

  Time stamp is not optional because it always occupy place in protocol buffers, as all numeric fields.  Defaults to 0.

  Options are saved in the map file.  NO_SET_BAD is the only option one can have insterest to set before loading, while not set in the file.
  */
  enum Options {
	  // Delimited to overcome a Protocol Buffers limitation.  It is automatic, but can be forced with this options:
	  FEATURES_FILE_DELIMITED,	/*!< Saves features file in delimited form: many protocol buffers messages in one file. */
	  FEATURES_FILE_NOT_DELIMITED,	/*!< Avoids saving features file in delimited form: only one protocol buffers message in the file, as always done with mappoints and keyframes. */

	  // Skip files, for analisys propuses
	  NO_MAPPOINTS_FILE,	/*!< Skip saving mappoints file.  Map will be incomplete, only for analysis porpouses. */
	  NO_KEYFRAMES_FILE,	/*!< Skip saving keyframes file.  Map will be incomplete, only for analysis porpouses. */
	  NO_FEATURES_FILE, 	/*!< Skip saving features file.  Map will be incomplete, only for analysis porpouses. */

	  // Shrink file
	  NO_FEATURES_DESCRIPTORS,	/*!< Skip saving descriptors in features file.  Descriptors are saved only in mappoints.  It is usually used with ONLY_MAPPOINTS_FEATURES to notable reduce features file size. */
	  ONLY_MAPPOINTS_FEATURES,	/*!< Skip saving features that not belong to mappoints.  This notable reduce features file size.  Map will be fine for tracking, may get a little hard (not impossible) to continue mapping. */

	  // Options
	  NO_LOOPS,			/*!< Skip saving and retrieving (loading) loops edges.  Not expected size reduction.  Map will work. For debug and analysis porpouses. */
	  K_IN_KEYFRAME,	/*!< Force saving K camera matrix on each keyframe instead of saving them on yaml file.  Keyframes file will increase. */

	  // Depuration and rebuild options
	  NO_DEPURATION,	/*!< Avoids map depuration process before save.  Depuration get rid of bad elements, but could ruin the map if illformed.  */
	  NO_SET_BAD,		/*!< Avoids bad mappoints and keyframe detection while rebuilding the map, right after load.  Used with dummy maps examples to prevent anomally detection, because dummy maps have incomplete class implementations. */
	  NO_APPEND_FOUND_MAPPOINTS,	/*!< On depuration process before save, avoids adding to the map any found good mappoint erroneously deleted from map. */

	  OPTIONS_SIZE	// /*!< Number of options.  Not an option. */
  };

  /**
  Set of chosen options for serializing.  User must set options prior to saving a map.  Loading a map should reflect in this property the saved map options.
  */
  bitset<32> options = 0;

  /** ORB-SLAM2 map to be serialized. */
  OsmapMap &map;
  OsmapTracking &tracker;

  /** Database of keyframes to be build after loading. */
  KeyFrameDatabase &keyFrameDatabase;

  /** System, needed to populate new keyframes after construction on deserialization, with some configuration values.*/
  System &system;

  /** Any frame, to copy configuration values from.  Constructor takes currentFrame from Tracker. */
  Frame &currentFrame;

  /**
   * This keyframe is a vehicle to MapPoint construction, which need a pRefKF as argument.
   * MapPoint construction occurs only while loading.
   * pRefKF can be any KeyFrame.
   * mapLoad can be invoked when there is no map in orbslam2, which means there is no keyframe in it.
   * So Osmap constructs one.  But constructed a KeyFrame requires a populated Frame (Osmap uses tracker.currentFrame),
   * and it isn't populated with data before first tracking loop.  This means pRefKF can't be constructed in Osmap constructor.
   * pRefKF keyframe is constructed on the first map load, more specifically right before mappoints are loaded.
   */
  KeyFrame *pRefKF = NULL;

  /**
  Usually there is only one common matrix K for all KeyFrames in the entire map, there can be more, but there won't be as many K as KeyFrames.
  This vector temporarily store different K for serialization and deserialization.  This avoids serializing one K per KeyFrame.
  This vector is consumed in keyframe deserialization.
  */
  vector<Mat const *> vectorK;

  /**
  For each index KeyFrame.mnId the value returned is an index to vectorK.
  The later is the index saved in each keyframe as k.
  It is populated only by getVectorKFromKeyframes, and consumed only while saving the map, during keyframe serialization.
  */
  vector<unsigned int> keyframeid2vectorkIdx;


  /**
   * Buffer where map's mappoints are stored in ascending id order, to save them to file in this order.
   * This vector is set in mapSave, and it's left ontouched for user interest.
   * This vector is consumed in serialize
   */
  vector<OsmapMapPoint*> vectorMapPoints;

  /**
   * Buffer where map's keyframes are stored in ascending id order, to save them to file in this order.
   * This vector is set in mapSave, and it's left ontouched for user interest.
   */
  vector<OsmapKeyFrame*> vectorKeyFrames;


  /**
   * Whether or not print on console info for debuging.
   */
  bool verbose = false;

  /**
  Only constructor, the only way to set the orb-slam2 map.
  */
  Osmap(System &_system);

  /**
  Saves the map to a set of files in the actual directory, with the extensionless name provided as the only argument and different extensions for each file.
  If filename has .yaml extension, mapSave will remove it to get the actual basefilename.
  Any existing file is rewritten without warning.
  This is the entry point to save a map.  This method uses the Osmap object to serialize the map to files.
  Before calling this method:
  - ORB-SLAM2 threads must be stopped to assure map is not being modify while saving.
  - Actual directory must be set to the desired destination.  Often a new directory exclusive for the map is created.
  - options must be set.

  @param basefilename File name without extenion or with .yaml extension.  Many files will be created with this basefilename and different extensions.
  @param pauseThreads Serializing needs some orb-slam2 threads to be paused.  true (the default value) signals mapSave to pause the threads before saving, and resume them after saving.  false when threads are paused and resumed by other means.

  MapSave copy map's mappoints and keyframes sets to vectorMapPoints and vectorKeyFrames and sort them, to save objects in ascending id order.
  MapLoad doesn't use those vector.

  If features number exceed an arbitrary maximum, in order to avoid size related protocol buffer problems,  mapSave limit the size of protocol buffer's messages saving features file in delimited form, using Kendon Varda writeDelimitedTo function.
  */
  void mapSave(string basefilename, bool pauseThreads = true);

  /**
  Loads the map from a set of files in the folder whose name is provided as an argument.
  This is the entry point to load a map.  This method uses the Osmap object to serialize the map to files.

  @param yamlFilename file name of .yaml file (including .yaml extension) describing a map.
  @param noSetBad true to avoid bad mappoints and keyframes deletion on rebuilding after loading.
  @param stopTrheads Serializing needs some orb-slam2 threads to be paused.  true (the default value) signals mapLoad to pause the threads before saving, and resume them after saving.  false when threads are paused and resumed by other means.

  Only these properties are read from yaml:
  - file nKeyframes
  - options
  - camera calibration matrices K
  - other files' names

  Before calling this method, threads must be paused.
  */
  void mapLoad(string yamlFilename, bool noSetBad = false, bool pauseThreads = true, string sensor = "mono");

  /**
   * Save the content of vectorMapPoints to file like "map.mappoints".
   * @param filename full name of the file to be created and saved.
   * @returns number of mappoints serialized.  -1 if error.
   *
   * This method ignores option NO_MAPPOINTS_FILE
   * If not K_IN_KEYFRAME option (the default), vectorK must be populated (see getVectorKFromKeyframes) prior invocation of this method.
   */
  int MapPointsSave(string filename);



  /**
   * Load the content of a "map.mappoints" file to vectorMapPoints.
   * @param filename full name of the file to open.
   */
  int MapPointsLoad(string filename);

  int ObjectsSave(string filename);

  int ObjectsLoad(string filename);


  /**
   * Save the content of vectorKeyFrames to file like "map.keyframes".
   * Need
   * @param filename full name of the file to be created and saved.
   * @returns number of keyframes serialized.  -1 if error.
   *
   * This method ignores option NO_KEYFRAMES_FILE
   */
  int KeyFramesSave(string filename);

  /**
   * Load the content of a "map.keyframes" file to vectorKeyFrames.
   * @param filename full name of the file to open.
   */
  int KeyFramesLoad(string filename);

  // GRAPH =======================
  int GraphsSave(string filename);
  int GraphsLoad(string filename);

  /**
   * Save KeyFrane's features of vectorKeyFrames to file, usually "map.features".
   * @param filename full name of the file to be created and saved.
   */
  int featuresSave(string filename);

  /**
   * Load the content of a "map.features" file and applies it to vectorKeyFrames.
   * @param filename full name of the file to open.
   */
  int featuresLoad(string filename);

  /**
   * Populate vectorMapPoints with MapPoints from Map.mspMapPoints.
   * This is done as the first step to save mappoints.
   */
  void getMapPointsFromMap();

  void getObjectsFromMap();
  void setObjectToMap();

  /**
   * Populate Map.mspMapPoints with MapPoints from vectorMapPoints.
   * This is done after loading mappoints.
   */
  void setMapPointsToMap();

  /**
   * Populate vectorKeyFrames with MapPoints from Map.mspKeyFrames.
   * This is done as the first step to save keyframes.
   */
  void getKeyFramesFromMap();

  /**
   * Populate Map.mspKeyFrames with MapPoints from vectorKeyFrames.
   * This is done after loading keyframes.
   */
  void setKeyFramesToMap();




  /**
   * Irons keyframes and mappoints sets in map, before save.
   *
   * Tests all keyframes in MapPoints.mObservations and mappoints in KeyFrames.mvpMapPoints, checking they are:
   *
   * - not bad (!isBad())
   * - in map (in Map::mspMapPoints and Map::mspKeyFrames)
   *
   * Those who not pass this check are eliminated, avoiding serialization of elements not belonging to the map.
   *
   * This depuration affects (improves) the actual map in memory.
   *
   * Invoked by mapSave.
   *
   */
  void depurate();

  /**
   * Rebuilding takes place right after loading a map from files, before the map is copied to ORB-SLAM2' sets.
   *
   * Works on Osmap::vectorMapPoints and Osmap::vectorKeyFrames.
   * After rebuilding, the elements in these vector should be copied to the map sets.
   *
   * rebuild() needs KeyPoints and MapPoints to be initialized with a lot of properties set,
   * as the default constructors for OsmapKeyFrame and OsmapMapPoint show.
   *
   * @param noSetBad true to avoid bad mappoints and keyframes deletion on rebuilding.
   *
   * This method checks Osmap::verbose property to produce console output for debugging purposes.
   *
   *
   * How rebuild works:
   *
   *  - Loops on every keyframe:
   * 		- ComputeBOW, building BOW vectors from descriptors
   * 		- Builds many pose matrices from pose
   * 		- Builds the grid
   * 		- Adds to KeyFrameDatabase
   * 		- Builds its mappoints observations
   * 		- UpdateConnections, building the spaning tree and the covisibility graph
   *  - Sets KeyFrame::nNextId
   *  - Retries UpdateConnections on isolated keyframes.
   *  - Sets bad keyframes remaining isolated (avoided with noSetBad argument true)
   *  - Sets map.mvpKeyFrameOrigins
   *  - Travels keyframes looking for orphans, trying to assign a parent, thus including them in the spanning tree.
   *  - Loops on every mappoint:
   * 		- Sets bad mappoints without observations (avoided with noSetBad argument true)
   * 		- Sets mpRefKF
   * 		- UpdateNormalAndDepth, setting mNormalVector, mfMinDistance, mfMaxDistance
   *  - Sets MapPoint::nNextId
   *
   */
  void rebuild(bool noSetBad = false);



  /**
   * Clear temporary vectors.
   *
   * Only clears the vectors, not the objects its pointer elements pointed to (keyframes, mappoints, K matrices), because they belong to the map.
   *
   */
  void clearVectors();

  /**
   * Utility for separating path and filename from a full path.
   * @param filename Output string, filename found in path, '' if no file in the path.  Can be NULL.
   * @param pathDirectory Output string with the directory's path, '' if none.  Can be NULL.
   */
  void parsePath(const string &path, string *filename = NULL, string *pathDirectory = NULL);

  /**
  Traverse map's keyframes looking for different K matrices, and stores them into vectorK.
  It also populates keyframeid2vectork.
  */
  void getVectorKFromKeyframes();

  /**
  Looks for a mappoint by its id in the map.
  @param id Id of the MapPoint to look for.
  @returns a pointer to the MapPoint with the given id, or NULL if not found.
  */
  MapPoint *getMapPoint(unsigned int id);


  /**
  Looks for a KeyFrame id in the map.
  Keyframes are usually stored in ascending id order.  This function will be more probably called in the same way, so it is optimized for this expected behaviour.  It uses itLastKF.
  @param id Id of the KeyFrame to look for.
  @returns a pointer to the KeyFrame with the given id, or NULL if not found.
  Used only in Osmap::deserialize(const SerializedKeyframeFeatures&).
  */
  OsmapKeyFrame *getKeyFrame(unsigned int id);


  /**
   * Count the number of features in vectorKeyFrames.
   * Invoked by mapSave to decide to use or not delimited form of feature file.
   */
  int countFeatures();



  // Protocol buffer messages serialization for orb-slam2 objects


  // K matrix ====================================================================================================

  /**
  Serialize provided intrinsic camera matrix K to the protocol buffer message.
  All 4 fields required.
  @param k Source to be serialized.  K matrix, also known as calibration matrix, instrinsic matrix and camera matrix, 3x3 float Mat, usually from KeyFrame::mK.
  @param serializedK Protocol buffers destination message to be serialized.
  */
  void serialize(const Mat &k, SerializedK *serializedK);

  /**
  Reconstruct the intrinsic camera matrix K from protocol buffer message.
  All 4 fields are required.
  @param serializedK Protocol buffers source message.
  @param k Destination to be reconstruted.  K matrix, also known as calibration matrix, instrinsic matrix and camera matrix, 3x3 float Mat, usually from KeyFrame::mK.

  */
  void deserialize(const SerializedK&, Mat& k);


  /**
  Serialize an array of camera calibration matrix K.
  The array should be retrieved from keyframes analisys.
  Usually there is only one K common to all keyframes.
  @param vK The vector of K matrices to serialize.
  @param serializedKArray The serialization destination object.
  */
  void serialize(const vector<Mat*> &vK, SerializedKArray &serializedKArray);


  /**
  Reconstruct an array of camera calibration matrix K.
  The array should be retrieved from keyframes analisys.
  Usually there is only one K common to all keyframes.
  @param serializedKArray The serialization destination object.
  @param vK The vector of K matrices to serialize.  Usually Osmap::vectorK member.

  */
  void deserialize(const SerializedKArray &serializedKArray, vector<Mat*> &vK);


  // Descriptor ====================================================================================================

  /**
  Serializes a descriptor, an 1x32 uchar Mat (256 bits).
  Protocol Buffers doesn't have a Byte type, so it's serialized to 8 int.
  Exactly 8 int required.
  */
  void serialize(const Mat&, SerializedDescriptor*);


  /**
  Reconstruct a descriptor, an 1x32 uchar Mat (256 bits).
  Exactly 8 int required.
  */
  void deserialize(const SerializedDescriptor&, Mat&);



  // Pose matrix ====================================================================================================

  /**
  Serialize a 4x4 float Mat representing a pose in homogeneous coordinates.
  Exactly 12 float required.
  */
  void serialize(const Mat&, SerializedPose*);


  /**
  Reconstruct a 4x4 float Mat representing a pose in homogeneous coordinates.
  Exactly 12 float required.
  */
  void deserialize(const SerializedPose&, Mat&);


  // ELlipsoid ====================================================================================================

  /**
  Serialize a Eigen::Matrix4d representing a pose an ellipsoid (dual matrix).
  */
  void serialize(const Ellipsoid&, SerializedEllipsoid*);


  /**
  Reconstruct a Eigen::MAtrix4d representing an ellipsoid.
  */
  void deserialize(const SerializedEllipsoid&, Ellipsoid&);


  // BBox2 ====================================================================================================

  /**
  Serialize a BBox2
  */
  void serialize(const BBox2&, SerializedBBox2*);


  /**
  Reconstruct a BBox2
  */
  void deserialize(const SerializedBBox2&, BBox2&);



  // Position vector ====================================================================================================

  /**
  Serialize 3D mappoint position, a 3x1 float Mat.
  All 3 fields required.
  */
  void serialize(const Mat&, SerializedPosition*);

  /**
  Reconstructs 3D mappoint position in a 3x1 float Mat.
  All 3 fields required.
  */
  void deserialize(const SerializedPosition&, Mat&);


  // KeyPoint ====================================================================================================
  /**
  Serialize 4 properties of a KeyPoint.
  All 4 fields required.
  */
  void serialize(const KeyPoint&, SerializedKeypoint*);


  /**
  Reconstructs a KeyPoint.
  All 4 fields required.
  */
  void deserialize(const SerializedKeypoint&, KeyPoint&);






  // MapPoint ====================================================================================================

  /**
  Serializes a MapPoint, according to options.
  */
  void serialize(const OsmapMapPoint&, SerializedMappoint*);

  /**
  Creates and fills a MapPoint from optional message fields.
  It doesn't perform MapPoint initialization.  This should be done after deserialization.
  @param serializedMappoint Protocol buffers source message.
  @returns *MapPoint A pointer to the newly created MapPoint, ready to be added to the map.
  */
  OsmapMapPoint *deserialize(const SerializedMappoint& serializedMappoint);

  /**
  Serialized array of MapPoints.
  The serialized message can be saved to an exclusive file, or be appended to a multiobject file.
  @param start Inclusive begining of the range of MapPoints to be serialized.  Usually map.mspMapPoints.begin().
  @param end Exclusive end of the range of MapPoints to be serialized.  Usually map.mspMapPoints.end().
  @param serializedMapPointArray message to set up.  Data comes from the range iterated.
  @returns Number of MapPoints serialized or -1 if error.  The number of MapPoints serialized should be the same number of MapPoints in the map.
  */
  int serialize(const vector<OsmapMapPoint*>&, SerializedMappointArray &);

  /**
  Retrieves MapPoints from an array, and append them to the map.
  @param serializedMapPointArray message to set up.  Data goes to the output iterator.
  @param output iterator on the destination container.  Usually std::inserter(mspMapPoints, mspMapPoints.end()).  For vector it can be a Back inserter iterator where output is placed in arrival order.  Usually mspMapPoints.begin().  About set back_inserter: https://stackoverflow.com/questions/908272/stdback-inserter-for-a-stdset
  @returns Number of MapPoints retrieved or -1 if error.
  Map's MapPoints set should be emptied before calling this method.
  */
  int deserialize(const SerializedMappointArray &, vector<OsmapMapPoint*>&);


  // KeyFrame ====================================================================================================

  /**
  Serialize a KeyFrame.
  Serialization and deserialization assume KeyFrames are processed in ascending id order.
  */
  void serialize(const OsmapKeyFrame&, SerializedKeyframe*);

  /**
  Reconstructs a KeyFrame from optional fields.
  It doesn't perform KeyFrame initialization.  This should be done after deserialization.
  */
  OsmapKeyFrame *deserialize(const SerializedKeyframe&);

  /**
  Serialized array of KeyFrames.  This can make a file, or be appended to a multiobject file.
  KeyFrames will be serialized in ascending id order.
  @param serializedKeyFrameArray message to set up.  Data comes from map.
  @returns Number of KeyFrames serialized or -1 if error.  The number of KeyFrames serialized should be the same number of MapPoints in the map.
  */
  int serialize(const vector<OsmapKeyFrame*>&, SerializedKeyframeArray&);

  /**
  Retrieves MapPoints from an array, and append them to the map.
  @param serializedMapPointArray message to set up.  Data goes from map.
  @returns Number of MapPoints retrieved or -1 if error.
  Map's MapPoints set should be emptied before calling this method.
  */
  int deserialize(const SerializedKeyframeArray&, vector<OsmapKeyFrame*>&);


  //GRAPH ====================================================================================================
  int serialize(const vector<OsmapKeyFrame*>& vectorKF, SerializedGraphArray &serializedGraphArray);

  void serialize(Graph&, SerializedGraph*);
  void serialize(Node, SerializedNode*);

  int deserialize(const SerializedGraphArray&, vector<OsmapKeyFrame*>&);
  void deserialize(const SerializedGraph &, Graph*);
  void deserialize(const SerializedNode&, Node&);

  // Feature ====================================================================================================

  /**
  Serialize all features observed by a KeyFrame.
  @param pKF KeyFrame owner of the feature.  The features will be stored in this KeyFrame containers.
  @param SerializedKeyframeFeatures Message destination of serialization.
  @returns The serialized message object.
  */
  void serialize(const OsmapKeyFrame&, SerializedKeyframeFeatures*);

  /**
  Retrieves all features belonging to one keyframe.
  First it takes the required keyframe_id field, and look in map for the keyframe with that id.  Hence, keyframes must be already deserialized in map.

  Puts all KeyPoints, MapPoints* and descriptors in their respective containters of the provided KeyFrame.

  @param SerializedKeyframeFeatures Object to be deserialized.
  */
  OsmapKeyFrame *deserialize(const SerializedKeyframeFeatures&);

  /**
   * Serialize all keyframe's features from provided keyframes container, to the specified serialization object.
   * @param vKF vector of KeyFrames to save, usually vectorKeyFrames member order by mnId.
   * @param serializedKeyframeFeaturesArray output serialization object.
   */
  int serialize(const vector<OsmapKeyFrame*>& vKF, SerializedKeyframeFeaturesArray& serializedKeyframeFeaturesArray);

  /**
   * Retrieves all keyframe's features from the specified serialization object to vectorKeyframe.
   */
  int deserialize(const SerializedKeyframeFeaturesArray&);


  /**
  Writes a message to a vector file.  A vector file is an array of messages.
  In order to possibilitate deserialization, the message size is prepended to the message.
  @param message
  @param rawOutput output stream, writable destination file.
  @returns true if ok, false if error.

  Kendon Varda code to serialize many messages in one file, from https://stackoverflow.com/questions/2340730/are-there-c-equivalents-for-the-protocol-buffers-delimited-i-o-functions-in-ja

  When writing to a file, *rawOutput object must be deleted before closing the file, to ensure flushing.
  */
  bool writeDelimitedTo(
    const google::protobuf::MessageLite& message,
    google::protobuf::io::ZeroCopyOutputStream* rawOutput
  );

  /**
  Reads a message to a vector file.  A vector file is an array of messages.
  @param message
  @param rawOutput input stream, readable source file.
  @returns true if ok, false if error.

  Kendon Varda code to serialize many messages in one file, from https://stackoverflow.com/questions/2340730/are-there-c-equivalents-for-the-protocol-buffers-delimited-i-o-functions-in-ja
  */
  bool readDelimitedFrom(
    google::protobuf::io::ZeroCopyInputStream* rawInput,
    google::protobuf::MessageLite* message
  );


  // Modified LOG function from https://stackoverflow.com/questions/29326460/how-to-make-a-variadic-macro-for-stdcout
  void log() {cout << endl;}
  template<typename Head, typename... Args> void log(const Head& head, const Args&... args ){
  	if(verbose){
  		std::cout << head << " ";
  		log(args...);
  	}
  }

};



}	// namespace ORB_SLAM2

#endif /* OSMAP_H_ */
