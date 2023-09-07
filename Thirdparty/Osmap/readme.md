# Osmap: ORB-SLAM2 Map serialization

[Osmap documentation with Doxygen](https://alejandrosilvestri.github.io/osmap/html/class_o_r_b___s_l_a_m2_1_1_osmap.html)

OSMAP stands for Orb-Slam2 Map.  It's a serialization addendum for ORB-SLAM2.

ORB-SLAM2 is a visual SLAM that generate a point cloud (sort of) map from a video stream, so it can localize itself in that map.  ORB-SLAM2 code in GitHub ar https://github.com/raulmur/ORB_SLAM2 is open source, and was made as a proof of concept to support its paper *ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras.*

ORB-SLAM2 is not a final product.  It generates maps but can't save them to a file, neither load them from a file.  Many proposals was made to add this functionality to ORB-SLAM2 code on GitHub, but saving maps to files ir beyond the project's porpuse.

After analizing many serializartion proposals, making one algorithm myself and having some experience with map files, I decide to restart this project from scratch and define a file format with broader goals in mind:

- Flexible evolution: let improve the format while maintaining compatibility
- Mind the size: shrink when possible, reconstruct when desirable.
- Format with options: map files can grow huge.  Let the user decide which parts to save and which parts to automatically reconstruct.
- Transparent, accesible and well documented: simplify format for debug and analisys, so third developers can do code with different porpuses than running on orb-slam2, like visualizing the map, editing it, analisyng its content, converting from a format to another, etc.  The serialization format can be used to real-time transfer the map to a visualizer via network.

This is why this serialization format has the following features:

- One map is serialized in many files.  If you want to analise mappoints only, you access that file alone.  In the future these files could be unified in a single zip file.
- Protocol buffers instead of boost::serialization.  proto files are a way to both document and generate serialization code.  Plus it applies some data compression.
- YAML as a header, mainly because ORB-SLAM2 already uses YAML, otherwise it could be JSON or XML.  That human readable header helps people who wants to analise or manipulate the map.


## The files in the project
- osmap.proto is the only protocol buffer definition, that serves as format documentation.

- osmap.pb.cc and osmap.pb.h are protocol buffer automatically generated code that defines messages classes.

- osmap.cpp and osmap.h defines the osmap class responsible for saving and loading maps.

- dummymap.h is provided to load and save a map without having to compile you application with orbslam2.  You can make map analisys applications without the burden of compiling with orbslam2.  To use dummymap.h instead of orbslam2's map you must define the preprocessor symbol OSMAP\_DUMMY\_MAP in you environment.  It is only used in osmap.h.

Example folder has some test files, which create some dummy map, saves it, loads it and show its values to verify the whole process.



## Standalone example
Osmap is an addon for ORB-SLAM2.  Next section is about appending Osmap to ORB-SLAM2.  However, Osmap includes an example code that let you test the implementation on a dummy map: it serializes the map thus creating the files, and then reads those files and show data on console.

To properly compile this test, you must define *OSMAP_DUMMY_MAP* symbol.

This example's main purpouse is to show how to use Osmap without ORB-SLAM2.  An application that do this is [Osmap viewer](https://github.com/AlejandroSilvestri/Osmap-viewer), that let you see the 3D mappoint cloud you saved in a file. 


## How to bundle with ORB-SLAM2
Right now osmap aims monocular SLAM only, so it won't serialize some variables needed for stereo SLAM and RGB-D SLAM.  But osmap could be extended to do that.

Because I don't pretend osmap be added to Raúl Mur's ORB-SLAM2, and because that project could still receive minor changes, this is the recipe to merge osmap with up to date orb-slam2.  It need some editing and compiling.

1- __Install Google's Protocol Buffers__ and generate __osmap.pb.cc__ and __osmap.pb.h__ with the following command line:

    $ protoc --cpp_out=. osmap.proto

From now on, you must use these locally generated files instead of the provided in this repository.

2- __Add Osmap files to ORB-SLAM2 project.__  Copy osmap.pb.cc and Osmap.cpp to src folder, and osmap.pb.cc and Osmap.h files to include folder.  You don't need the extra files: nor dummymap.h, nor osmap.proto, etc.

3- Modify System.h .  This is the only "atomic" editing to ORB-SLAM2 files: comment or delete __private:__ declaration line in [System.h](https://github.com/raulmur/ORB_SLAM2/blob/master/include/System.h#L125).  This is in line 125 since Dec 2016, last checked in 2019.

4- __Write the code to call save and load__, usually attached to UI.  As an example, in Orb-Slam2's main(), this code will save and load a map:

    ...
    #include "Osmap.h"
    ...
    // Construct the osmap object, can be right after SLAM construction.  You only need one instance to load and save as many maps you want.
    ORB_SLAM2::Osmap osmap = ORB_SLAM2::Osmap(SLAM);
    ...
    // When you already has a map to save
    osmap.mapSave("myFirstMap");	// "myFirstMap" or "myFirstMap.yaml", same thing
    ...
    // Now you want to load the map
    osmap.mapLoad("myFirstMap.yaml");
    
    // If you want to start ORB-SLAM2 loading a map, ensure GrabImageMonocular (or GrabImageRGBD or GrabImageStereo) is invoked at least once, because they initialize some objects needed to load the map.

5- __Compile__ with your IDE or command line, adding protobuf library (-lprotobuf in gcc), run.

OR

5 bis- __Compile with cmake.__  Instead of compiling by hand or with an IDE, you can modify ORB-SLAM2 CMakeLists.txt.  You will need to modify ORB-SLAM2 CMakeLists.txt with these additions:

    include(FindProtobuf)
    find_package(Protobuf REQUIRED)
    
    add_library(${PROJECT_NAME} SHARED
      src/Osmap.cpp
      src/osmap.pb.cc
    )
    
    include_directories(
      ${PROTOBUF_INCLUDE_DIR}
    )
    
    target_link_libraries(${PROJECT_NAME}
      ${PROTOBUF_LIBRARIES}
    )


Where? [right before building](https://github.com/raulmur/ORB_SLAM2/blob/f2e6f51cdc8d067655d90a78c06261378e07e8f3/CMakeLists.txt#L80).  You can really put these additions in many places inside CMakeLists.txt, after defining the project in line 2, and before building in line 80. 


### Known issues
#### Protocol buffers
Some users experienced problems due to bad versions of Protocol Buffers, like:

- Two different versions of Protocol Buffers installed, conflicting each other
- protoc and libprotobuf.so belonging to different versions
- Old version of Protocol Buffers

You can uninstall all versions of Protocol Buffers from your system, then install the last stable version.

Some skilled users edited CMakeLists.txt writing Protocol Buffers libraries full path instead of CMake variables.

All kind of compilation errors with osmap.pb.cc and osmap.pb.h tell you have a problem with your protocol buffers installation.  First step: check versions of protoc and the library cmake uses.

1- Check protoc version in a terminal

    $ protoc --version
    libprotoc 3.6.1

Your version will be different of this one.  Newer, I hope.

2- Check libprotobuf version cmakes found, looking in your long cmake output (the same output that gave you errors) something like this:

    -- Found Protobuf: /usr/local/lib/libprotobuf.so;-lpthread (found version "3.6.1") 

Both version should match.  If not, well, make them and try installation again.


#### Segmentation fault error
Some users reported this error when loading a map right after starting Orb-SLAM2, and it was solved.

If you have a seg fault crash, be sure you are using an updated Osmap and report it in the "Issues" tab.

## About save options
There are many options that let you optimize map file size.  Options must be set before calling mapSave.  Most relevant are:

### ONLY\_MAPPOINTS\_FEATURES

    osmap.options.set(ORB_SLAM2::Osmap::ONLY_MAPPOINTS_FEATURES, 1);   
    
ORB-SLAM2 detects a lot of features, but uses only those belonging to a mappoint.  The other are useful to find new mappoints, but the chance to do it are pretty small.  ONLY\_MAPPOINTS\_FEATURES skips saving those unwanted features, shrinking your map files A LOT, like 5 times smaller.


### NO\_FEATURES\_DESCRIPTORS

    osmap.options.set(ORB_SLAM2::Osmap::NO_FEATURES_DESCRIPTORS | ORB_SLAM2::Osmap::ONLY_MAPPOINTS_FEATURES, 1);   

This option saves the descriptor on each mappoints, avoiding saving it on each mappoint observation.  Using NO\_FEATURES\_DESCRIPTORS with ONLY\_MAPPOINTS\_FEATURES (it usually doesn't make sense using it alone) your map file will shrink A _LOTTER_, like 20 times smaller.
 
 
# Debugging
You can turn on verbose mode to fill your console with a ton of boring data that turns out to be useful if your application crash, usually with segmentation fault.

    osmap.verbose = true;
    
When commenting an Osmap issue, please paste this data.  Usually the important part is the last one, some lines right before crashing.

# OSMap status
OSMap is finished as for March 2020.  It is no longer maintained, as there were no new issues for a year.
