#  HTW -  Hackathon
## Lidar Segmentation Example 
 
### Version 
0.1
  
### Author 
Sven Eckelmann (eckelmann@htw-dresden.de) 

### Synopsis
Extracting surface and lane marking points from a lidar scan relative to our vehicle position 
The algorithm  
* uses pass through filter in z direction 
* applies a ransac to extract only surface points and 
* uses a pass through for intensity to extract reflecting points 


* subscribed topic:
    + /vlp_102/velodyne_points           
 

* publishs topic:
    + /cloud_seg/ground    #represents the surface 
    + /cloud_seg/lane      #represents the  lane marking points  
    + /cloud_seg/not_segmented          # all non segmented points
   
## Motivation
Creating an lane assist based on lidar information 

## Dependencies:
- pcl 
- eigen3
  
 

## Installation
Download the source in your /catkin/src directory and run catkin_make 
Run node with:   
`rosrun lidar_segementation_example lidar_segementation_example_node ` or  

## Hints / TodDos 
* remove static threshold for pass through intensity filter 
* create spline based on lane marking points 
* and so on ...
## Tests
not done ... 

 

    
