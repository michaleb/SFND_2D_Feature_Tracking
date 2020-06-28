# SFND 2D Feature Tracking

<img src="images/keyimages.png" width="820" height="248" />

The overall goal is to build a collision detection system using Lidar points and camera images. As a preliminary step a feature tracking component is required and for accuracy it is tested with various detector / descriptor combinations to see which ones perform best. The following steps outline the order in which this preceding goal is acheived.

* To optimize memory usage a ring buffer is utilized for loading images. 
* Keypoint detectors (e.g. HARRIS, FAST, BRISK and SIFT) are next implemented and compared with regard to number of keypoints detected and speed. 
* Descriptor extraction and matching using brute force, FLANN is used to identify corresponding keypoints in the subsequent frame. 
* All feasible combinations of detector/discriptor are tested and compared to select best performers. 


## Current local configuration
* cmake = 3.13.2
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make = 4.1 (Linux - Ubuntu 18.04.4 LTS)
  * Linux: make is installed by default on most Linux distros
* OpenCV = 4.3.0 (N.B. cv::SIFT::create() to implement this detector type)
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.3.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.3.0)
* gcc/g++ = 7.5.0
  * Linux: gcc / g++ is installed by default on most Linux distros
  

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.


[//]: # (Image References)

[image1]: /images/Shi-Tomasi-neighborhood.png "Shi-Tomasi detector matches"
[image2]: /images/Harris-neighborhood.png "Harris detector matches"
[image3]: /images/FAST-neighborhood.png "FAST detector matches"
[image4]: /images/BRISK-neighborhood.png "BRISK detector matches"
[image5]: /images/AKAZE-neighborhood.png "AKAZE detector matches"
[image6]: /images/SIFT-neighborhood.png "SIFT detector matches"
[image7]: /images/ORB-neighborhood.png "ORB detector matches"


## Implementing the code

## 1. Data Buffer Optimization
A memory buffer was used to reduce to 2 the maximum number of images stored in memory for processing. Code was impemented at `line # 76 in MidTermProject_Camera_student.cpp`
 

```
        DataFrame frame;
        
        if (dataBuffer.size() == dataBufferSize)
        {
            dataBuffer.erase(dataBuffer.begin());
        }   
        
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame); 
```

## 2. Keypoints
Keypoints are salient points or points on interest in an image that are used to track an object in the image through subsequent frames.
### 2a. Keypoint Detection
This is the process of detecting points of interest in an image where there are noticeable changes in features of the image.

### Detectected Keypoints
The keypoints shown in the table are those that the ego car's camera detected on the preceding car directly in front of it.

|Detector Type    |Img1|Img2|Img3|Img4|Img5|Img6|Img7|Img8|Img9|      
|:----------------|:---:|:---:|:---:|:---:|:---:|:---:|:----:|:----:|:---:|
|Shi-Tomasi   |118 |123 |120 | 120|113  | 114 | 123  |  111 |  112  | 
|Harris       |  14  |  18  |   21 |    26 |   43 |   18 |   31 |  26 |   34 |
|FAST          |  152 |  150 |   155 |   149 |   149  |  156 |  150 |   138  |  143|
|BRISK         |  282  | 282  |  277  |  297  |  279 |   289  | 272 |   266 |   254|
|ORB           |  102  | 106  |  113  |  109  |  125  |  130 |  129  |  127  |  128|
|AKAZE         |  157  | 161  |  155  |  163   | 164  |  173  | 175  |  177  |  179|
|SIFT          |  132 |  124  |  137  |  134  |  140  |  137 |  148  |  159  |  137|

   ##### Table 1 - The number of keypoints detected per detector type  

The Harris and Shi-Tomasi detectors have fixed block sizes of 2 and 4 respectively. For this implementation of the detectors no overlap was permitted between features and there was no variation in their neighborhood sizes which were determined by size of each block (2x2),(4x4). The FAST detector also had a fixed size neighborhood with some overlap.

The BRISK, ORB, AKAZE and SIFT detectors had varied neghborhood sizes, BRISK and ORB had small to very large neighborhood sizes appearing as clusters at times, while AKAZE and SIFT had small to medium neighborhood sizes. All four detectors had a fairly even distribution of the various size neighborhoods.

![alt text][image1]     ![alt text][image2]     ![alt text][image3]   ![alt text][image4]   ![alt text][image5]
![alt text][image6]     ![alt text][image7]


### 2b. Keypoint Removal
Only keypoints on the car directly in front of the camera fall within the region of interest (roi) and so only this subset of keypoints are taken from the captured image and the others discarded. The OpenCV `cv::Rect` function and its `contains` method is used to achieve this task.

```
        vector<cv::KeyPoint> newkeyPoints;
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            for(auto roiKeypoint: keypoints)
            {
                if (vehicleRect.contains(roiKeypoint.pt))
                {
                    newkeyPoints.push_back(roiKeypoint); 
                }
            } 
            keypoints = newkeyPoints;
        }
```        

## 3. Descriptors
They are used to uniquely identify the salient points detected in the camera image.

### 3a. Keypoint Descriptors
The descriptors with their varied call functions are found in the OpenCV library. This was implemented at `line 60 of matching2D_Student.cpp code`

```
// select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling 
                                   // the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::SIFT::create();
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else extractor = cv::xfeatures2d::FREAK::create();
```

### 3b. Discriptor Matching
Selecting FLANN speeds up the nearest neighbor search by incorporating a KD-Tree reduces search time. This was implemented at`line 19 of matching2D_Student.cpp code`

```
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        //cout << "FLANN matching";
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        vector<vector<cv::DMatch>> knn_matches;
        matcher->knnMatch(descSource, descRef, knn_matches, 2); // finds the 2 best matches
        //cout << " (KNN) with n=" << knn_matches.size() << endl;
```


### 3c. Discriptor Distance Ratio
A distance ratio of 0.8 was used to select suitable descriptor matches. Any descriptor pair distance ratio less than this value was considered a match. This was implemented at `line # 36 in matching2D_Student.cpp` code:

```
        // Filter matches using descriptor distance ratio test
        double minDescDistRatio = 0.8;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {
            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }
        cout << "# keypoints removed = " << knn_matches.size() - matches.size() << endl;
 ```



### 4a. Performance Evaluation I
The number of keypoints on the preceding vehicle for all detectors are shown below:

| Detector Type|Descriptor Type|		Img1|	Img2|	Img3|	Img4|	Img5|	Img6|	Img7|	Img8|	Img9 |
|:------------|:---------------|:------:|:-----:|:----:|:-----:|:-----:|:----:|:-----:|:-----:|:-----:|
| SHITOMASI	|BRISK	        	|95	|88	|80	|90	|82	|79	|85	|86	|82                       |
| HARRIS|	BRISK	              |  12	|10	|14	|15	|16	|16	|15	|23|	21                   |
| FAST	 |       BRISK		       | 97	|104	|101|	98|	85|	107|	107|	100|	100              |
| BRISK	  |      BRISK		       | 171|	176	|157|	176	|174|	188	|173|	171	|184          |
| ORB	   |     BRISK		        |73	|74	|79	|85	|79	|92	|90	|88	|91                     |
| AKAZE	  |      BRISK		        |137	|125	|129	|129	|131	|132	|142	|146	|144          |
| SIFT	 |       BRISK		        |64	|66	|62|	66|	59|	64|	64|	67|	80                    |
|                                                                           |
|   |
| SHITOMASI	|BRIEF	        	|115|	111|	104	|101	|102	|102|	100|	109|	100              |
| HARRIS	|BRIEF	               | 14	|11	|15	|20|	24|	26|	16|	24|	23                   |
| FAST|	        BRIEF   		|119	|130|	118|	126|	108|	123|	131|	125	|119                |
| BRISK|	BRIEF		|178|	205|	185|	179	|183|	195	|207|	189|	183 |
| ORB	|BRIEF		|49	|43	|45|	59|	53|	78|	68|	84|	66            |
| AKAZE|	BRIEF	|	141	|134|	131|	130|	134	|146	|150|	148	|152 |
| SIFT|	BRIEF	|	86|	78|	76|	85|	69|	74|	76	|70|	88           |
| |
| |
| SHITOMASI	|ORB	|	104	|103|	100	|102|	103|	98|	98|	102|	97  |
| HARRIS|	ORB		|12	|13	|16	|18	|24	|18	|15	|24	|20           |
| FAST|	ORB	|	122	|122|	115	|129|	107|	120	|126|	122|	118    |
| BRISK	|ORB|		160|	171	|157|	170|	154|	180	|171|	175	|172   |
| ORB|	ORB	|	65|	69|	71|	85|	91|	101|	95|	93|	91             |
| AKAZE|	ORB	|	130	|129|	128	|115	|132	|132|	137	|137|	146   |
|              |
| |
| SHITOMASI|	FREAK	|	90|	88|	87|	89|	83|	78|	81|	86|	84      |
| HARRIS|	FREAK	|	13|	13|	15|	15|	17|	20|	14	|21|	18         |
| FAST|	FREAK|		97|	98|	94|	99|	88|	99|	104|	99|	103         |
| BRISK|	FREAK	|	160	|178|	156	|173	|160	|183	|169	|179	|168 |
| ORB	|FREAK		|42|	36|	45|	47|	44	|51|	52|	49|	55            |
| AKAZE|	FREAK	|	126	|128|	128|	121|	123|	132|	145|	148|	137 |
| SIFT|	FREAK	|	64	|	65|	66|	63|	58|	64|	65|	79           |
|   |
| |
| AKAZE	|AKAZE	|	138|	138|	133|	127	|129	|146|	147	|151|	150 |
|   |
| |
| SHITOMASI	|SIFT|		112	|109|	104	|103|	99|	101|	96|	106|	97 |
| HARRIS|	SIFT	|	14|	11|	16|	19|	22	|22|	13|	24|	22          |
| FAST|	SIFT	|	118|	123|	110|	119|	114|	119|	123	|117|	103   |
| BRISK	|SIFT	|	182|	193	|169|	183|	171|	195|	194	|176|	183  |
| ORB	|SIFT		|67|	79|	78|	79|	82|	95|	95|	94|	94             |
| AKAZE	|SIFT	|	134	|134|	130|	136|	137|	147	|147	|154	|151  |
| SIFT	|SIFT	|	82|	81|	85|	93|	90|	81|	82|	102	|104          |
|                                                  |



### 4b. Performance Evaluation II & III
The number of keypoints detected per combined detector and descriptor and the total time for keypoint detection and descriptor extraction are shown below.

Based on the data shown below the top 3 detector-descriptor combination for keypoint detection on cars are:

|#|Detector Type |Descriptor Type| Total Time (ms)|
|:--:|:--------:|:---------:|:-----:|
|1|FAST| BRIEF| 0.00243|
|2|FAST|BRISK |0.00343 |
|3|FAST    |ORB  | 0.00688 |
|                          |
|                          |


### Table of Performance values


|detectorType|descriptorType  |imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|FIELD9|Accuracy|
|------------|----------------|----------------|------------|--------------|--------------------|----------------|---------|------|--------|
|SHITOMASI   |BRISK           |1               |0.0224168   |0.00215261    |118                 |95              |0.0245694|      |0.805   |
|            |                |2               |0.0168469   |0.00181144    |123                 |88              |0.0186583|      |0.715   |
|            |                |3               |0.0130094   |0.0023408     |120                 |80              |0.0153502|      |0.667   |
|            |                |4               |0.0168009   |0.00217147    |120                 |90              |0.0189724|      |0.75    |
|            |                |5               |0.0134018   |0.00168542    |113                 |82              |0.0150872|      |0.726   |
|            |                |6               |0.0132079   |0.00245311    |114                 |79              |0.015661 |      |0.693   |
|            |                |7               |0.0146563   |0.00219153    |123                 |85              |0.0168478|      |0.691   |
|            |                |8               |0.0142881   |0.00219574    |111                 |86              |0.0164838|      |0.775   |
|            |                |9               |0.01506     |0.00220022    |112                 |82              |0.0172602|      |0.732   |
|            |                |                |            |              |                    |                |         |      |        |
|            | |                |Mean Accuracy =            |0.728         |Mean Total Time     |                |0.01765  |      |        |
|            |                |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType  |imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|SHITOMASI   |BRIEF           |1               |0.0181884   |0.00202525    |118                 |115             |0.0202137|      |0.975   |
|            |                |2               |0.0143026   |0.000805035   |123                 |111             |0.0151076|      |0.902   |
|            |                |3               |0.0150841   |0.0016488     |120                 |104             |0.0167329|      |0.867   |
|            |                |4               |0.0140848   |0.00134888    |120                 |101             |0.0154337|      |0.842   |
|            |                |5               |0.0141072   |0.000780503   |113                 |102             |0.0148877|      |0.903   |
|            |                |6               |0.0155014   |0.00132965    |114                 |102             |0.016831 |      |0.895   |
|            |                |7               |0.0144711   |0.00138082    |123                 |100             |0.0158519|      |0.813   |
|            |                |8               |0.0135415   |0.000768027   |111                 |109             |0.0143095|      |0.982   |
|            |                |9               |0.014017    |0.00148308    |112                 |100             |0.0155001|      |0.893   |
|            |                |                |            |              |                    |                |         |      |        |
|            |                |Mean Accuracy = |            |0.897         |                    |Mean Total Time |0.0161   |      |        |
|            |                |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType  |imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|SHITOMASI   |ORB             |1               |0.0167312   |0.00506559    |118                 |104             |0.0217968|      |0.881   |
|            |                |2               |0.0139072   |0.00525136    |123                 |103             |0.0191585|      |0.837   |
|            |                |3               |0.0141509   |0.00502366    |120                 |100             |0.0191746|      |0.833   |
|            |                |4               |0.0146905   |0.00462269    |120                 |102             |0.0193132|      |0.85    |
|            |                |5               |0.0148035   |0.00540691    |113                 |103             |0.0202104|      |0.912   |
|            |                |6               |0.0133077   |0.00485111    |114                 |98              |0.0181588|      |0.86    |
|            |                |7               |0.0140899   |0.0054414     |123                 |98              |0.0195313|      |0.797   |
|            |                |8               |0.0166603   |0.00587059    |111                 |102             |0.0225309|      |0.919   |
|            |                |9               |0.017112    |0.00531995    |112                 |97              |0.022432 |      |0.866   |
|            |                |                |            |              |                    |                |         |      |        |
|            |                |Mean Accuracy = |            |0.862         |Mean Total Time     |                |0.02026  |      |        |
|            |                |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType  |imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|SHITOMASI   |FREAK           |1               |0.0142728   |0.0461494     |118                 |90              |0.0604221|      |0.763   |
|            |                |2               |0.0135621   |0.0461391     |123                 |88              |0.0597012|      |0.715   |
|            |                |3               |0.0134626   |0.0457085     |120                 |87              |0.0591711|      |0.725   |
|            |                |4               |0.0126511   |0.0467311     |120                 |89              |0.0593822|      |0.742   |
|            |                |5               |0.0132597   |0.0463861     |113                 |83              |0.0596458|      |0.735   |
|            |                |6               |0.0161977   |0.0454615     |114                 |78              |0.0616592|      |0.684   |
|            |                |7               |0.0128044   |0.0466571     |123                 |81              |0.0594615|      |0.659   |
|            |                |8               |0.0125877   |0.0458761     |111                 |86              |0.0584638|      |0.775   |
|            |                |9               |0.0132815   |0.0457056     |112                 |84              |0.0589871|      |0.75    |
|            |                |                |            |              |                    |                |         |      |        |
|            |                |Mean Accuracy = |            |0.728         |Mean Total Time     |                |0.05965  |      |        |
|            |                |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType  |imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|SHITOMASI   |SIFT            |1               |0.0140511   |0.0230614     |118                 |112             |0.0371125|      |0.949   |
|            |                |2               |0.0144733   |0.019141      |123                 |109             |0.0336143|      |0.886   |
|            |                |3               |0.0166172   |0.0232842     |120                 |104             |0.0399014|      |0.867   |
|            |                |4               |0.0167802   |0.0245681     |120                 |103             |0.0413483|      |0.858   |
|            |                |5               |0.0149423   |0.020907      |113                 |99              |0.0358493|      |0.876   |
|            |                |6               |0.0140722   |0.0200615     |114                 |101             |0.0341337|      |0.886   |
|            |                |7               |0.0143321   |0.0210561     |123                 |96              |0.0353882|      |0.78    |
|            |                |8               |0.0140353   |0.02111       |111                 |106             |0.0351452|      |0.955   |
|            |                |9               |0.0142021   |0.0313785     |112                 |97              |0.0455806|      |0.866   |
|            |                |                |            |              |                    |                |         |      |        |
|            |                |Mean Accuracy = |            |0.88          |Mean Total Time     |                |0.03756  |      |        |




|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|FIELD9|Accuracy|
|------------|--------------|----------------|------------|--------------|--------------------|----------------|---------|------|--------|
|HARRIS      |BRISK         |1               |0.014907    |0.000986707   |14                  |12              |0.0158937|      |0.857   |
|            |              |2               |0.0148228   |0.00100041    |18                  |10              |0.0158232|      |0.556   |
|            |              |3               |0.0138173   |0.00054348    |21                  |14              |0.0143608|      |0.667   |
|            |              |4               |0.0129658   |0.00111061    |26                  |15              |0.0140764|      |0.577   |
|            |              |5               |0.028621    |0.00126787    |43                  |16              |0.0298888|      |0.372   |
|            |              |6               |0.0134113   |0.000508852   |18                  |16              |0.0139201|      |0.889   |
|            |              |7               |0.014814    |0.00114069    |31                  |15              |0.0159547|      |0.484   |
|            |              |8               |0.0150991   |0.00109961    |26                  |23              |0.0161987|      |0.885   |
|            |              |9               |0.0196809   |0.00070172    |34                  |21              |0.0203826|      |0.618   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.656         |                    |Mean Total Time |0.01739  |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|HARRIS      |BRIEF         |1               |0.0152381   |0.000934187   |14                  |14              |0.0161723|      |1       |
|            |              |2               |0.0149763   |0.000991677   |18                  |11              |0.015968 |      |0.611   |
|            |              |3               |0.0138466   |0.000419652   |21                  |15              |0.0142663|      |0.714   |
|            |              |4               |0.0141557   |0.000948314   |26                  |20              |0.015104 |      |0.769   |
|            |              |5               |0.0275744   |0.000503753   |43                  |24              |0.0280782|      |0.558   |
|            |              |6               |0.0130952   |0.000944405   |18                  |26              |0.0140396|      |1       |
|            |              |7               |0.0159128   |0.0009818     |31                  |16              |0.0168946|      |0.516   |
|            |              |8               |0.0148749   |0.000956582   |26                  |24              |0.0158315|      |0.923   |
|            |              |9               |0.0219088   |0.00101183    |34                  |23              |0.0229206|      |0.676   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.752         |                    |Mean Total Time |0.0177   |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|HARRIS      |ORB           |1               |0.0151902   |0.00512612    |14                  |12              |0.0203164|      |0.857   |
|            |              |2               |0.0155803   |0.00459101    |18                  |13              |0.0201713|      |0.722   |
|            |              |3               |0.0143      |0.00463897    |21                  |16              |0.018939 |      |0.762   |
|            |              |4               |0.0153324   |0.00476448    |26                  |18              |0.0200969|      |0.692   |
|            |              |5               |0.0335734   |0.0049081     |43                  |24              |0.0384815|      |0.558   |
|            |              |6               |0.0140824   |0.00510246    |18                  |18              |0.0191848|      |1       |
|            |              |7               |0.0168741   |0.00475623    |31                  |15              |0.0216304|      |0.484   |
|            |              |8               |0.0171453   |0.00486388    |26                  |24              |0.0220091|      |0.923   |
|            |              |9               |0.0218378   |0.00683484    |34                  |20              |0.0286727|      |0.588   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.732         |                    |Mean Total Time |0.02328  |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|HARRIS      |FREAK         |1               |0.0176012   |0.0480989     |14                  |13              |0.0657001|      |0.929   |
|            |              |2               |0.0149884   |0.0462983     |18                  |13              |0.0612867|      |0.722   |
|            |              |3               |0.0128322   |0.0466104     |21                  |15              |0.0594426|      |0.714   |
|            |              |4               |0.014633    |0.0477623     |26                  |15              |0.0623953|      |0.577   |
|            |              |5               |0.0308123   |0.047868      |43                  |17              |0.0786803|      |0.395   |
|            |              |6               |0.0123642   |0.0446106     |18                  |20              |0.0569748|      |1       |
|            |              |7               |0.0150187   |0.0442651     |31                  |14              |0.0592838|      |0.452   |
|            |              |8               |0.0137416   |0.0438383     |26                  |21              |0.0575799|      |0.808   |
|            |              |9               |0.0180822   |0.0452941     |34                  |18              |0.0633763|      |0.529   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.681         |                    |Mean Total Time |0.06275  |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|HARRIS      |SIFT          |1               |0.0124609   |0.0205027     |14                  |14              |0.0329636|      |1       |
|            |              |2               |0.014627    |0.0196271     |18                  |11              |0.0342542|      |0.611   |
|            |              |3               |0.0142316   |0.0220368     |21                  |16              |0.0362684|      |0.762   |
|            |              |4               |0.0174767   |0.0168372     |26                  |19              |0.0343138|      |0.731   |
|            |              |5               |0.0290441   |0.0208186     |43                  |22              |0.0498627|      |0.512   |
|            |              |6               |0.0143956   |0.0200992     |18                  |22              |0.0344947|      |1       |
|            |              |7               |0.0159457   |0.0155562     |31                  |13              |0.0315019|      |0.419   |
|            |              |8               |0.0148028   |0.0204427     |26                  |24              |0.0352456|      |0.923   |
|            |              |9               |0.0195651   |0.0204137     |34                  |22              |0.0399788|      |0.647   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.734         |                    |Mean Total Time |0.03654  |      |        |





|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime |FIELD9|Accuracy|
|------------|--------------|----------------|------------|--------------|--------------------|----------------|----------|------|--------|
|FAST        |BRISK         |1               |0.00133669  |0.00254868    |152                 |97              |0.00388536|      |0.638   |
|            |              |2               |0.0012576   |0.00200546    |150                 |104             |0.00326306|      |0.693   |
|            |              |3               |0.00129208  |0.00199948    |155                 |101             |0.00329156|      |0.652   |
|            |              |4               |0.00242645  |0.00190298    |149                 |98              |0.00432943|      |0.658   |
|            |              |5               |0.00127637  |0.00198644    |149                 |85              |0.00326281|      |0.57    |
|            |              |6               |0.00127866  |0.00207772    |156                 |107             |0.00335638|      |0.686   |
|            |              |7               |0.00123059  |0.00192073    |150                 |107             |0.00315133|      |0.713   |
|            |              |8               |0.0012474   |0.0017916     |138                 |100             |0.003039  |      |0.725   |
|            |              |9               |0.00137834  |0.00192485    |143                 |100             |0.0033032 |      |0.699   |
|            |              |                |            |              |                    |                |          |      |        |
|            |              |Mean Accuracy = |            |0.67          |                    |Mean Total Time |0.00343   |      |        |
|            |              |                |            |              |                    |                |          |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime |      |Accuracy|
|FAST        |BRIEF         |1               |0.00131406  |0.00154192    |152                 |119             |0.00285598|      |0.783   |
|            |              |2               |0.00127478  |0.00090807    |150                 |130             |0.00218285|      |0.867   |
|            |              |3               |0.00212624  |0.000924713   |155                 |118             |0.00305095|      |0.761   |
|            |              |4               |0.00134546  |0.00131497    |149                 |126             |0.00266043|      |0.846   |
|            |              |5               |0.00122397  |0.000946808   |149                 |108             |0.00217078|      |0.725   |
|            |              |6               |0.00127115  |0.000932195   |156                 |123             |0.00220335|      |0.788   |
|            |              |7               |0.00122161  |0.000953412   |150                 |131             |0.00217503|      |0.873   |
|            |              |8               |0.00156554  |0.000860313   |138                 |125             |0.00242585|      |0.906   |
|            |              |9               |0.00126158  |0.000904185   |143                 |119             |0.00216576|      |0.832   |
|            |              |                |            |              |                    |                |          |      |        |
|            |              |Mean Accuracy = |            |0.82          |                    |Mean Total Time |0.00243   |      |        |
|            |              |                |            |              |                    |                |          |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime |      |Accuracy|
|FAST        |ORB           |1               |0.00136523  |0.00512043    |152                 |122             |0.00648565|      |0.803   |
|            |              |2               |0.0012248   |0.0049245     |150                 |122             |0.00614929|      |0.813   |
|            |              |3               |0.00126497  |0.00534432    |155                 |115             |0.00660929|      |0.742   |
|            |              |4               |0.00232202  |0.00726233    |149                 |129             |0.00958434|      |0.866   |
|            |              |5               |0.00126471  |0.00560654    |149                 |107             |0.00687125|      |0.718   |
|            |              |6               |0.00125987  |0.00575942    |156                 |120             |0.00701929|      |0.769   |
|            |              |7               |0.00123293  |0.00530131    |150                 |126             |0.00653424|      |0.84    |
|            |              |8               |0.00131145  |0.00504944    |138                 |122             |0.00636089|      |0.884   |
|            |              |9               |0.00124782  |0.00502553    |143                 |118             |0.00627335|      |0.825   |
|            |              |                |            |              |                    |                |          |      |        |
|            |              |Mean Accuracy = |            |0.807         |                    |Mean Total Time |0.00688   |      |        |
|            |              |                |            |              |                    |                |          |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime |      |Accuracy|
|FAST        |FREAK         |1               |0.00171743  |0.0482883     |152                 |97              |0.0500057 |      |0.638   |
|            |              |2               |0.0016603   |0.045537      |150                 |98              |0.0471973 |      |0.653   |
|            |              |3               |0.0016204   |0.0444264     |155                 |94              |0.0460468 |      |0.606   |
|            |              |4               |0.00165422  |0.0451091     |149                 |99              |0.0467633 |      |0.664   |
|            |              |5               |0.00166375  |0.045047      |149                 |88              |0.0467108 |      |0.591   |
|            |              |6               |0.00302418  |0.0450727     |156                 |99              |0.0480969 |      |0.635   |
|            |              |7               |0.00163863  |0.0445422     |150                 |104             |0.0461808 |      |0.693   |
|            |              |8               |0.00164751  |0.0450761     |138                 |99              |0.0467236 |      |0.717   |
|            |              |9               |0.00168788  |0.0445041     |143                 |103             |0.046192  |      |0.72    |
|            |              |                |            |              |                    |                |          |      |        |
|            |              |Mean Accuracy = |            |0.657         |                    |Mean Total Time |0.0471    |      |        |
|            |              |                |            |              |                    |                |          |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime |      |Accuracy|
|FAST        |SIFT          |1               |0.00132001  |0.0217091     |152                 |118             |0.0230291 |      |0.776   |
|            |              |2               |0.00203512  |0.02068       |150                 |123             |0.0227151 |      |0.82    |
|            |              |3               |0.00122668  |0.0216792     |155                 |110             |0.0229059 |      |0.71    |
|            |              |4               |0.0012512   |0.0204532     |149                 |119             |0.0217044 |      |0.799   |
|            |              |5               |0.00123229  |0.0205375     |149                 |114             |0.0217698 |      |0.765   |
|            |              |6               |0.00123205  |0.0215614     |156                 |119             |0.0227935 |      |0.763   |
|            |              |7               |0.00122623  |0.0220944     |150                 |123             |0.0233206 |      |0.82    |
|            |              |8               |0.00121731  |0.0202847     |138                 |117             |0.0215021 |      |0.848   |
|            |              |9               |0.00126617  |0.0216879     |143                 |103             |0.022954  |      |0.72    |
|            |              |                |            |              |                    |                |          |      |        |
|            |              |Mean Accuracy = |            |0.78          |                    |Mean Total Time |0.02252   |      |        |



|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|FIELD9|Accuracy|
|------------|--------------|----------------|------------|--------------|--------------------|----------------|---------|------|--------|
|BRISK       |BRISK         |1               |0.0424119   |0.00336808    |282                 |171             |0.04578  |      |0.606   |
|            |              |2               |0.0412748   |0.00352847    |282                 |176             |0.0448033|      |0.624   |
|            |              |3               |0.0417855   |0.00331514    |277                 |157             |0.0451006|      |0.567   |
|            |              |4               |0.0415284   |0.00353659    |297                 |176             |0.045065 |      |0.593   |
|            |              |5               |0.040524    |0.00334382    |279                 |174             |0.0438678|      |0.624   |
|            |              |6               |0.0405396   |0.00346224    |289                 |188             |0.0440019|      |0.651   |
|            |              |7               |0.040858    |0.00326067    |272                 |173             |0.0441187|      |0.636   |
|            |              |8               |0.0401548   |0.00321501    |266                 |171             |0.0433698|      |0.643   |
|            |              |9               |0.0415176   |0.00307924    |254                 |184             |0.0445969|      |0.724   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.63          |                    |Mean Total Time |0.04452  |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|BRISK       |BRIEF         |1               |0.0423905   |0.0014139     |282                 |178             |0.0438044|      |0.631   |
|            |              |2               |0.0413654   |0.00142728    |282                 |205             |0.0427927|      |0.727   |
|            |              |3               |0.0416821   |0.00143494    |277                 |185             |0.0431171|      |0.668   |
|            |              |4               |0.0417921   |0.0014909     |297                 |179             |0.043283 |      |0.603   |
|            |              |5               |0.04102     |0.00142243    |279                 |183             |0.0424424|      |0.656   |
|            |              |6               |0.0412064   |0.00146898    |289                 |195             |0.0426754|      |0.675   |
|            |              |7               |0.0399668   |0.00133525    |272                 |207             |0.041302 |      |0.761   |
|            |              |8               |0.0414587   |0.00138478    |266                 |189             |0.0428435|      |0.711   |
|            |              |9               |0.0413622   |0.00131325    |254                 |183             |0.0426755|      |0.72    |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.684         |                    |Mean Total Time |0.04277  |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|BRISK       |ORB           |1               |0.0441996   |0.0162916     |282                 |160             |0.0604911|      |0.567   |
|            |              |2               |0.0408597   |0.0168625     |282                 |171             |0.0577222|      |0.606   |
|            |              |3               |0.0414157   |0.0166863     |277                 |157             |0.0581021|      |0.567   |
|            |              |4               |0.041902    |0.0159169     |297                 |170             |0.0578189|      |0.572   |
|            |              |5               |0.0412199   |0.016766      |279                 |154             |0.0579859|      |0.552   |
|            |              |6               |0.0406907   |0.0165194     |289                 |180             |0.0572101|      |0.623   |
|            |              |7               |0.0402497   |0.0159216     |272                 |171             |0.0561713|      |0.629   |
|            |              |8               |0.0416459   |0.0167055     |266                 |175             |0.0583514|      |0.658   |
|            |              |9               |0.0416842   |0.0164417     |254                 |172             |0.0581259|      |0.677   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.606         |                    |Mean Total Time |0.058    |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|BRISK       |FREAK         |1               |0.0437394   |0.0497298     |282                 |160             |0.0934691|      |0.567   |
|            |              |2               |0.0415845   |0.0471628     |282                 |178             |0.0887472|      |0.631   |
|            |              |3               |0.0410546   |0.0460274     |277                 |156             |0.0870819|      |0.563   |
|            |              |4               |0.0408031   |0.0457169     |297                 |173             |0.08652  |      |0.582   |
|            |              |5               |0.0410033   |0.0459835     |279                 |160             |0.0869868|      |0.573   |
|            |              |6               |0.040702    |0.0460331     |289                 |183             |0.0867351|      |0.633   |
|            |              |7               |0.0412816   |0.0457102     |272                 |169             |0.0869918|      |0.621   |
|            |              |8               |0.0403237   |0.0459492     |266                 |179             |0.0862729|      |0.673   |
|            |              |9               |0.041506    |0.0453373     |254                 |168             |0.0868433|      |0.661   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.612         |                    |Mean Total Time |0.08774  |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|BRISK       |SIFT          |1               |0.0447193   |0.0384118     |282                 |182             |0.0831311|      |0.645   |
|            |              |2               |0.0408706   |0.0357175     |282                 |193             |0.0765881|      |0.684   |
|            |              |3               |0.0411159   |0.0345928     |277                 |169             |0.0757087|      |0.61    |
|            |              |4               |0.0408921   |0.0353959     |297                 |183             |0.076288 |      |0.616   |
|            |              |5               |0.0442493   |0.0354835     |279                 |171             |0.0797328|      |0.613   |
|            |              |6               |0.0411885   |0.0350969     |289                 |195             |0.0762854|      |0.675   |
|            |              |7               |0.0401721   |0.0342602     |272                 |194             |0.0744322|      |0.713   |
|            |              |8               |0.0403453   |0.0365829     |266                 |176             |0.0769281|      |0.662   |
|            |              |9               |0.0406822   |0.0351914     |254                 |183             |0.0758737|      |0.72    |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.66          |                    |Mean Total Time |0.07722  |      |        |



|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime |FIELD9|Accuracy|
|------------|--------------|----------------|------------|--------------|--------------------|----------------|----------|------|--------|
|ORB         |BRISK         |1               |0.010363    |0.00142238    |102                 |73              |0.0117854 |      |0.716   |
|            |              |2               |0.00899295  |0.00136505    |106                 |74              |0.010358  |      |0.698   |
|            |              |3               |0.00899972  |0.00156368    |113                 |79              |0.0105634 |      |0.699   |
|            |              |4               |0.0101907   |0.00146409    |109                 |85              |0.0116548 |      |0.78    |
|            |              |5               |0.00906113  |0.00160197    |125                 |79              |0.0106631 |      |0.632   |
|            |              |6               |0.00926957  |0.00166411    |130                 |92              |0.0109337 |      |0.708   |
|            |              |7               |0.0100821   |0.00164042    |129                 |90              |0.0117225 |      |0.698   |
|            |              |8               |0.00913874  |0.00166151    |127                 |88              |0.0108002 |      |0.693   |
|            |              |9               |0.00901811  |0.00168509    |128                 |91              |0.0107032 |      |0.711   |
|            |              |                |            |              |                    |                |          |      |        |
|            |              |Mean Accuracy = |            |0.704         |                    |Mean Total Time |0.01102   |      |        |
|            |              |                |            |              |                    |                |          |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime |      |Accuracy|
|ORB         |BRIEF         |1               |0.00946931  |0.000732872   |102                 |49              |0.0102022 |      |0.48    |
|            |              |2               |0.00906891  |0.000743017   |106                 |43              |0.00981193|      |0.406   |
|            |              |3               |0.0087506   |0.000778413   |113                 |45              |0.00952902|      |0.398   |
|            |              |4               |0.00983921  |0.000750346   |109                 |59              |0.0105896 |      |0.541   |
|            |              |5               |0.0086842   |0.000834745   |125                 |53              |0.00951894|      |0.424   |
|            |              |6               |0.00878746  |0.000840901   |130                 |78              |0.00962836|      |0.6     |
|            |              |7               |0.00912937  |0.000828127   |129                 |68              |0.00995749|      |0.527   |
|            |              |8               |0.00906722  |0.000830598   |127                 |84              |0.00989782|      |0.661   |
|            |              |9               |0.00912862  |0.000834394   |128                 |66              |0.00996301|      |0.516   |
|            |              |                |            |              |                    |                |          |      |        |
|            |              |Mean Accuracy = |            |0.506         |                    |Mean Total Time |0.0099    |      |        |
|            |              |                |            |              |                    |                |          |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime |      |Accuracy|
|ORB         |ORB           |1               |0.00914669  |0.0176144     |102                 |65              |0.0267611 |      |0.637   |
|            |              |2               |0.00910478  |0.0179881     |106                 |69              |0.0270929 |      |0.651   |
|            |              |3               |0.00899284  |0.01743       |113                 |71              |0.0264228 |      |0.628   |
|            |              |4               |0.00907468  |0.0172634     |109                 |85              |0.026338  |      |0.78    |
|            |              |5               |0.0092217   |0.0174582     |125                 |91              |0.0266799 |      |0.728   |
|            |              |6               |0.00945725  |0.0174466     |130                 |101             |0.0269039 |      |0.777   |
|            |              |7               |0.00885835  |0.0168299     |129                 |95              |0.0256883 |      |0.736   |
|            |              |8               |0.00914966  |0.0174018     |127                 |93              |0.0265515 |      |0.732   |
|            |              |9               |0.00935976  |0.0170519     |128                 |91              |0.0264117 |      |0.711   |
|            |              |                |            |              |                    |                |          |      |        |
|            |              |Mean Accuracy = |            |0.709         |                    |Mean Total Time |0.02654   |      |        |
|            |              |                |            |              |                    |                |          |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime |      |Accuracy|
|ORB         |FREAK         |1               |0.00920163  |0.0472803     |102                 |42              |0.056482  |      |0.412   |
|            |              |2               |0.00926788  |0.0453204     |106                 |36              |0.0545882 |      |0.34    |
|            |              |3               |0.00888072  |0.0440721     |113                 |45              |0.0529528 |      |0.398   |
|            |              |4               |0.00896273  |0.0439287     |109                 |47              |0.0528914 |      |0.431   |
|            |              |5               |0.00900881  |0.0434145     |125                 |44              |0.0524233 |      |0.352   |
|            |              |6               |0.00984358  |0.0433129     |130                 |51              |0.0531565 |      |0.392   |
|            |              |7               |0.00909343  |0.0438254     |129                 |52              |0.0529188 |      |0.403   |
|            |              |8               |0.00906374  |0.0443277     |127                 |49              |0.0533914 |      |0.386   |
|            |              |9               |0.0102166   |0.0431284     |128                 |55              |0.053345  |      |0.43    |
|            |              |                |            |              |                    |                |          |      |        |
|            |              |Mean Accuracy = |            |0.394         |                    |Mean Total Time |0.05357   |      |        |
|            |              |                |            |              |                    |                |          |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime |      |Accuracy|
|ORB         |SIFT          |1               |0.00925456  |0.0388165     |102                 |67              |0.048071  |      |0.657   |
|            |              |2               |0.00891516  |0.0395769     |106                 |79              |0.048492  |      |0.745   |
|            |              |3               |0.00919594  |0.0395787     |113                 |78              |0.0487746 |      |0.69    |
|            |              |4               |0.00883629  |0.0373768     |109                 |79              |0.0462131 |      |0.725   |
|            |              |5               |0.00893024  |0.0384231     |125                 |82              |0.0473534 |      |0.656   |
|            |              |6               |0.00898886  |0.0407261     |130                 |95              |0.0497149 |      |0.731   |
|            |              |7               |0.00895255  |0.0442379     |129                 |95              |0.0531904 |      |0.736   |
|            |              |8               |0.00906007  |0.0445837     |127                 |94              |0.0536438 |      |0.74    |
|            |              |9               |0.00928015  |0.0411822     |128                 |94              |0.0504624 |      |0.734   |
|            |              |                |            |              |                    |                |          |      |        |
|            |              |Mean Accuracy = |            |0.713         |                    |Mean Total Time |0.04955   |      |        |




|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|FIELD9|Accuracy|
|------------|--------------|----------------|------------|--------------|--------------------|----------------|---------|------|--------|
|AKAZE       |BRISK         |1               |0.0713499   |0.00208506    |157                 |137             |0.0734349|      |0.873   |
|            |              |2               |0.0640103   |0.00207021    |161                 |125             |0.0660806|      |0.776   |
|            |              |3               |0.0654115   |0.0019681     |155                 |129             |0.0673796|      |0.832   |
|            |              |4               |0.0650967   |0.00211597    |163                 |129             |0.0672127|      |0.791   |
|            |              |5               |0.0642516   |0.00209296    |164                 |131             |0.0663446|      |0.799   |
|            |              |6               |0.0654246   |0.00218145    |173                 |132             |0.067606 |      |0.763   |
|            |              |7               |0.0653145   |0.00217469    |175                 |142             |0.0674892|      |0.811   |
|            |              |8               |0.0681521   |0.00220152    |177                 |146             |0.0703537|      |0.825   |
|            |              |9               |0.063839    |0.00231616    |179                 |144             |0.0661551|      |0.804   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.808         |                    |Mean Total Time |0.06801  |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|AKAZE       |BRIEF         |1               |0.0701212   |0.000946189   |157                 |141             |0.0710674|      |0.898   |
|            |              |2               |0.0663128   |0.000970234   |161                 |134             |0.0672831|      |0.832   |
|            |              |3               |0.0650462   |0.000919756   |155                 |131             |0.065966 |      |0.845   |
|            |              |4               |0.0655912   |0.00123581    |163                 |130             |0.066827 |      |0.798   |
|            |              |5               |0.0617562   |0.00112531    |164                 |134             |0.0628815|      |0.817   |
|            |              |6               |0.0622434   |0.000978418   |173                 |146             |0.0632218|      |0.844   |
|            |              |7               |0.0630309   |0.000982058   |175                 |150             |0.0640129|      |0.857   |
|            |              |8               |0.067934    |0.00143622    |177                 |148             |0.0693703|      |0.836   |
|            |              |9               |0.0683751   |0.00111045    |179                 |152             |0.0694855|      |0.849   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.842         |                    |Mean Total Time |0.06668  |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|AKAZE       |ORB           |1               |0.0673824   |0.0121153     |157                 |130             |0.0794977|      |0.828   |
|            |              |2               |0.066804    |0.0113977     |161                 |129             |0.0782017|      |0.801   |
|            |              |3               |0.0641996   |0.0120217     |155                 |128             |0.0762213|      |0.826   |
|            |              |4               |0.0648812   |0.011845      |163                 |115             |0.0767262|      |0.706   |
|            |              |5               |0.0649153   |0.0117359     |164                 |132             |0.0766513|      |0.805   |
|            |              |6               |0.0680379   |0.0121053     |173                 |132             |0.0801431|      |0.763   |
|            |              |7               |0.0642816   |0.0121061     |175                 |137             |0.0763877|      |0.783   |
|            |              |8               |0.0686072   |0.0124319     |177                 |137             |0.0810391|      |0.774   |
|            |              |9               |0.0672581   |0.0119221     |179                 |146             |0.0791802|      |0.816   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.789         |                    |Mean Total Time |0.07823  |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|AKAZE       |FREAK         |1               |0.0636594   |0.0451964     |157                 |126             |0.108856 |      |0.803   |
|            |              |2               |0.069228    |0.0489775     |161                 |128             |0.118205 |      |0.795   |
|            |              |3               |0.0700143   |0.0454108     |155                 |128             |0.115425 |      |0.826   |
|            |              |4               |0.0656281   |0.045863      |163                 |121             |0.111491 |      |0.742   |
|            |              |5               |0.0661322   |0.0456744     |164                 |123             |0.111807 |      |0.75    |
|            |              |6               |0.0634402   |0.0439851     |173                 |132             |0.107425 |      |0.763   |
|            |              |7               |0.0598534   |0.0443749     |175                 |145             |0.104228 |      |0.829   |
|            |              |8               |0.0571682   |0.0445525     |177                 |148             |0.101721 |      |0.836   |
|            |              |9               |0.0573672   |0.0437836     |179                 |137             |0.101151 |      |0.765   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.79          |                    |Mean Total Time |0.10892  |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|AKAZE       |AKAZE         |1               |0.0690262   |0.0562783     |157                 |138             |0.125305 |      |0.879   |
|            |              |2               |0.0788932   |0.0579442     |161                 |138             |0.136837 |      |0.857   |
|            |              |3               |0.0653162   |0.0555136     |155                 |133             |0.12083  |      |0.858   |
|            |              |4               |0.0625988   |0.0570274     |163                 |127             |0.119626 |      |0.779   |
|            |              |5               |0.0678001   |0.0555759     |164                 |129             |0.123376 |      |0.787   |
|            |              |6               |0.065596    |0.0549893     |173                 |146             |0.120585 |      |0.844   |
|            |              |7               |0.0653856   |0.0565389     |175                 |147             |0.121925 |      |0.84    |
|            |              |8               |0.0657775   |0.0547833     |177                 |151             |0.120561 |      |0.853   |
|            |              |9               |0.0614345   |0.0572089     |179                 |150             |0.118643 |      |0.838   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.837         |                    |Mean Total Time |0.12308  |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|AKAZE       |SIFT          |1               |0.0673778   |0.0234936     |157                 |134             |0.0908714|      |0.854   |
|            |              |2               |0.0701857   |0.0240477     |161                 |134             |0.0942333|      |0.832   |
|            |              |3               |0.0657674   |0.0253002     |155                 |130             |0.0910676|      |0.839   |
|            |              |4               |0.0659044   |0.0252297     |163                 |136             |0.0911341|      |0.834   |
|            |              |5               |0.0636864   |0.023535      |164                 |137             |0.0872214|      |0.835   |
|            |              |6               |0.0647614   |0.0239678     |173                 |147             |0.0887292|      |0.85    |
|            |              |7               |0.0674355   |0.0241566     |175                 |147             |0.0915921|      |0.84    |
|            |              |8               |0.0667559   |0.0239209     |177                 |154             |0.0906768|      |0.87    |
|            |              |9               |0.0689788   |0.0265541     |179                 |151             |0.095533 |      |0.844   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.844         |                    |Mean Total Time |0.09123  |      |        |



                                                                                                            
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|FIELD9|Accuracy|
|------------|--------------|----------------|------------|--------------|--------------------|----------------|---------|------|--------|
|SIFT        |BRISK         |1               |0.0936595   |0.0017748     |132                 |64              |0.0954342|      |0.485   |
|            |              |2               |0.0914613   |0.00175812    |124                 |66              |0.0932194|      |0.532   |
|            |              |3               |0.0906672   |0.00180749    |137                 |62              |0.0924746|      |0.453   |
|            |              |4               |0.0918513   |0.0017713     |134                 |66              |0.0936226|      |0.493   |
|            |              |5               |0.0905167   |0.00199072    |140                 |59              |0.0925074|      |0.421   |
|            |              |6               |0.0918951   |0.00176536    |137                 |64              |0.0936604|      |0.467   |
|            |              |7               |0.0903629   |0.001881      |148                 |64              |0.0922439|      |0.432   |
|            |              |8               |0.0927862   |0.00199492    |159                 |67              |0.0947811|      |0.421   |
|            |              |9               |0.0906954   |0.00178016    |137                 |80              |0.0924755|      |0.584   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.476         |                    |Mean Total Time |0.09338  |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|SIFT        |BRIEF         |1               |0.0975213   |0.000939027   |132                 |86              |0.0984603|      |0.652   |
|            |              |2               |0.0926043   |0.000910491   |124                 |78              |0.0935148|      |0.629   |
|            |              |3               |0.0911639   |0.000915715   |137                 |76              |0.0920796|      |0.555   |
|            |              |4               |0.0919889   |0.000964333   |134                 |85              |0.0929532|      |0.634   |
|            |              |5               |0.0915156   |0.000923309   |140                 |69              |0.0924389|      |0.493   |
|            |              |6               |0.0904579   |0.000904412   |137                 |74              |0.0913623|      |0.54    |
|            |              |7               |0.090675    |0.000941262   |148                 |76              |0.0916163|      |0.514   |
|            |              |8               |0.0914952   |0.000978198   |159                 |70              |0.0924734|      |0.44    |
|            |              |9               |0.0910361   |0.000900201   |137                 |88              |0.0919363|      |0.642   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.567         |                    |Mean Total Time |0.09298  |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|SIFT        |FREAK         |1               |0.0980496   |0.0437466     |132                 |64              |0.141796 |      |0.485   |
|            |              |2               |0.092671    |0.0438197     |124                 |72              |0.136491 |      |0.581   |
|            |              |3               |0.0908002   |0.0439002     |137                 |65              |0.1347   |      |0.474   |
|            |              |4               |0.0924165   |0.0443042     |134                 |66              |0.136721 |      |0.493   |
|            |              |5               |0.0934186   |0.0439333     |140                 |63              |0.137352 |      |0.45    |
|            |              |6               |0.0931045   |0.0454318     |137                 |58              |0.138536 |      |0.423   |
|            |              |7               |0.0944624   |0.0439843     |148                 |64              |0.138447 |      |0.432   |
|            |              |8               |0.0944175   |0.0434288     |159                 |65              |0.137846 |      |0.409   |
|            |              |9               |0.0913999   |0.0435665     |137                 |79              |0.134966 |      |0.577   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.48          |                    |Mean Total Time |0.13743  |      |        |
|            |              |                |            |              |                    |                |         |      |        |
|detectorType|descriptorType|imageIndex      |detectorTime|descriptorTime|detectectedKeypoints|matchedKeypoints|totalTime|      |Accuracy|
|SIFT        |SIFT          |1               |0.0953738   |0.0759936     |132                 |82              |0.171367 |      |0.621   |
|            |              |2               |0.0943037   |0.0753263     |124                 |81              |0.16963  |      |0.653   |
|            |              |3               |0.093405    |0.0773558     |137                 |85              |0.170761 |      |0.62    |
|            |              |4               |0.0963032   |0.0781313     |134                 |93              |0.174435 |      |0.694   |
|            |              |5               |0.0935031   |0.0771011     |140                 |90              |0.170604 |      |0.643   |
|            |              |6               |0.0966461   |0.0783181     |137                 |81              |0.174964 |      |0.591   |
|            |              |7               |0.0972709   |0.0780169     |148                 |82              |0.175288 |      |0.554   |
|            |              |8               |0.0932585   |0.0781826     |159                 |102             |0.171441 |      |0.642   |
|            |              |9               |0.0935898   |0.0765319     |137                 |104             |0.170122 |      |0.759   |
|            |              |                |            |              |                    |                |         |      |        |
|            |              |Mean Accuracy = |            |0.642         |                    |Mean Total Time |0.17207  |      |        |
