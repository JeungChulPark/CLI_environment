# B-EXTRA E.5 -- eliminate the duplicate feature extraction on the ORB-SLAM3 odometry path

## The duplication, verified

`Mem/UseOdomFeatures` (Parameters.h:234, default **true**) exists exactly to stop RTAB-Map
re-extracting features the odometry already computed. The gate is
`corelib/src/Memory.cpp:4871-4875`:

```cpp
	if(!_useOdometryFeatures ||
		data.keypoints().empty() ||
		(int)data.keypoints().size() != data.descriptors().rows ||
		(_feature2D->getType() == Feature2D::kFeatureOrbOctree && data.descriptors().empty()))
	{
		// ... full re-extraction ...
```

`data.setFeatures(...)` is called by only three odometry implementations:

```
corelib/src/odometry/OdometryF2F.cpp:302
corelib/src/odometry/OdometryF2M.cpp:338, 1264, 1299
corelib/src/Odometry.cpp:782                 (the Odom/ImageDecimation rescale path)
```

`corelib/src/odometry/OdometryORBSLAM3.cpp` has **no setFeatures call**. It fills only
`info->words` (:564) and `info->localMap` (:584), which are visualisation channels that
`Memory` never reads.

**So with `Odom/Strategy=5`, features are extracted twice on every mapped frame.**

## Actual cost -- smaller than it looks, but it is a landmine

| Rtabmap/DetectionRate | Extra extractions/s | Impact |
|---|---|---|
| 1 (default) | 1 | negligible (~4-10 ms once per second) |
| 5 | 5 | noticeable on the mapping thread |
| 30 | 30 | **~120-300 ms/s wasted; breaks the mapping thread** |

Anyone chasing "30 FPS mapping" will raise DetectionRate and hit this.

## Why it is not a one-line fix

RTAB-Map's vocabulary (`VWDictionary` + FLANN over `Kp/DetectorStrategy` descriptors) and
ORB-SLAM3's DBoW2 vocabulary are different objects built over different descriptor
distributions. The keypoints could be shared; the *dictionary* cannot.

More practically: `ORB_SLAM3::System` exposes `GetTrackedKeyPointsUn()` (used at
OdometryORBSLAM3.cpp:547) but **no descriptor getter**. The descriptors exist in
`Tracking::mCurrentFrame.mDescriptors` and are simply not surfaced through `System`.

## Sketch

1. In the matlabbe ORB-SLAM3 fork, add to `System`:
   ```cpp
   cv::Mat GetTrackedDescriptors();   // returns mpTracker->mCurrentFrame.mDescriptors.clone()
   ```
2. In `OdometryORBSLAM3::computeTransform`, after the existing `GetTrackedMapPoints()` /
   `GetTrackedKeyPointsUn()` calls (around :547), build the 3-D points from the depth
   image and publish them:
   ```cpp
   std::vector<cv::KeyPoint> kptsUn = orbslam_->GetTrackedKeyPointsUn();
   cv::Mat desc = orbslam_->GetTrackedDescriptors();
   if((int)kptsUn.size() == desc.rows && !kptsUn.empty())
   {
       // reuse RTAB-Map's own RGB-D 3D-ization so the convention matches exactly
       std::vector<cv::Point3f> kpts3D = util3d::generateKeypoints3DDepth(
               kptsUn, data.depthRaw(), data.cameraModels(), 0.0f, 0.0f);
       data.setFeatures(kptsUn, kpts3D, desc);
   }
   ```
3. Set `--Kp/DetectorStrategy 2` (ORB) so `Memory`'s configured type is consistent.

## Two hard constraints -- read before attempting this

- **`Memory::parseParameters` will disable `Mem/UseOdomFeatures` if the types disagree**
  (`corelib/src/Memory.cpp:833-849`):
  ```cpp
  	if(visFeatureType != kpDetectorStrategy)
  	{
  		UWARN("%s is enabled, but %s and %s parameters are not the same! Disabling %s...", ...);
  		_useOdometryFeatures = false;
  ```
  So `Vis/FeatureType` and `Kp/DetectorStrategy` must both be 2 (ORB). Note this is a
  slight fiction -- ORB-SLAM3's ORBextractor is not cv::ORB -- but the descriptors are
  both 256-bit BRIEF-family and Hamming-comparable, so the vocabulary works.

- **Keypoints must be in the SAME image frame RTAB-Map expects.**
  `GetTrackedKeyPointsUn()` returns *undistorted* coordinates. With
  `Rtabmap/ImagesAlreadyRectified=true` (Parameters.h:193, default true) that matches.
  If you ever set it false, the coordinate conventions diverge and this patch is wrong.

## Recommendation

Unless you are raising `Rtabmap/DetectionRate` above ~5 Hz, **do not do this**. The cost
at the default 1 Hz is real but negligible, and the change requires forking ORB-SLAM3.
Document the landmine instead, and revisit only if DetectionRate goes up.

## Separately -- a better use of the same effort

`OdometryORBSLAM3.cpp:514-534` hard-codes the odometry covariance to `(baseline/8)^2`
(and 1e-4 on rotation), identical for a perfectly-tracked frame and a barely-tracked one.
RTAB-Map builds every graph edge's information matrix from this
(`Optimizer/VarianceIgnored=false`, Parameters.h:430), so **every odometry edge carries
the same weight**. `mapPoints.size()` (:480) and the inlier count (:574) are already in
scope. Scaling `linearVar` by the inlier ratio is ~15 lines and would materially improve
the optimised trajectory. Do this first.

## Also worth fixing while you are in this file

`OdometryORBSLAM3.cpp:458` -- copy-paste bug in the stereo path:
```cpp
	cv::Mat rightMono = data.rightRaw();
	if(data.rightRaw().channels() == 3) {
		rightMono = cv::Mat();
		cv::cvtColor(data.imageRaw(), rightMono, CV_BGR2GRAY);   // <-- should be data.rightRaw()
	}
```
The LEFT image is converted into `rightMono`. Only triggers for 3-channel stereo input.
