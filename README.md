# Absolute Pose From One or Two Scaled and Oriented Features

This repository provides code for our CVPR 2024 paper "Absolute Pose From One or Two Scaled And Oriented Features."

Ventura, Jonathan, Kueklova, Zuzana, Sattler, Torsten and Baráth, Dániel.  Absolute Pose from One or Two Scaled and Oriented Features.  Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition, 2024.

### Abstract ###

Keypoints used for image matching often include an estimate of the feature scale and orientation.  While recent work has demonstrated the advantages of using feature scales and orientations for relative pose estimation, relatively little work has considered their use for absolute pose estimation.   We introduce minimal solutions for absolute pose from two oriented feature correspondences in the general case, or one scaled and oriented correspondence given a known vertical direction.   Nowadays, assuming a known direction is not particularly restrictive as modern consumer devices, such as smartphones or drones, are equipped with Inertial Measurement Units (IMU) that provide the gravity direction by default.  Compared to traditional absolute pose methods requiring three point correspondences, our solvers need a smaller minimal sample, reducing the cost and complexity of robust estimation.  Evaluations on large-scale and public real datasets demonstrate the advantage of our methods for fast and accurate localization in challenging conditions.

### Dependencies ###

- [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
- [PoseLib](https://github.com/PoseLib/PoseLib)

### Code ###

The P2ORI solver computes absolute pose from two oriented feature correspondences.  The function `P2ORI::solve()` accepts the following inputs:

* `R_ref` rotation of reference image (world-to-camera)
* `t_ref` translation of reference image (world-to-camera)
* `s_ref`, `c_ref` sine and cosine of feature orientation in reference image
* `s_query`, `c_query` sine and cosine of feature orientation in reference image
* `p_ref` 2D point observation in reference image
* `d` depth of point in reference image
* `n` surface normal in world coordinate system
* `p_query` 2D point observation in query image

and outputs at most eight rotation and translation solutions for the query image in world-to-camera format.

The UP1SIFT solver computes absolute pose from one scaled and oriented feature correspondence, assuming knowledge of the gravity direction.  The function `UP1SIFT::solve()` accepts the same inputs as P2ORI, plus these additional arguments:

* `q` ratio of feature scales (scale in query image / scale in reference image)
* `Rxz` query image gravity rotation such that R\_query=Ry*Rxz

and outputs at most two rotation and translation solutions for the query image in world-to-camera format.

An example program is provided in `examples/test_random_problems.cpp` which tests the solvers on random synthetic data problems.

### Reference ###

    @InProceedings{Ventura_2024_CVPR,
        author    = {Ventura, Jonathan and Kukelova, Zuzana and Sattler, Torsten and Bar\'ath, D\'aniel},
        title     = {Absolute Pose from One or Two Scaled and Oriented Features},
        booktitle = {Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)},
        month     = {June},
        year      = {2024},
    }
