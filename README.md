# ikd-Tree
**ikd-Tree** is an incremental k-d tree designed for robotic applications. The ikd-Tree incrementally updates a k-d tree with new coming points only, leading to much lower computation time than existing static k-d trees. Besides point-wise operations, the ikd-Tree supports several features such as box-wise operations and down-sampling that are practically useful in robotic applications.



### Developers

[Yixi Cai 蔡逸熙](https://github.com/Ecstasy-EC): Data structure design and implementation

[Wei Xu 徐威](https://github.com/XW-HKU): Incorporation into LiDAR-inertial odometry package ([FAST_LIO2 Released](https://github.com/hku-mars/FAST_LIO))

More details please refer to our paper and video~



### Related paper

**Related paper** available on arxiv:

[ikd-Tree: An Incremental K-D Tree for robotic applications](https://arxiv.org/abs/2102.10808)

**Related video**: https://youtu.be/ueOunk03zxA



## Version 2

- We upgraded our **ikd-Tree** to achieve a more stable and efficient performance. More details are shown as follows:
  - Replace the queue and priority queue in STL with our code to avoid memory conflicts.
  - Fix some bugs in re-building of the previous version, which may result in information loss during the multi-thread re-building process. 
  - Add a new parameter `max_dist` to support ranged search to achieve faster nearest search in robotic applications.
  - Fix minor bugs to improve the overall performance. 


## Build & Run demo
### 1. How to build this project
```bash
cd ~/catkin_ws/src
git clone git@github.com:hku-mars/ikd-Tree.git
cd ikd-Tree/build
cmake ..
make -j 9
```
### 2. Run our examples

**Note: To run Example 2 & 3, please download the PCD file ([HKU_demo_pointcloud](https://drive.google.com/file/d/1tMYiBIFn-fcjisaoIrmIKA09NICGG9KJ/view?usp=sharing))  into`${Your own directory}/ikd-Tree/materials`**

```bash
cd ${Your own directory}/ikd-Tree/build
# Example 1. Check the speed of ikd-Tree
./ikd_tree_demo
# Example 2. Searching-points-by-box examples
./ikd_Tree_Search_demo
# Example 3. An aysnc. exmaple for readers' better understanding of the principle of ikd-Tree
./ikd_tree_async_demo
```

**Example 2: ikd_tree_Search_demo** 

Box Search Result  |   Radius Search Result
:-------------------------:|:-------------------------:
![](materials/imgs/Box_Search.png) |  ![](materials/imgs/Radius_Search.png)

Points returned from the two search methods are shown in red.

**Example 3: ikd_tree_Async_demo**

Original Map:

<img src="materials/imgs/HKU_campus.png" style="zoom:50%;" />



Box Delete Results:

Points removed from ikd-Tree(red) |       Map after box delete        
:-------------------------:|:-------------------------:
![](materials/imgs/removed.png) |  ![](materials/imgs/remain.png)

This example is to demonstrate the asynchronous phenomenon in ikd-Tree. The points are deleted by attaching 'deleted' on the tree nodes (map shown in the ) instead of being removed from the ikd-Tree immediately. They are removed from the tree when rebuilding process is performed. Please refer to our paper for more details about delete and rebuilding.


## Acknowledgments
Thanks [Marcus Davi](https://github.com/Marcus-Davi) for helps in templating the ikd-Tree for more general applications.

Thanks [Hyungtae Lim 임형태](https://github.com/LimHyungTae) for providing application examples on point clouds. 

