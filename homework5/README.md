# homework5

### Whitted风格的光线追踪
- 每个像素点的着色：从相机连透过此像素点，打到三维空间上第一个接触到的物体，然后尝试连线光源观察是否有遮挡，同时根据物体的材质，决定是否计算折射与反射路径。最终像素点的着色由所有不被遮挡的路径加权。
- 需要求解【射线是否与三角形有交点？】，Moller-Trumbore算法计算。
- 交点应当在射线上，同时应当在三角形所在平面(用重心坐标写法)。O+td = (1-u-v)p0+ u*p1 + v*p2, O射线起点，d射线方向，p0,p1,p2为三角形顶点。
- 详见 https://www.bilibili.com/video/BV1X7411F744?p=13  0:57:17