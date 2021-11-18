# homework1
### MVP过程的矩阵变换
从空间中物体->屏幕上成像，中间过程中涉及坐标变换，坐标变换部分，可分为三步，MVP。

Model transformations：场景中的模型进旋转，缩放，平移。

View transformations：令摄像机为标准位置时，场景物体坐标的变换过程，即计算相对相机的坐标。通常定义摄像机的位置为e，视野方向(Look-at/gaze direction)g，向上方向(Up direction)t。一般来说：

- 将e变为原点，平移
- 旋转g到-Z方向
- 旋转t到Y方向
- 旋转(g x t)到X方向


Projection transformations：投影变换包含两种投影方式：正交投影(Orthographic projection)和透视投影(Perspective projection)。
- 正交投影
  - 对于任意一个空间中的立方体空间[l,r] * [b,t]*[f,n]映射到标准立方体(canonical cube)[-1,1] * [-1,1] * [-1,1],平移 & 缩放。注：此处为右手系，OPENGL是左手系。物体原本的形状会被改变，之后的视口变换会处理这个问题。
- 透视投影
  - 透视投影关注的空间不再是长方体，而是一个Frustum。首先将Frustum，变换成一个Cuboid长方体，后转换为正交投影问题。
  - Frustum的由两个概念定义/决定：宽高比(Aspect ratio)和视角(Field of View, fov)。
    - Aspect ratio = width / height
    - fovY： 摄像机，上边中点与下中点边所成角,垂直可视角度，fov一般默认垂直可视角度。
    - fovX：摄像机，左边中点与右边中点所成角，水平可视角度。

### Build & Run
mkdir build

cd build

cmake ..

make -j4

./Rasterizer

