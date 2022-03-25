# homework3
- 重写rasterize_triangle，三角形内，每个pixel的颜色通过顶点的信息，用重心坐标插值得到。
- 值得注意的是坐标使用的是screen space还是camera space，一些计算，例如Blinn-Phong计算光照时需要使用变形前的场景中的坐标来计算距离和光照。
- 写几种片元着色器：分别使用纹理着色，布林-冯(Blinn-Phong)光照模型着色，法线着色，凹凸贴图着色(Bump)和它们的组合。

